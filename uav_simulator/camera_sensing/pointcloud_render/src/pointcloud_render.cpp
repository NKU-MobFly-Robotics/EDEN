#include "pointcloud_render.h"

void PointcloudRender::initialize(ros::NodeHandle &nh) {
  // Read render parameters
  nh.getParam("/pointcloud_render/render_rate", render_rate);
  nh.getParam("/pointcloud_render/sensor_pose_rate", sensor_pose_rate);
  nh.getParam("/pointcloud_render/inflate_radius", inflate_radius);
  nh.getParam("/pointcloud_render/verbose", verbose);

  // Read camera parameters
  nh.getParam("/uav_model/sensing_parameters/image_width", width);
  nh.getParam("/uav_model/sensing_parameters/image_height", height);
  nh.getParam("/uav_model/sensing_parameters/camera_intrinsics/fx", fx);
  nh.getParam("/uav_model/sensing_parameters/camera_intrinsics/fy", fy);
  nh.getParam("/uav_model/sensing_parameters/camera_intrinsics/cx", cx);
  nh.getParam("/uav_model/sensing_parameters/camera_intrinsics/cy", cy);
  nh.getParam("/uav_model/sensing_parameters/max_depth", sensing_horizon);

  depthrender.set_para(fx, fy, cx, cy, width, height, inflate_radius);

  cam02body << 0.0, 0.0, 1.0, 0.0, //
      -1.0, 0.0, 0.0, 0.0,         //
      0.0, -1.0, 0.0, 0.0,         //
      0.0, 0.0, 0.0, 1.0;

  // init cam2world transformation
  cam2world = Matrix4d::Identity();

  odom_sub = nh.subscribe("/uav_simulator/odometry", 50, &PointcloudRender::odometryCallbck, this);

  // publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("/uav_simulator/depth_image", 1000);
  pub_color = nh.advertise<sensor_msgs::Image>("/uav_simulator/depth_image_color", 1000);
  pub_pose = nh.advertise<geometry_msgs::TransformStamped>("/uav_simulator/sensor_pose", 1000);
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl", 1);
  pub_caminfo = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
  pub_odom = nh.advertise<nav_msgs::Odometry>("/uav_simulator/sensor_odom", 1000);

  local_sensing_timer =
      nh.createTimer(ros::Duration(1.0 / render_rate), &PointcloudRender::renderCallback, this);
  estimation_timer =
      nh.createTimer(ros::Duration(1.0 / sensor_pose_rate), &PointcloudRender::pubCameraPose, this);

  _inv_resolution = 1.0 / _resolution;

  init_pose = false;
  init_cloud = false;
}

void PointcloudRender::setPointcloud(const pcl::PointCloud<pcl::PointXYZ> &cloud, bool add_floor) {
  if (cloud.points.size() == 0)
    return;
  for (int i = 0; i < int(cloud.points.size()); i++) {
    pcl::PointXYZ pt_in = cloud.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }

  if (add_floor) {
    Eigen::Vector2d mmin(0, 0), mmax(0, 0);
    for (const pcl::PointXYZ &pt : cloud) {
      mmin[0] = min(mmin[0], double(pt.x));
      mmin[1] = min(mmin[1], double(pt.y));
      mmax[0] = max(mmax[0], double(pt.x));
      mmax[1] = max(mmax[1], double(pt.y));
    }

    pcl::PointCloud<pcl::PointXYZ> floor_cloud;
    for (double x = mmin[0]; x < mmax[0]; x += 0.02) {
      for (double y = mmin[1]; y < mmax[1]; y += 0.02) {
        floor_cloud.push_back(pcl::PointXYZ(x, y, 0));
      }
    }

    for (const pcl::PointXYZ &pt : floor_cloud) {
      cloud_data.push_back(pt.x);
      cloud_data.push_back(pt.y);
      cloud_data.push_back(pt.z);
    }
  }

  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  init_cloud = true;

  if (verbose)
    ROS_INFO("[PointcloudRender] Set pointcloud with %lu points.", cloud.points.size());
}

void PointcloudRender::odometryCallbck(const nav_msgs::Odometry &odom) {
  /*if(!has_global_map)
    return;*/
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  request_pose.x() = odom.pose.pose.orientation.x;
  request_pose.y() = odom.pose.pose.orientation.y;
  request_pose.z() = odom.pose.pose.orientation.z;
  request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  // convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);

  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;

  init_pose = true;

  if (verbose)
    ROS_INFO_ONCE("[PointcloudRender] Received odometry message.");
}

void PointcloudRender::pubCameraPose(const ros::TimerEvent &event) {
  if (!init_pose)
    return;

  geometry_msgs::TransformStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.child_frame_id = "camera";
  camera_pose.transform.translation.x = cam2world(0, 3);
  camera_pose.transform.translation.y = cam2world(1, 3);
  camera_pose.transform.translation.z = cam2world(2, 3);
  camera_pose.transform.rotation.x = cam2world_quat.x();
  camera_pose.transform.rotation.y = cam2world_quat.y();
  camera_pose.transform.rotation.z = cam2world_quat.z();
  camera_pose.transform.rotation.w = cam2world_quat.w();
  camera_pose_ = camera_pose;
}

void PointcloudRender::renderCallback(const ros::TimerEvent &event) {
  if (!(init_pose && init_cloud))
    return;

  renderDepthImage();
  // renderDepthPointcloud();
}

void PointcloudRender::renderDepthPointcloud() {
  // for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for (int u = 0; u < width; u++)
    for (int v = 0; v < height; v++) {
      float depth = depth_mat.at<float>(v, u);

      if (depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;

      pose_in_world = cam2world * pose_in_camera;

      if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon)
        continue;

      pose_pt = pose_in_world.head(3);
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id = "world";
  local_map_pcl.header.stamp = last_odom_stamp;

  pub_pcl_wolrd.publish(local_map_pcl);
}

void PointcloudRender::renderDepthImage() {
  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      pose[j + 4 * i] = cam_pose(i, j);
    }
  }

  depthrender.render_pose(pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 500.0f;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i, j) = depth;
    }
  if (verbose)
    ROS_INFO("[PointcloudRender] Pointcloud render cost %lf ms.",
             (ros::Time::now().toSec() - this_time) * 1000.0f);
  // printf("max_depth %lf.\n", max);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());
  camera_pose_.header.stamp = last_odom_stamp;

  pub_pose.publish(camera_pose_);
  pub_odom.publish(_odom);
  cv::Mat adjMap;
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  // cv::imshow("depth_image", adjMap);

  // For running Faster code, FST
  sensor_msgs::CameraInfo camera_info;
  camera_info.header.stamp = last_odom_stamp;
  camera_info.header.frame_id = "camera";
  camera_info.width = width;
  camera_info.height = height;
  camera_info.K[0] = fx;
  camera_info.K[4] = fy;
  camera_info.K[2] = cx;
  camera_info.K[5] = cy;
  camera_info.binning_x = 0;
  camera_info.binning_y = 0;
  pub_caminfo.publish(camera_info);
}
