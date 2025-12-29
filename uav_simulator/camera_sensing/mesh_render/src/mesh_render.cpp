#include "mesh_render/mesh_render.h"

MeshRender::MeshRender() {}

MeshRender::~MeshRender() {
  delete scene;
  delete renderer;

  Application &app = Application::GetInstance();
  app.OnTerminate();
}

int MeshRender::initialize(ros::NodeHandle &nh) {
  // Read render parameters
  std::string open3d_resource_path;
  nh.param("/mesh_render/open3d_resource_path", open3d_resource_path, std::string(""));
  nh.param("/mesh_render/enable_depth", enable_depth_, true);
  nh.param("/mesh_render/enable_color", enable_color_, false);
  nh.param("/mesh_render/render_rate", render_rate_, 10.0);
  nh.param("/mesh_render/verbose", verbose_, false);

  // Read sensor parameters
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/fx", fx_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/fy", fy_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/cx", cx_, 0.0);
  nh.param("/uav_model/sensing_parameters/camera_intrinsics/cy", cy_, 0.0);
  nh.param("/uav_model/sensing_parameters/image_width", width_, 0);
  nh.param("/uav_model/sensing_parameters/image_height", height_, 0);
  nh.param("/uav_model/sensing_parameters/min_depth", min_depth_, 0.01);
  nh.param("/uav_model/sensing_parameters/max_depth", max_depth_, 10.0);

  if (fx_ == 0.0 || fy_ == 0.0 || cx_ == 0.0 || cy_ == 0.0) {
    ROS_ERROR("fx, fy, cx, cy is empty!");
    return -1;
  }

  if (width_ == 0 || height_ == 0) {
    ROS_ERROR("width, height is empty!");
    return -1;
  }

  Eigen::Matrix3d camera_intrinsic;
  camera_intrinsic << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;

  // Read map parameters
  nh.param("/map_config/map_file", map_file_, std::string(""));
  nh.param("/map_config/scale", scale_, 1.0); // scale_ = 1: m, scale_ = 100: cm

  if (map_file_.empty()) {
    ROS_ERROR("map_file_ is empty!");
    return -1;
  }
  // if map_file is absolute path, use it directly
  if (map_file_[0] == '/') {
    ROS_INFO("[MeshRender] Using absolute path: %s", map_file_.c_str());
  } else {
    // if map_file is relative path, use ros package path
    std::string map_render_path;
    map_render_path = ros::package::getPath("map_render");
    map_file_ = map_render_path + "/resource/" + map_file_;
    ROS_INFO("[MeshRender] Using relative path: %s", map_file_.c_str());
  }

  // Get transformation matrix from parameter server
  if (!getTransformationMatrixfromConfig(nh, &T_m_w_, "/map_config/T_m_w"))
    return -1;
  if (!getTransformationMatrixfromConfig(nh, &T_b_c_, "/map_config/T_b_c"))
    return -1;

  if (verbose_) {
    std::cout << "*** Mesh Render Node Parameters ***" << std::endl;
    std::cout << "enable_depth: " << enable_depth_ << std::endl;
    std::cout << "enable_color: " << enable_color_ << std::endl;
    std::cout << "render_rate: " << render_rate_ << std::endl;
    std::cout << "map_file: " << map_file_ << std::endl;
    std::cout << "scale: " << scale_ << std::endl;
    std::cout << "fx: " << fx_ << std::endl;
    std::cout << "fy: " << fy_ << std::endl;
    std::cout << "cx: " << cx_ << std::endl;
    std::cout << "cy: " << cy_ << std::endl;
    std::cout << "width: " << width_ << std::endl;
    std::cout << "height: " << height_ << std::endl;
    std::cout << "T_m_w_: " << std::endl << T_m_w_ << std::endl;
    std::cout << "T_b_c_: " << std::endl << T_b_c_ << std::endl;
  }

  pose_sub_ = nh.subscribe("/uav_simulator/odometry", 1, &MeshRender::odometryCallback, this);

  depth_image_pub_ = nh.advertise<sensor_msgs::Image>("/uav_simulator/depth_image", 1);
  color_image_pub_ = nh.advertise<sensor_msgs::Image>("/uav_simulator/color_image", 1);
  sensor_pose_pub_ = nh.advertise<geometry_msgs::TransformStamped>("/uav_simulator/sensor_pose", 1);
  pub_caminfo = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
  pub_odom = nh.advertise<nav_msgs::Odometry>("/uav_simulator/sensor_odom", 1000);

  render_timer =
      nh.createTimer(ros::Duration(1.0 / render_rate_), &MeshRender::renderCallback, this);

  Application &app = Application::GetInstance();
  app.Initialize(open3d_resource_path.c_str());
  EngineInstance::EnableHeadless();
  renderer = new FilamentRenderer(EngineInstance::GetInstance(), width_, height_,
                                  EngineInstance::GetResourceManager());
  scene = new Open3DScene(*renderer);

  if (open3d::io::ReadTriangleModel(map_file_, model)) {
    ROS_INFO("[MeshRender] Read triangle model successfully");
  } else {
    ROS_ERROR("[MeshRender] Failed to read triangle model");
    return -1;
  }
  for (size_t i = 0; i < model.meshes_.size(); i++) {
    model.meshes_[i].mesh->Scale(1.0 / scale_, Eigen::Vector3d::Zero());
  }

  scene->AddModel("model", model);
  scene->GetCamera()->SetProjection(camera_intrinsic, min_depth_, max_depth_, width_, height_);

  init_odom_ = false;

  return 0;
}

// Input:
// T_w_b_: body pose from uav simulator
// Output:
// Depth image: rendered depth image from mesh file with input agent pose
// T_w_c_: camera pose
void MeshRender::odometryCallback(const nav_msgs::OdometryConstPtr &msg) {
  Eigen::Vector3d t_w_b_(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         msg->pose.pose.position.z);
  Eigen::Quaterniond q_w_b_(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Matrix3d R_w_b_ = q_w_b_.toRotationMatrix();
  T_w_b_.block<3, 3>(0, 0) = R_w_b_;
  T_w_b_.block<3, 1>(0, 3) = t_w_b_;
  T_w_b_(3, 3) = 1.0;

  latest_odometry_timestamp_ = msg->header.stamp;

  init_odom_ = true;
  odom_ = *msg;
}

void MeshRender::renderCallback(const ros::TimerEvent &event) {
  if (!init_odom_)
    return;

  // Under map frame
  Eigen::Vector3f camera_lookat_b(1.0, 0.0, 0.0);
  Eigen::Vector3f camera_up_b(0.0, 0.0, 1.0);

  Eigen::Vector3f camera_pos;
  Eigen::Vector3f camera_lookat;
  Eigen::Vector3f camera_up;

  // Get from world frame first
  camera_pos = T_w_b_.block<3, 1>(0, 3).cast<float>();
  camera_lookat = T_w_b_.block<3, 3>(0, 0).cast<float>() * camera_lookat_b +
                  T_w_b_.block<3, 1>(0, 3).cast<float>();
  camera_up = T_w_b_.block<3, 3>(0, 0).cast<float>() * camera_up_b;

  // Transform to map frame
  camera_pos =
      T_m_w_.block<3, 3>(0, 0).cast<float>() * camera_pos + T_m_w_.block<3, 1>(0, 3).cast<float>();
  camera_lookat = T_m_w_.block<3, 3>(0, 0).cast<float>() * camera_lookat +
                  T_m_w_.block<3, 1>(0, 3).cast<float>();
  camera_up = T_m_w_.block<3, 3>(0, 0).cast<float>() * camera_up;

  T_m_b_ = T_m_w_ * T_w_b_;

  if (verbose_) {
    std::cout << "T_w_b_: " << std::endl << T_w_b_ << std::endl;
    std::cout << "T_m_b_: " << std::endl << T_m_b_ << std::endl;
    std::cout << "camera_pos: " << std::endl << camera_pos << std::endl;
    std::cout << "camera_lookat: " << std::endl << camera_lookat << std::endl;
    std::cout << "camera_up: " << std::endl << camera_up << std::endl;
  }

  scene->GetCamera()->LookAt(camera_lookat, camera_pos, camera_up);

  Application &app = Application::GetInstance();

  if (enable_depth_) {
    ros::Time start_time_depth = ros::Time::now();
    std::shared_ptr<geometry::Image> depth_img = app.RenderToDepthImage(
        *renderer, scene->GetView(), scene->GetScene(), width_, height_, true);
    ros::Time end_time_depth = ros::Time::now();
    if (verbose_)
      ROS_INFO("[MeshRender] Depth image time: %f ms",
               (end_time_depth - start_time_depth).toSec() * 1000.0);

    cv::Mat depth_mat = cv::Mat::zeros(height_, width_, CV_32FC1);
    for (int i = 0; i < height_; i++)
      for (int j = 0; j < width_; j++) {
        float *value = depth_img->PointerAt<float>(j, i);
        depth_mat.at<float>(i, j) = *value < 500.0f ? *value : 500.0f;
      }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = latest_odometry_timestamp_;
    out_msg.header.frame_id = "camera";
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    out_msg.image = depth_mat.clone();
    depth_image_pub_.publish(out_msg.toImageMsg());
  }

  if (enable_color_) {
    ros::Time start_time_color = ros::Time::now();
    std::shared_ptr<geometry::Image> color_img =
        app.RenderToImage(*renderer, scene->GetView(), scene->GetScene(), width_, height_);
    ros::Time end_time_color = ros::Time::now();
    if (verbose_)
      ROS_INFO("[MeshRender] Color image time: %f ms",
               (end_time_color - start_time_color).toSec() * 1000.0);

    // Grey image
    cv::Mat color_mat = cv::Mat::zeros(height_, width_, CV_8UC3);
    for (int i = 0; i < height_; i++)
      for (int j = 0; j < width_; j++) {
        uint8_t *value0 = color_img->PointerAt<uint8_t>(j, i, 0);
        uint8_t *value1 = color_img->PointerAt<uint8_t>(j, i, 1);
        uint8_t *value2 = color_img->PointerAt<uint8_t>(j, i, 2);
        color_mat.at<cv::Vec3b>(i, j) = cv::Vec3b(*value2, *value1, *value0); // BGR
      }

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = latest_odometry_timestamp_;
    out_msg.header.frame_id = "camera";
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = color_mat.clone();
    color_image_pub_.publish(out_msg.toImageMsg());
  }
  sensor_msgs::CameraInfo camera_info;
  camera_info.header.stamp = latest_odometry_timestamp_;
  camera_info.header.frame_id = "camera";
  camera_info.width = width_;
  camera_info.height = height_;
  camera_info.K[0] = fx_;
  camera_info.K[4] = fy_;
  camera_info.K[2] = cx_;
  camera_info.K[5] = cy_;
  camera_info.binning_x = 0;
  camera_info.binning_y = 0;
  pub_caminfo.publish(camera_info);
  // Publish 
  T_w_c_ = T_w_b_ * T_b_c_;
  Eigen::Quaterniond q_w_c_(T_w_c_.block<3, 3>(0, 0));

  geometry_msgs::TransformStamped sensor_pose;
  sensor_pose.header.stamp = latest_odometry_timestamp_;
  sensor_pose.header.frame_id = "world";
  sensor_pose.child_frame_id = "camera";
  sensor_pose.transform.translation.x = T_w_c_(0, 3);
  sensor_pose.transform.translation.y = T_w_c_(1, 3);
  sensor_pose.transform.translation.z = T_w_c_(2, 3);
  sensor_pose.transform.rotation.w = q_w_c_.w();
  sensor_pose.transform.rotation.x = q_w_c_.x();
  sensor_pose.transform.rotation.y = q_w_c_.y();
  sensor_pose.transform.rotation.z = q_w_c_.z();

  sensor_pose_pub_.publish(sensor_pose);
  pub_odom.publish(odom_);
}

template <typename Scalar>
void MeshRender::xmlRpcToEigen(XmlRpc::XmlRpcValue &xml_rpc, Eigen::Matrix<Scalar, 4, 4> *eigen) {
  if (eigen == nullptr) {
    ROS_ERROR("[MeshRender] Null pointer given");
    return;
  }
  if (xml_rpc.size() != 4) {
    ROS_ERROR("[MeshRender] XmlRpc matrix has %d rows", xml_rpc.size());
    return;
  }
  // read raw inputs
  for (size_t i = 0; i < 3; ++i) {
    if (xml_rpc[i].size() != 4) {
      ROS_ERROR("[MeshRender] XmlRpc matrix has %d columns in its %ld row", xml_rpc[i].size(), i);
      return;
    }
    for (size_t j = 0; j < 3; ++j) {
      (*eigen)(i, j) = static_cast<double>(xml_rpc[i][j]);
    }
    (*eigen)(i, 3) = static_cast<double>(xml_rpc[i][3]);
  }
}

template <typename Scalar>
bool MeshRender::getTransformationMatrixfromConfig(ros::NodeHandle &nh,
                                                   Eigen::Matrix<Scalar, 4, 4> *T,
                                                   const std::string &param_name) {
  XmlRpc::XmlRpcValue T_xml;
  if (nh.getParam(param_name, T_xml)) {
    T->setIdentity();
    xmlRpcToEigen(T_xml, T);
    return true;
  } else {
    ROS_ERROR("[MeshRender] Failed to load %s from parameter server", param_name.c_str());
    return false;
  }
}

bool MeshRender::saveImage(const std::string &filename,
                           const std::shared_ptr<geometry::Image> &image) {
  ROS_INFO("[MeshRender] Writing image file to %s", filename.c_str());
  return io::WriteImage(filename, *image);
}
