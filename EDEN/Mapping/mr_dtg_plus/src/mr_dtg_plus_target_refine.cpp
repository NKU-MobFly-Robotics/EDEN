#include <mr_dtg_plus/mr_dtg_plus.h>
using namespace DTGPlus;

void MultiDtgPlus::ReFineTargetVp(const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, 
    const Eigen::Vector4d &vp2, const uint32_t &eroi_id, const uint8_t &vp_id, 
    const vector<Eigen::Vector3d> &path2v2, const double &motion_gain, const double &vm, const double &arc,
    Eigen::Vector4d &refined_vp1, vector<Eigen::Vector3d> &path2v1_r, vector<Eigen::Vector3d> &path2v2_r){
    double best_gain = motion_gain, better_gain;
    double g0 = EROI_->GetGain(eroi_id, vp_id);
    

    Eigen::Vector3d eroi_up, eroi_down;
    Eigen::Vector4d r_vp1;
    eroi_up = EROI_->EROI_[eroi_id].center_ + EROI_->node_scale_ * 0.5;
    eroi_down = EROI_->EROI_[eroi_id].center_ - EROI_->node_scale_ * 0.5;
    cout<<"vp2:"<<vp2.transpose()<<endl;
    cout<<"motion_gain:"<<motion_gain<<endl;
    cout<<"g0:"<<g0<<endl;
    
    for(int i = 1; i < path2v2.size(); i++){
        if(i >= refine_num_ + 1) break;
        double arc_i = arc * (path2v2.size() - i - 1) / path2v2.size();

        cout<<"arc:"<<arc<<endl;
        cout<<"arc_i:"<<arc_i<<endl;

        ReFineTargetVpSample(ps, vs, path2v2[i], vp2, eroi_id, vm, arc_i, g0, best_gain, r_vp1, better_gain);
        cout<<"better_gain:"<<better_gain<<endl;
        if(better_gain > 0){
            cout<<"=================================i:"<<i<<endl;
            refined_vp1 = r_vp1;
            best_gain = better_gain;
            path2v2_r.clear();
            for(int j = 0; j < path2v2.size(); j++) path2v2_r.emplace_back(path2v2[j]);
            Eigen::Vector3i si = LRM_->GlobalPos2GlobalId3(path2v2[i]);
            double l;
            LRM_->RetrieveHPath(si, path2v1_r, l, true);
        }
    }
}

void MultiDtgPlus::ReFineTargetVpSample(const Eigen::Vector4d &ps, const Eigen::Vector4d &vs, 
    const Eigen::Vector3d &r_vp_pos, const Eigen::Vector4d &vp2, const uint32_t &eroi_id, 
    const double &vm, const double &arc, const double &gain_0, const double &motion_gain, 
    Eigen::Vector4d &r_vp, double &better_gain){
    better_gain = -1;
    double dist;
    if(!LRM_->Connected(r_vp_pos)){
        ROS_ERROR("---");
        return;
    }
    else{
        auto lrn = LRM_->GlobalPos2LocalNode(r_vp_pos);
        dist = lrn->path_g_;
    }

    double cam_hor_ang, cam_ver_ang, ang_det;
    EROI_->GetCamFov(cam_hor_ang, cam_ver_ang);
    int cam_hor_num = ceil(cam_hor_ang / yaw_slice_ang_);
    ang_det = cam_hor_ang / cam_hor_num;

    vector<pair<double, double>> gains; // unknown v, unknown in eroi v
    Eigen::Vector3d dir, ray_end;
    double cos_phi;
    double g, ang_g;
    double sin_2_dphi = sin(ang_det / 2);
    double vn, r, c_g, c_ge, r_l;
    bool ray_in_eroi;
    VoxelState state;

    list<Eigen::Vector3d> ray;

    Eigen::Vector3d eroi_up, eroi_down;
    eroi_up = EROI_->EROI_[eroi_id].center_ + EROI_->node_scale_ * 0.5;
    eroi_down = EROI_->EROI_[eroi_id].center_ - EROI_->node_scale_ * 0.5;
    for(double ang_h = ang_det * 0.5; ang_h < M_PI * 2; ang_h += ang_det){
        gains.emplace_back(0, 0);
        for(double ang_v = ang_det * 0.5 - 0.5 * cam_ver_ang; ang_v < 0.5 * cam_ver_ang; ang_v += ang_det){
            dir(0) = cos(ang_h) * cos(ang_v);
            dir(1) = sin(ang_h) * cos(ang_v);
            dir(2) = sin(ang_v);
            cos_phi = cos(ang_v);
            ang_g = ang_det*sin_2_dphi*cos_phi;
            ray_end = r_vp_pos + dir * sensor_range_;
            BM_->GetCastLine(r_vp_pos, ray_end, ray);
            vn = 0;
            c_g = 0;
            c_ge = 0;
            r = 0;
            r_l = 0;
            for(auto &p : ray){
                if(!EROI_->InsideExpMap(p)) break;

                state = BM_->GetVoxState(p);
                r = (r_vp_pos - p).norm();
                vn += 1;
                if(r - r_l > 0.4) {
                    g = (2*pow(r*0.5 + 0.5*r_l, 2)*(r - r_l) + 1.0/6*pow(r - r_l, 3)) * ang_g;
                    // cout<<"g:"<<g<<"  c_ge:"<<c_ge<<"  c_g:"<<c_g<<"   vn:"<<vn<<endl;
                    gains.back().first += g * c_g / vn;
                    gains.back().second += g * c_ge / vn;
                    r_l = r;
                    c_g = 0, vn = 0, c_ge = 0;
                }
                if(state == VoxelState::free){
                    continue;
                }
                else if(state == VoxelState::occupied || state == VoxelState::out){
                    break;
                }
                else{
                    if(p(0) > eroi_up(0) || p(0) < eroi_down(0) || p(1) > eroi_up(1) || p(1) < eroi_down(1) || p(2) > eroi_up(2) || p(2) < eroi_down(2)){
                    }
                    else{
                        c_ge += 1;
                    }
                    c_g += 1;
                }
            }
        }
    }

    // /* init first vp gain */
    Eigen::Vector3d vp_diff;
    Eigen::Matrix3d R;
    vp_diff = vp2.head(3) - r_vp_pos;
    double gain = 0, gain_e = 0, gain_exp;
    double best_gain = -1;
    double yaw = cam_hor_num * 0.5 * ang_det;
    double best_yaw;
    cout<<"cam_hor_num:"<<cam_hor_num<<endl;
    for(int i = 0; i < cam_hor_num; i++){
        gain += gains[i].first;
        gain_e += gains[i].second;
    }
    R << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
    cout<<"gain_0:"<<gain_0<<endl;
    cout<<"gain_e:"<<gain_e<<endl;
    cout<<"EROI_->InsideFov(R, vp_diff):"<<EROI_->InsideFov(R, vp_diff)<<endl;
    if(gain_e > gain_0 * g_thr_fac_ && EROI_->InsideFov(R, vp_diff)){
        best_gain = gain;
        best_yaw = yaw;
    }
    // dyaw = abs(YawDiff(y0, yaw));
    // if(dyaw < yaw_thresh){
    //     best_yaw = yaw;
    //     best_gain = gain;
    // }

    // /* find best */
    int delet_idx, add_idx;


    for(int i = 1; i < gains.size(); i++){
        delet_idx = i - 1;
        add_idx = delet_idx + cam_hor_num;
        if(add_idx >= gains.size()) add_idx -= gains.size();
        gain += gains[add_idx].first;
        gain -= gains[delet_idx].first;
        gain_e += gains[add_idx].second;
        gain_e -= gains[delet_idx].second;
        yaw = (i + cam_hor_num * 0.5) * ang_det;
        R << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
        if(EROI_->InsideFov(R, vp_diff)){
            cout<<"gain_0 * g_thr_fac_:"<<gain_0 * g_thr_fac_<<endl;
            cout<<"gain_e:"<<gain_e<<endl;
            cout<<"gain:"<<gain<<endl;
            cout<<"yaw:"<<yaw<<endl;
            cout<<"vp_diff:"<<vp_diff.transpose()<<" vp2:"<<vp2.transpose()<<" r_vp_pos:"<<r_vp_pos.transpose()<<endl;
        }

        if(gain_e > gain_0 * g_thr_fac_ && EROI_->InsideFov(R, vp_diff)){
            Eigen::Vector4d vp1_new;
            vp1_new.head(3) = r_vp_pos;
            vp1_new(3) = yaw;
            Eigen::Vector3d vs3 = vs.head(3);
            Eigen::Vector3d v2_pos = vp2.head(3);
            double mg = GetGainExp2(ps, vp1_new, vs3, dist, v2_pos, arc, vm, vp2(3));

            Eigen::Vector3d dp = (vp1_new.head(3) - ps.head(3)).normalized();
            Eigen::Vector3d dv = dp * v_max_ - vs.head(3);
            cout<<"dp1:"<<dp.transpose()<<" dv1:"<<dv.transpose()<<endl;
            double t = max(dist / v_max_ + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(vp1_new(3), ps(3))) / yv_max_ * yaw_gain_);
            t = max(dist / v_max_ + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(vp1_new(3), ps(3))) / yv_max_ * yaw_gain_);
            cout<<"ty1:"<<abs(EROI_->YawDiff(vp1_new(3), ps(3))) / yv_max_ * yaw_gain_<<endl;
            cout<<"g1:"<<exp(-lambda_e_ * t)<<endl;
            dv = (v2_pos - vp1_new.head(3)).normalized()*v_max_ - (vp1_new.head(3) - ps.head(3)).normalized()*v_max_;
            cout<<"dp2:"<<arc<<" dv2:"<<dv.transpose()<<endl;
            t += max(arc / vm + dv.norm() / a_max_ * acc_gain_, abs(EROI_->YawDiff(vp2(3), vp1_new(3))) / yv_max_ * yaw_gain_);
            cout<<"g2:"<<exp(-lambda_e_ * t)<<endl;
            cout<<"ty2:"<<abs(EROI_->YawDiff(vp2(3), vp1_new(3))) / yv_max_ * yaw_gain_<<endl;




            // cout<<"ps:"<<ps.transpose()<<endl;
            // cout<<"vp1_new:"<<vp1_new.transpose()<<endl;
            // cout<<"vs3:"<<vs3.transpose()<<endl;
            cout<<"dist:"<<dist<<endl;
            cout<<"mg:"<<mg<<endl;
            cout<<"motion_gain:"<<motion_gain<<endl;
            if(mg > motion_gain){
                better_gain = mg;
                r_vp = vp1_new;
            }
            best_gain = gain;
            // best_yaw = yaw;
        }

    }
    // cout<<"best_gain2:"<<best_gain<<endl;
    // if(best_gain > 0){
    //     Eigen::Vector4d vp1_new;
    //     vp1_new.head(3) = r_vp_pos;
    //     vp1_new(3) = best_yaw;
    //     Eigen::Vector3d vs3 = vs.head(3);
    //     double mg = GetGainExp2(ps, vp1_new, vs3, dist, EROI_->EROI_[eroi_id].center_, arc, vm, vp2(3));
    //     cout<<"ps:"<<ps.transpose()<<endl;
    //     cout<<"vp1_new:"<<vp1_new.transpose()<<endl;
    //     cout<<"vs3:"<<vs3.transpose()<<endl;
    //     cout<<"mg:"<<mg<<endl;
    //     cout<<"motion_gain:"<<motion_gain<<endl;
    //     if(mg > motion_gain){
    //         better_gain = mg;
    //         r_vp = vp1_new;
    //     }
    // }
    // return best_gain;


}