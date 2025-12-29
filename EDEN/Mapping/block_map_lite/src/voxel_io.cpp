#include <block_map_lite/block_map_lite.h>

bool BlockMapLite::SaveData(const int bid, shared_ptr<Grid_Block> &GB){
    if(GB->state_ == UNKNOWN) return false; // dont save unknown GB

    mtx_.lock();
    if(!GB->show_) {
        changed_blocks_.emplace_back(bid);
        // cout<<"delete add change:"<<bid<<endl;
    }
    float thr_min = thr_min_ - 1e-3;
    string p = map_path_ + "/" + to_string(bid);
    mtx_.unlock();

    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    int data_size = ceil(voxel_num / 5);
    uint8_t data[data_size];
    uint8_t data_seg[5]; // for x and y, x <= 8. 3^(y=5) / 2^(x=8) is the maximum value, to minimize the data waste
    std::ofstream outfile(p.c_str(), ios::binary);
    if(!outfile.is_open()){
        ROS_ERROR("open file failed");
        ros::shutdown();
        // mtx_.unlock();
        return false;
    }
    
    uint64_t idx;
    for(int i = 0, j = 0; i < data_size; i++){
        for(int k = 0; k < 5; k++){
            if(j >= voxel_num){
                data_seg[k] = 0;
            }
            else{
                mtx_.lock();
                if(GB->odds_log_[j] > 0) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.erase(idx);
                    data_seg[k] = 0;
                }
                else if(GB->odds_log_[j] > thr_min) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.erase(idx);
                    data_seg[k] = 1;
                }
                else {
                    data_seg[k] = 2;
                }
                mtx_.unlock();
            }
            j++;
        }
        data[i] = EncodeVoxel(data_seg);
    }
    outfile.write(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
    outfile.close();
    return true;
}

bool BlockMapLite::ReadData(const int bid, shared_ptr<Grid_Block> &GB){
    mtx_.lock();
    GB = make_shared<Grid_Block>();
    if(bid < 0){
        GB = NULL;
        mtx_.unlock();
        return false;
    }

    float thr_max = thr_max_ - 1e-3, thr_min = thr_min_ + 1e-3, unk = thr_min_ - 999.0;
    string p = map_path_ + "/" + to_string(bid);
    mtx_.unlock();
    GB->origin_ = Id2BlockIdx3(bid).cwiseProduct(block_size_);


    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    GB->Reset(-9999.0, voxel_num);
    GB->id_ = bid;
    int data_size = ceil(voxel_num / 5);
    uint8_t data[data_size];
    std::ifstream infile(p.c_str(), ios::binary);
    if(!infile.is_open()){ // not exist, use unknown 
        GB->state_ = UNKNOWN;
        // mtx_.unlock();
        return false;
    }
    infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[data_size]));
    infile.close();
    // ROS_WARN("read success");
    // cout<<"data_size:"<<data_size<<endl;
    // cout<<"voxel_num:"<<voxel_num<<endl;
    int u = 0, o = 0, f = 0;
    GB->state_ = MIXED;
    uint64_t idx;
    uint8_t data_seg[5];
    for(int i = 0, j = 0; i < data_size; i++){
        DecodeVoxel(data[i], data_seg);
        for(int k = 0; k < 5; k++){
            if(j >= voxel_num){
                // do nothing
            }
            else{
                if(data_seg[k] == 0) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.insert({idx, {1, 0}});
                    GB->odds_log_[j] = thr_max;
                    o++;
                }
                else if(data_seg[k] == 1) {
                    idx = LocalId2GlobalId(j, GB);
                    changed_pts_.insert({idx, {0, 0}});
                    GB->odds_log_[j] = thr_min;
                    f++;
                }
                else {
                    GB->odds_log_[j] = unk;
                    u++;
                }
            }
            j++;
        }
    }

    GB->show_ = true;
    mtx_.lock();
    // cout<<"reload add change:"<<bid<<endl;
    changed_blocks_.emplace_back(bid);
    mtx_.unlock();
    return true;
}

bool BlockMapLite::ReadDataRaw(const int &bid, shared_ptr<Grid_Block> &GB){
    GB = make_shared<Grid_Block>();
    if(bid < 0){
        GB = NULL;
        return false;
    }
    mtx_.lock();
    GB->origin_ = Id2BlockIdx3(bid).cwiseProduct(block_size_);
    double thr_max = thr_max_ *0.01, thr_min = thr_min_ * 0.01, unk = thr_min_ - 999.0;
    string p = map_path_ + "/" + to_string(bid);
    mtx_.unlock();

    int voxel_num = block_size_(0) * block_size_(1) * block_size_(2);
    GB->Reset(0, voxel_num);
    int data_size = ceil(voxel_num / 5);
    uint8_t data[voxel_num];
    std::ifstream infile(p.c_str(), ios::binary);
    if(!infile.is_open()){ // not exist, use unknown 
        GB->state_ = UNKNOWN;
        return false;
    }
    infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[voxel_num / 5]));
    infile.close();
    GB->state_ = MIXED;
    uint8_t data_seg[5];
    for(int i = 0, j = 0; i < data_size; i++){
        DecodeVoxel(data[i], data_seg);
        for(int k = 0; k < 5; k++){
            if(j >= voxel_num){
                // do nothing
            }
            else{
                if(data_seg[k] == 0) GB->odds_log_[j] = thr_max;
                else if(data_seg[k] == 1) GB->odds_log_[j] = thr_min;
                else GB->odds_log_[j] = unk;
            }
            j++;
        }
    }
    return true;
}