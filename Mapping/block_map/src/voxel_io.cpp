#include <block_map/block_map.h>

bool BlockMap::SaveData(const int &bid, shared_ptr<Grid_Block> &GB){
    if(GB->state_ == UNKNOWN) return false; // dont save unknown GB

    int voxel_num = GB->block_size_(0) * GB->block_size_(1) * GB->block_size_(2);
    int data_size = ceil(voxel_num / 5);
    uint8_t data[voxel_num];
    uint8_t data_seg[5]; // for x and y, x <= 8. 3^(y=5) / 2^(x=8) is the maximum value, to minimize the data waste
    string p = voxel_path_ + "/" + to_string(bid);
    std::ofstream outfile(p.c_str(), ios::binary);
    if(!outfile.is_open()){
        ROS_ERROR("open file failed");
        ros::shutdown();
        return false;
    }
    
    for(int i = 0, j = 0; i < data_size; i++){
        for(int k = 0; k < 5; k++){
            if(j >= voxel_num){
                data_seg[k] = 0;
            }
            else{
                if(GB->odds_log_[j] > 0) data_seg[k] = 0;
                else if(GB->odds_log_[j]  > thr_min_) data_seg[k] = 1;
                else data_seg[k] = 2;
            }
            j++;
        }
        data[i] = EncodeVoxel(data_seg);
    }
    int i = 3;
    outfile.write(reinterpret_cast<char*>(&data), sizeof(uint8_t[voxel_num]));
    outfile.close();
    return true;
}

bool BlockMap::ReadData(const int &bid, shared_ptr<Grid_Block> &GB){
    GB = make_shared<Grid_Block>();
    mtx_.lock();
    GB->block_size_ = block_size_;
    mtx_.unlock();

    int voxel_num = GB->block_size_(0) * GB->block_size_(1) * GB->block_size_(2);
    GB->Reset(0);
    int data_size = ceil(voxel_num / 5);
    uint8_t data[voxel_num];
    string p = voxel_path_ + "/" + to_string(bid);
    std::ifstream infile(p.c_str(), ios::binary);
    if(!infile.is_open()){ // not exist, use unknown 
        GB->state_ = UNKNOWN;
        return false;
    }
    infile.read(reinterpret_cast<char*>(&data), sizeof(uint8_t[voxel_num]));

    infile.close();
    uint8_t data_seg[5];
    for(int i = 0, j = 0; i < data_size; i++){
        DecodeVoxel(data[i], data_seg);
        for(int k = 0; k < 5; k++){
            if(j >= voxel_num){
                // do nothing
            }
            else{
                if(data_seg[k] == 0) GB->odds_log_[j] = thr_max_ * 0.01;
                else if(data_seg[k] == 1) GB->odds_log_[j] = thr_min_ * 0.01;
                else GB->odds_log_[j] = thr_min_ - 999.0;
            }
            j++;
        }
    }
    return true;
}