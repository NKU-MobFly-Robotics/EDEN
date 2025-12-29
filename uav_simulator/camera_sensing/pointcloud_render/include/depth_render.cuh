#ifndef DEPTH_RENDER_CUH
#define DEPTH_RENDER_CUH

#include <cstdlib>
#include <ctime>
#include <cuda_runtime.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#include "cuda_exception.cuh"
#include "device_image.cuh"
#include "helper_math.h"

using namespace std;

struct Parameter {
  int point_number;
  float fx, fy, cx, cy;
  int width, height;
  float r[3][3];
  float t[3];
  float radius;
};

class DepthRender {
public:
  DepthRender();
  ~DepthRender();

  void set_para(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float radius);
  void set_data(vector<float> &cloud_data);

  void render_pose(double *transformation, int *host_ptr);

private:
  int cloud_size;

  // data
  float3 *host_cloud_ptr;
  float3 *dev_cloud_ptr;
  bool has_devptr;

  // camera
  Parameter parameter;
  Parameter *parameter_devptr;
};

#endif