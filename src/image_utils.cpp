// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include/image_utils.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>


#include "dnn/hb_sys.h"

std::shared_ptr<NV12PyramidInput> ImageUtils::GetNV12Pyramid(
    const cv::Mat &bgr_mat, int scaled_img_height, int scaled_img_width) {
  int original_img_height = bgr_mat.rows;
  int original_img_width = bgr_mat.cols;
  double factor = static_cast<double>(original_img_width) /
                  static_cast<double>(scaled_img_width);
  auto tmp_height = static_cast<double>(original_img_height) / factor;
  cv::Mat mat_tmp;
  mat_tmp.create(tmp_height, scaled_img_width, bgr_mat.type());
  cv::Mat M = cv::Mat::zeros(2, 3, CV_32FC1);
  M.at<float>(0, 0) = 0.5;
  M.at<float>(1, 1) = 0.5;
  cv::warpAffine(bgr_mat, mat_tmp, M, mat_tmp.size());
  // cv::resize(bgr_mat, mat_tmp, mat_tmp.size(), 0, 0);
  // cv::imwrite("resized_img.jpg", mat_tmp);

  cv::Mat img_nv12;
  auto ret = ImageUtils::BGRToNv12(mat_tmp, img_nv12);
  if (ret) {
    std::cout << "get nv12 image failed " << std::endl;
    return nullptr;
  }

  auto *y = new hbSysMem;
  auto *uv = new hbSysMem;

  auto w_stride = ALIGN_16(scaled_img_width);
  hbSysAllocCachedMem(y, scaled_img_height * w_stride);
  hbSysAllocCachedMem(uv, scaled_img_height / 2 * w_stride);

  uint8_t *data = img_nv12.data;
  auto *hb_y_addr = reinterpret_cast<uint8_t *>(y->virAddr);
  auto *hb_uv_addr = reinterpret_cast<uint8_t *>(uv->virAddr);
  int crop_len_y = scaled_img_width * (tmp_height - scaled_img_height) / 2;
  int crop_len_uv = crop_len_y >> 1;
  int src_y_len = scaled_img_width * tmp_height;
  int y_len = scaled_img_width * scaled_img_height;
  int uv_len = y_len >> 1;
  memcpy(hb_y_addr, data + crop_len_y, y_len);
  memcpy(hb_uv_addr, data + src_y_len + crop_len_uv, uv_len);

  hbSysFlushMem(y, HB_SYS_MEM_CACHE_CLEAN);
  hbSysFlushMem(uv, HB_SYS_MEM_CACHE_CLEAN);
  auto pyramid = new NV12PyramidInput;
  pyramid->width = scaled_img_width;
  pyramid->height = scaled_img_height;
  pyramid->y_vir_addr = y->virAddr;
  pyramid->y_phy_addr = y->phyAddr;
  pyramid->y_stride = w_stride;
  pyramid->uv_vir_addr = uv->virAddr;
  pyramid->uv_phy_addr = uv->phyAddr;
  pyramid->uv_stride = w_stride;
  return std::shared_ptr<NV12PyramidInput>(pyramid,
                                           [y, uv](NV12PyramidInput *pyramid) {
                                             // Release memory after deletion
                                             hbSysFreeMem(y);
                                             hbSysFreeMem(uv);
                                             delete y;
                                             delete uv;
                                             delete pyramid;
                                           });
}


int32_t ImageUtils::BGRToNv12(cv::Mat &bgr_mat, cv::Mat &img_nv12) {
  auto height = bgr_mat.rows;
  auto width = bgr_mat.cols;

  if (height % 2 || width % 2) {
    std::cerr << "input img height and width must aligned by 2!";
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  if (yuv_mat.data == nullptr) {
    std::cerr << "yuv_mat.data is null pointer" << std::endl;
    return -1;
  }

  auto *yuv = yuv_mat.ptr<uint8_t>();
  if (yuv == nullptr) {
    std::cerr << "yuv is null pointer" << std::endl;
    return -1;
  }
  img_nv12 = cv::Mat(height * 3 / 2, width, CV_8UC1);
  auto *ynv12 = img_nv12.ptr<uint8_t>();

  int32_t uv_height = height / 2;
  int32_t uv_width = width / 2;

  // copy y data
  int32_t y_size = height * width;
  memcpy(ynv12, yuv, y_size);

  // copy uv data
  uint8_t *nv12 = ynv12 + y_size;
  uint8_t *u_data = yuv + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;

  for (int32_t i = 0; i < uv_width * uv_height; i++) {
    *nv12++ = *u_data++;
    *nv12++ = *v_data++;
  }
  return 0;
}

