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

#ifndef INCLUDE_PARKING_PERCEPTION_OUTPUT_PARSER_H_
#define INCLUDE_PARKING_PERCEPTION_OUTPUT_PARSER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "dnn/hb_dnn_ext.h"
#include "perception_common.h"

#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include "dnn_node/dnn_node.h"

using hobot::dnn_node::DNNTensor;
using hobot::dnn_node::Model;

struct ParkingConfig {
  std::vector<int> heights;
  std::vector<int> widths;
  std::vector<int> strides;
  int class_num;
  std::vector<std::string> class_names;
  std::string det_name_list;
};
extern ParkingConfig default_parking_config;


class ParkingPerceptionResult {
 public:
  Perception perception;
  void Reset() {
    perception.seg.seg.clear();
    perception.seg.data.clear();
    perception.det.clear();
  }
};

class ParkingPerceptionOutputParser {
 public:
  int32_t Parse(
      std::shared_ptr<ParkingPerceptionResult> &output,
      std::vector<std::shared_ptr<DNNTensor>> &output_tensors);

 private:

  int SegmentationPostProcess(
      std::vector<std::shared_ptr<DNNTensor>>& output_tensors,
      Perception& perception);

  int DetectionPostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &output_tensors,
    Perception &perception);
    
  void ComputeLocations(const int &height,
                        const int &width,
                        const int &feature_stride,
                        cv::Mat &location);

  // numpy arange
  template <typename T>
  inline std::vector<T> Arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step) values.push_back(value);
    return values;
  }

  // numpy meshgrid
  inline void MeshGrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat &X,
                      cv::Mat &Y) {
    cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
    cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
  }

  void NMS(
    std::vector<std::shared_ptr<Detection>> &candidates, const float &overlap_ratio,
    const int &top_k, std::vector<std::shared_ptr<Detection>> &result);

  int DecodeClsDataNCHW(
    const std::vector<std::shared_ptr<DNNTensor>> &tensors, std::vector<Detection> &dets,
    int basic_pyramid_image_height, int basic_pyramid_image_width,
    int src_image_height, int src_image_width,
    float score_threshold, float nms_threshold, int post_nms_top_k);

 private:
  int num_classes_ = 8;
  float score_threshold_ = 0.3;
  float nms_threshold_ = 0.6;
  int nms_top_k_ = 1000;

  const size_t model_output_count_ = 13;
  int model_input_width_ = 640;
  int model_input_height_ = 320;

  const size_t output_segmentation_index_ = 0;
  const size_t output_class_index_ = 1;
  const size_t output_bbox_index_ = 5;
  const size_t output_centerness_index_ = 9;

  ParkingConfig parking_config_ = default_parking_config;
};


#endif  // INCLUDE_PARKING_PERCEPTION_OUTPUT_PARSER_H_
