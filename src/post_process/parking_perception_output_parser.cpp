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

#include "include/post_process/parking_perception_output_parser.h"

#include <queue>
#include "rclcpp/rclcpp.hpp"


ParkingConfig default_parking_config = {
    {{40, 20, 10, 5}},
    {{80, 40, 20, 10}},
    {{8, 16, 32, 64}},
    5,
    {"cyclist",    "person",      "rear",
     "vehicle",    "parkinglock"},
    ""};

struct ScoreId {
    float score;
    float centerness;
    int cls;
    int valid_index;
  };


std::vector<cv::Mat> locations(4);  // 分别对应stride=8,16,32,64
std::once_flag center_flag;

int32_t ParkingPerceptionOutputParser::Parse(
    std::shared_ptr<ParkingPerceptionResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_descriptions,
    std::shared_ptr<DNNTensor> &output_tensor,
    std::vector<std::shared_ptr<OutputDescription>> &depend_output_descs,
    std::vector<std::shared_ptr<DNNTensor>> &depend_output_tensors,
    std::vector<std::shared_ptr<DNNResult>> &depend_outputs) {
  if (!output_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                 "output_tensor invalid cast");
    return -1;
  }

  if (!input_descriptions.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                 "empty input_descriptions");
  }

  if (output_descriptions) {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                 "type: %s, GetDependencies size: %d",
                 output_descriptions->GetType().c_str(),
                 output_descriptions->GetDependencies().size());
    if (!output_descriptions->GetDependencies().empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                   "Dependencies: %d",
                   output_descriptions->GetDependencies().front());
    }
  }

  if (depend_output_tensors.size() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                 "depend out tensor size invalid cast");
    return -1;
  }

  std::shared_ptr<ParkingPerceptionResult> result = nullptr;
  if (!output) {
    result = std::make_shared<ParkingPerceptionResult>();
    result->Reset();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<ParkingPerceptionResult>(output);
    result->Reset();
  }

  int ret = 0;
  ret = SegmentationPostProcess(depend_output_tensors, result->perception);
  if (ret != 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                "segmentation postprocess return error, code = %d",
                ret);
  }

  ret = DetectionPostProcess(depend_output_tensors, result->perception);
  if (ret != 0) {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                "detection postprocess return error, code = %d",
                ret);
  }

  std::stringstream ss;
  ss << "ParkingPerceptionOutputParser parse finished, predict result: "
     << result->perception;
  RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"), "%s", ss.str().c_str());
  return ret;
}


int ParkingPerceptionOutputParser::SegmentationPostProcess(
    std::vector<std::shared_ptr<DNNTensor>>& tensors, Perception& perception) {
  perception.type = Perception::SEG;
  hbSysFlushMem(&(tensors[output_segmentation_index_]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);

  if (tensors[output_segmentation_index_]->properties.tensorLayout != HB_DNN_LAYOUT_NCHW) {
    return -1;
  }

  int channel = tensors[output_segmentation_index_]->properties.validShape.dimensionSize[1];
  int height = tensors[output_segmentation_index_]->properties.validShape.dimensionSize[2];
  int width = tensors[output_segmentation_index_]->properties.validShape.dimensionSize[3];

  RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"),
               "PostProcess width: %d height: %d channel: %d",
               width,
               height,
               channel);

  perception.seg.data.resize(height * width);
  perception.seg.seg.resize(height * width);
  perception.seg.width = width;
  perception.seg.height = height;
  perception.seg.channel = channel;
  perception.seg.num_classes = num_classes_;

  uint8_t* data = reinterpret_cast<uint8_t*>(tensors[0]->sysMem[0].virAddr);
  for (int index = 0; index < height * width; index++){
    perception.seg.seg[index] = data[index] % 10;
    perception.seg.data[index] = static_cast<float>(data[index] % 10);
  }

  return 0;
}


int ParkingPerceptionOutputParser::DetectionPostProcess(
    std::vector<std::shared_ptr<DNNTensor>> &tensors,
    Perception &perception) {
  if (!tensors[0]) {
    RCLCPP_ERROR(rclcpp::get_logger("ParkingPerceptionOutputParser"), "tensor layout error.");
    return -1;
  }

  for (size_t i = 0; i < tensors.size(); i++) {
    if (!tensors[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("ParkingPerceptionOutputParser"),
                  "tensor layout null, error.");
      return -1;
    }
    hbSysFlushMem(&(tensors[i]->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  }

  if (tensors[0]->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    std::call_once(center_flag, [&, this]() {
      for (size_t i = 0; i < locations.size(); i++) {  
        ComputeLocations(parking_config_.heights[i], 
                        parking_config_.widths[i], 
                        parking_config_.strides[i], locations[i]);
      }
    });

    DecodeClsDataNCHW(tensors, perception.det, model_input_height_,
              model_input_width_, model_input_height_,
              model_input_width_, score_threshold_,
              nms_threshold_, nms_top_k_);

  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("ParkingPerceptionOutputParser"), "tensor layout error.");
  }
  
  return 0;
}


void ParkingPerceptionOutputParser::ComputeLocations(const int &height,
                                                 const int &width,
                                                 const int &feature_stride,
                                                 cv::Mat &location) {
  std::vector<int32_t> shift_x = Arange(0, width);
  std::vector<int32_t> shift_y = Arange(0, height);
  cv::Mat shift_x_mat = cv::Mat(shift_x) * feature_stride;
  cv::Mat shift_y_mat = cv::Mat(shift_y) * feature_stride;
  // meshgrid
  cv::Mat shift_x_mesh, shift_y_mesh;
  MeshGrid(shift_x_mat, shift_y_mat, shift_x_mesh, shift_y_mesh);
  cv::Mat concat_xy;
  cv::vconcat(shift_x_mesh.reshape(1, 1), shift_y_mesh.reshape(1, 1),
              concat_xy);
  cv::transpose(concat_xy, location);
  location = location + feature_stride / 2;
}


//.........................................................//////

int ParkingPerceptionOutputParser::DecodeClsDataNCHW(
    const std::vector<std::shared_ptr<DNNTensor>> &tensors, std::vector<Detection> &dets,
    int basic_pyramid_image_height, int basic_pyramid_image_width,
    int src_image_height, int src_image_width,
    float score_threshold, float nms_threshold, int post_nms_top_k) {
  int cls_num = 5;
  int layer_num = 4;
  std::vector<int> align_c_list(layer_num);
  std::vector<int> align_h_list(layer_num);
  std::vector<int> align_w_list(layer_num);
  std::vector<int> aligned_hw_list(layer_num);
  std::vector<int> shape_c_list(layer_num);
  std::vector<int> shape_h_list(layer_num);
  std::vector<int> shape_w_list(layer_num);
  std::vector<float *> cls_data_list(layer_num);
  std::vector<float *> bbox_data_list(layer_num);
  std::vector<float *> ce_data_list(layer_num);

  double h_ratio = basic_pyramid_image_height * 1.0 / src_image_height;
  double w_ratio = basic_pyramid_image_width * 1.0 / src_image_width;
  float score_tmp_thresh = log(score_threshold / (1.0 - score_threshold));

  for (int i = 0; i < layer_num; i++) {
    auto tensor = tensors[i + 1];
    // NCHW
    if (tensor->properties.tensorLayout != HB_DNN_LAYOUT_NCHW) {
      return -1;
    }
    shape_c_list[i] = tensor->properties.validShape.dimensionSize[1];
    shape_h_list[i] = tensor->properties.validShape.dimensionSize[2];
    shape_w_list[i] = tensor->properties.validShape.dimensionSize[3];
    align_c_list[i] = tensor->properties.alignedShape.dimensionSize[1];
    align_h_list[i] = tensor->properties.alignedShape.dimensionSize[2];
    align_w_list[i] = tensor->properties.alignedShape.dimensionSize[3];
    aligned_hw_list[i] = align_h_list[i] * align_w_list[i];

    cls_data_list[i] =
        reinterpret_cast<float *>(tensors[i + 1].get()->sysMem[0].virAddr);
    bbox_data_list[i] = reinterpret_cast<float *>(
        tensors[i + 1 + layer_num].get()->sysMem[0].virAddr);
    ce_data_list[i] = reinterpret_cast<float *>(
        tensors[i + 1 + (layer_num << 1)].get()->sysMem[0].virAddr);
  }

  std::vector<std::vector<std::shared_ptr<Detection>>> box_data(cls_num);

  std::vector<std::vector<std::vector<ScoreId>>> score_id_datas(
      cls_num, std::vector<std::vector<ScoreId>>(layer_num));
  std::vector<std::vector<std::vector<float>>> sc_float_datas(
      cls_num, std::vector<std::vector<float>>(layer_num));
  std::vector<std::vector<std::vector<float>>> ce_float_datas(
      cls_num, std::vector<std::vector<float>>(layer_num));

  for (int i = 0; i < layer_num; i++) {
    auto aligned_hw = aligned_hw_list[i];
    auto shape_c = shape_c_list[i];
    auto shape_h = shape_h_list[i];
    auto shape_w = shape_w_list[i];
    auto cls_data = cls_data_list[i];
    auto ce_data = ce_data_list[i];

    for (int h = 0; h < shape_h; h++) {
      uint32_t offset_h = h * align_w_list[i];
      for (int w = 0; w < shape_w; w++) {
        int max_score_cls = 0;
        int offset_w = offset_h + w;
        int max_score_index = offset_w;
        float score_data = cls_data[offset_w];
        float max_score_data = score_data;
        int sec_score_cls = 0;
        int sec_score_index = 0;
        float sec_score_data = 0;

        for (int cls_c = 1; cls_c < shape_c; cls_c++) {
          int cls_index = cls_c * aligned_hw + offset_w;
          float score_data = cls_data[cls_index];
          if (score_data > max_score_data) {
            sec_score_data = max_score_data;
            sec_score_cls = max_score_cls;
            sec_score_index = max_score_index;
            max_score_data = score_data;
            max_score_cls = cls_c;
            max_score_index = cls_index;
          } else if (score_data > sec_score_data) {
            sec_score_data = score_data;
            sec_score_cls = cls_c;
            sec_score_index = cls_index;
          }
        }
        if (max_score_data <= score_tmp_thresh) continue;
        ScoreId max_score_id = {cls_data[max_score_index],
                                ce_data[max_score_index], max_score_cls,
                                max_score_index};
        score_id_datas[max_score_cls][i].push_back(max_score_id);
        sc_float_datas[max_score_cls][i].push_back(max_score_id.score);
        ce_float_datas[max_score_cls][i].push_back(max_score_id.centerness);

        if (sec_score_data > score_tmp_thresh) {
          ScoreId sec_score_id = {cls_data[sec_score_index],
                                  ce_data[sec_score_index], sec_score_cls,
                                  sec_score_index};
          score_id_datas[sec_score_cls][i].push_back(sec_score_id);
          sc_float_datas[sec_score_cls][i].push_back(sec_score_id.score);
          ce_float_datas[sec_score_cls][i].push_back(sec_score_id.centerness);
        }
      }  // for score_w
    }    // for score_h
  }      // for layer_num

  for (int cls = 0; cls < cls_num; cls++) {
    for (int layer = 0; layer < layer_num; layer++) {
      auto score_id_data = score_id_datas[cls][layer];
      auto sc_float_data = sc_float_datas[cls][layer];
      auto ce_float_data = ce_float_datas[cls][layer];
      auto valid_num = score_id_data.size();
      if (valid_num == 0) continue;

      cv::Mat box_score_data_raw(valid_num, 1, CV_32FC1, sc_float_data.data());
      cv::Mat box_ctr_data_raw(valid_num, 1, CV_32FC1, ce_float_data.data());

      cv::Mat box_score_data, box_ctr_data;
      cv::exp(box_score_data_raw * (-1), box_score_data);
      box_score_data = 1 / (1 + box_score_data);  // SigMoid
      cv::exp(box_ctr_data_raw * (-1), box_ctr_data);
      box_ctr_data = 1 / (1 + box_ctr_data);  // SigMoid
      cv::multiply(box_score_data, box_ctr_data, box_score_data);
      auto rows = box_score_data.rows;
      auto cols = box_score_data.cols;

      uint32_t step = 0;
      for (int row = 0; row < rows; row++) {
        float *ptr = box_score_data.ptr<float>(row);
        for (int col = 0; col < cols; col++) {

          if (ptr[col] > score_threshold) {

            // Detection detection;
            auto detection = std::make_shared<Detection>();

            // get detection score
            detection->score = ptr[col];  // todo

            // get detection box
            auto aligned_hw = aligned_hw_list[layer];
            auto bbox_data = bbox_data_list[layer];

            auto one_score = score_id_data[step];
            auto valid_index = one_score.valid_index;

            auto c_index = valid_index / aligned_hw;
            auto c_offset = valid_index % aligned_hw;
            auto location_0 = locations[layer].ptr<int32_t>(c_offset)[0];
            auto location_1 = locations[layer].ptr<int32_t>(c_offset)[1];
            auto c_box_index = c_index * (aligned_hw << 2);
            auto x1_index = c_box_index + 0 * aligned_hw + c_offset;
            auto y1_index = c_box_index + 1 * aligned_hw + c_offset;
            auto x2_index = c_box_index + 2 * aligned_hw + c_offset;
            auto y2_index = c_box_index + 3 * aligned_hw + c_offset;
            auto x1 = std::max(
                std::min(location_0 - bbox_data[x1_index],
                         static_cast<float>(basic_pyramid_image_width) - 1),
                0.0f);
            auto y1 = std::max(
                std::min(location_1 - bbox_data[y1_index],
                         static_cast<float>(basic_pyramid_image_height) - 1),
                0.0f);
            auto x2 = std::max(
                std::min(location_0 + bbox_data[x2_index],
                         static_cast<float>(basic_pyramid_image_width) - 1),
                0.0f);
            auto y2 = std::max(
                std::min(location_1 + bbox_data[y2_index],
                         static_cast<float>(basic_pyramid_image_height) - 1),
                0.0f);
            float tmp = 0.0;
            tmp = x1 / w_ratio;  // x1
            detection->bbox.xmin =
                tmp < 0.0 ? 0.0 : tmp > src_image_width ? src_image_width : tmp;
            tmp = y1 / h_ratio;  // y1
            detection->bbox.ymin = tmp < 0.0 ? 0.0 : tmp > src_image_height
                                                  ? src_image_height
                                                  : tmp;
            tmp = x2 / w_ratio;  // x2
            detection->bbox.xmax =
                tmp < 0.0 ? 0.0 : tmp > src_image_width ? src_image_width : tmp;
            tmp = y2 / h_ratio;  // y2
            detection->bbox.ymax = tmp < 0.0 ? 0.0 : tmp > src_image_height
                                                  ? src_image_height
                                                  : tmp;
            box_data[cls].emplace_back(detection);
          }
          step++;
        }
      }
    }
  }

  for (int cls = 0; cls < cls_num; cls++) {
    std::vector<std::shared_ptr<Detection>> cls_box_result;

    auto cls_box_data = box_data[cls];

    NMS(cls_box_data, nms_threshold, post_nms_top_k, cls_box_result);

    // Detection BaseData

    for (auto &box : cls_box_result) {

      Detection detection_result;

      detection_result.bbox.xmin = box->bbox.xmin;
      detection_result.bbox.ymin = box->bbox.ymin;
      detection_result.bbox.xmax = box->bbox.xmax;
      detection_result.bbox.ymax = box->bbox.ymax;

      detection_result.score = box->score;
      detection_result.id = cls;
      detection_result.class_name = parking_config_.class_names[cls].c_str();

      dets.push_back(detection_result);

    }
  }
  return 0;
}


void ParkingPerceptionOutputParser::NMS(
    std::vector<std::shared_ptr<Detection>> &candidates, const float &overlap_ratio,
    const int &top_k, std::vector<std::shared_ptr<Detection>> &result) {

  if (candidates.size() == 0) {
    return;
  }
  std::vector<bool> skip(candidates.size(), false);

  auto greater = [](const std::shared_ptr<Detection> &a,
                    const std::shared_ptr<Detection> &b) {
    return a->score > b->score;
  };


  std::stable_sort(candidates.begin(), candidates.end(), greater);

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); ++i) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    float area_i = (candidates[i]->bbox.xmax - candidates[i]->bbox.xmin) 
                  * (candidates[i]->bbox.ymax - candidates[i]->bbox.ymin);


    // suppress the significantly covered bbox
    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      // get intersections
      float xx1 = std::max(candidates[i]->bbox.xmin, candidates[j]->bbox.xmin);
      float yy1 = std::max(candidates[i]->bbox.ymin, candidates[j]->bbox.ymin);
      float xx2 = std::min(candidates[i]->bbox.xmax, candidates[j]->bbox.xmax);
      float yy2 = std::min(candidates[i]->bbox.ymax, candidates[j]->bbox.ymax);
      float area_intersection = (xx2 - xx1) * (yy2 - yy1);
      bool area_intersection_valid = (area_intersection > 0) && (xx2 - xx1 > 0);

      if (area_intersection_valid) {
        // compute overlap
        float area_j = (candidates[j]->bbox.xmax - candidates[j]->bbox.xmin) 
                  * (candidates[j]->bbox.ymax - candidates[j]->bbox.ymin);
        float o = area_intersection / (area_i + area_j - area_intersection);

        if (o > overlap_ratio) {
          skip[j] = true;
        }
      }
    }
    // store candidates box to xstream::BBox
    {
      result.emplace_back(candidates[i]);
    }
    candidates.clear();
  }

}