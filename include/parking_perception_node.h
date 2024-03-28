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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "include/post_process/parking_perception_output_parser.h"
#include "sensor_msgs/msg/image.hpp"
#ifdef SHARED_MEM_ENABLED
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#endif

#ifndef INCLUDE_PARKING_PERCEPTION_NODE_H_
#define INCLUDE_PARKING_PERCEPTION_NODE_H_

using rclcpp::NodeOptions;

using hobot::dnn_node::DnnNode;
using hobot::dnn_node::DnnNodeOutput;
using hobot::dnn_node::DnnNodePara;
using hobot::dnn_node::ModelTaskType;
using hobot::dnn_node::TaskId;

using hobot::dnn_node::DNNInput;
using hobot::dnn_node::NV12PyramidInput;

struct ParkingPerceptionOutput : public DnnNodeOutput {
  std::shared_ptr<std_msgs::msg::Header> image_msg_header_ = nullptr;
  uint32_t src_img_width_;
  uint32_t src_img_height_;
  std::string image_name_;

  std_msgs::msg::Header::_stamp_type stamp;
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;

  struct timespec preprocess_timespec_start;
  struct timespec preprocess_timespec_end;
};

class ParkingPerceptionNode : public DnnNode {
 public:
  ParkingPerceptionNode(const std::string &node_name,
                    const NodeOptions &options = NodeOptions());
  ~ParkingPerceptionNode() override;

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<DnnNodeOutput> &outputs) override;

 private:
  void GetConfig();
  int Start();
  void RosImgProcess(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  void RenderParkingPerception(
    const std::shared_ptr<DnnNodeOutput> &node_output,
    const ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg,
    Parsing &seg,
    std::vector<Detection> &dets);

  void SourceInputPadding(Parsing &seg);

#ifdef SHARED_MEM_ENABLED
  void SharedMemImgProcess(
      const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);
#endif

 private:
  // 输入参数
  int shared_mem_ = 1;
  int dump_render_img_ = 0;

  std::string model_file_name_ = "config/parking_perception_640x320.bin";
  std::string model_name_ = "multibranch";

  int source_input_width_ = 640;
  int source_input_height_ = 320;
  int model_input_width_ = 640;
  int model_input_height_ = 320;
  const int32_t model_output_count_ = 13;
  const int32_t output_index_ = 12;
  ModelTaskType model_task_type_ = ModelTaskType::ModelInferType;
  
  std::chrono::high_resolution_clock::time_point output_tp_;
  std::atomic_int output_frameCount_;
  std::mutex frame_stat_mtx_;
  std::string msg_pub_topic_name_ = "ai_msg_parking_perception";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_;

#ifdef SHARED_MEM_ENABLED
  rclcpp::Subscription<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr
      sharedmem_img_subscription_ = nullptr;
  std::string sharedmem_img_topic_name_ = "/hbmem_img";
#endif
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ros_img_subscription_ = nullptr;
  // 目前只支持订阅原图，可以使用压缩图"/image_raw/compressed" topic
  // 和sensor_msgs::msg::CompressedImage格式扩展订阅压缩图
  std::string ros_img_topic_name_ = "/image_raw";

};

#endif  // INCLUDE_PARKING_PERCEPTION_NODE_H_
