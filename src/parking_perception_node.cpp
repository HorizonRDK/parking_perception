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

#include "include/parking_perception_node.h"

#include <unistd.h>

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "dnn_node/dnn_node.h"
#include "include/image_utils.h"
#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#ifdef CV_BRIDGE_PKG_ENABLED
#include <cv_bridge/cv_bridge.h>
#endif

using hobot::easy_dnn::OutputDescription;
using hobot::easy_dnn::OutputParser;

builtin_interfaces::msg::Time ConvertToRosTime(
    const struct timespec& time_spec) {
  builtin_interfaces::msg::Time stamp;
  stamp.set__sec(time_spec.tv_sec);
  stamp.set__nanosec(time_spec.tv_nsec);
  return stamp;
}

int CalTimeMsDuration(const builtin_interfaces::msg::Time& start,
                      const builtin_interfaces::msg::Time& end) {
  return (end.sec - start.sec) * 1000 + end.nanosec / 1000 / 1000 -
         start.nanosec / 1000 / 1000;
}

ParkingPerceptionNode::ParkingPerceptionNode(const std::string &node_name,
                                     const NodeOptions &options)
    : DnnNode(node_name, options), output_frameCount_(0) {
  this->declare_parameter<int>("is_sync_mode", is_sync_mode_);
  this->declare_parameter<int>("shared_mem", shared_mem_);
  this->declare_parameter<std::string>("ai_msg_pub_topic_name",
                                       msg_pub_topic_name_);
  this->declare_parameter<std::string>("image_sub_topic_name",
                                       ros_img_topic_name_);
  this->declare_parameter<std::string>("feed_image", feed_image_);
  this->declare_parameter<int>("dump_render_img", dump_render_img_);

  this->get_parameter<int>("is_sync_mode", is_sync_mode_);
  this->get_parameter<int>("shared_mem", shared_mem_);
  this->get_parameter<std::string>("ai_msg_pub_topic_name",
                                   msg_pub_topic_name_);
  this->get_parameter<std::string>("image_sub_topic_name", ros_img_topic_name_);
  this->get_parameter<std::string>("feed_image", feed_image_);
  this->get_parameter<int>("dump_render_img", dump_render_img_);

  mkdir("./result/", 666);

  std::stringstream ss;
  ss << "Parameter:"
     << "\nshared_men:" << shared_mem_ << "\n is_sync_mode_: " << is_sync_mode_
     << "\n model_file_name_: " << model_file_name_
     << "\nfeed_image:" << feed_image_;
  RCLCPP_WARN(rclcpp::get_logger("parking_perception"), "%s",
              ss.str().c_str());
  if (Start() == 0) {
    RCLCPP_WARN(rclcpp::get_logger("parking_perception"),
                "start success!!!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "start fail!!!");
  }
}

ParkingPerceptionNode::~ParkingPerceptionNode() {}

int ParkingPerceptionNode::Start() {
  int ret = Init();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"), "Init failed!");
    return ret;
  }

  ret = GetModelInputSize(0, model_input_width_, model_input_height_);
  if (ret < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Get model input size fail!");
    return ret;
  }
  RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
              "The model input width is %d and height is %d",
              model_input_width_, model_input_height_);

  rclcpp::QoS qos(rclcpp::KeepLast(7));
  qos.reliable();
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
      msg_pub_topic_name_, qos);
      
  RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
              "msg_pub_topic_name: %s", msg_pub_topic_name_.data());
  

  if (!feed_image_.empty()) {
    std::cout << "parking perception read image:" << feed_image_
              << " to detect" << std::endl;
    PredictByImage(feed_image_);
    return 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
              "Detect images that use subscriptions");

  if (shared_mem_) {
#ifdef SHARED_MEM_ENABLED
    RCLCPP_WARN(rclcpp::get_logger("parking_perception"),
                "Create hbmem_subscription with topic_name: %s",
                sharedmem_img_topic_name_.c_str());
    sharedmem_img_subscription_ =
        this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
            sharedmem_img_topic_name_, 10,
            std::bind(&ParkingPerceptionNode::SharedMemImgProcess, this,
                      std::placeholders::_1));
#else
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Unsupport shared mem");
#endif
  } else {
    RCLCPP_WARN(rclcpp::get_logger("parking_perception"),
                "Create subscription with topic_name: %s",
                ros_img_topic_name_.c_str());
    ros_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        ros_img_topic_name_, 10,
        std::bind(&ParkingPerceptionNode::RosImgProcess, this,
                  std::placeholders::_1));
  }

  return 0;
}

int ParkingPerceptionNode::SetNodePara() {
  RCLCPP_INFO(rclcpp::get_logger("parking_perception"), "Set node para.");
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  dnn_node_para_ptr_->model_file = model_file_name_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 2;
  return 0;
}

int ParkingPerceptionNode::SetOutputParser() {
  // set output parser
  auto model_manage = GetModel();
  if (!model_manage || !dnn_node_para_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Invalid model");
    return -1;
  }

  if (model_manage->GetOutputCount() < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Error! Model %s output count is %d, unmatch with count %d",
                 dnn_node_para_ptr_->model_name.c_str(),
                 model_manage->GetOutputCount(), model_output_count_);
    return -1;
  }

  // 1. single branch helper
  for (int i = 0; i < output_index_; ++i) {
    std::shared_ptr<OutputParser> assist_parser =
        std::make_shared<ParkingPerceptionAssistParser>();
    model_manage->SetOutputParser(i, assist_parser);
  }

  // 2. set multi branch paser
  auto seg_output_desc = std::make_shared<OutputDescription>(
      model_manage, output_index_, "seg_branch");
  for (int i = 0; i < output_index_; ++i) {
    seg_output_desc->GetDependencies().push_back(i);
  }
  seg_output_desc->SetType("seg");
  model_manage->SetOutputDescription(seg_output_desc);
  auto SegParser = std::make_shared<ParkingPerceptionOutputParser>();
  std::shared_ptr<OutputParser> seg_out_parser =
      std::dynamic_pointer_cast<OutputParser>(SegParser);
  model_manage->SetOutputParser(output_index_, seg_out_parser);

  return 0;
}


int ParkingPerceptionNode::PostProcess(
    const std::shared_ptr<DnnNodeOutput> &node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }
  
  if (!msg_publisher_) {
    RCLCPP_ERROR(rclcpp::get_logger("example"), "Invalid msg_publisher_");
    return -1;
  }

  if (node_output->rt_stat->fps_updated) {
    RCLCPP_WARN(rclcpp::get_logger("parking_perception"),
                "input fps: %.2f, out fps: %.2f",
                node_output->rt_stat->input_fps,
                node_output->rt_stat->output_fps);
  }

  auto parking_perception_output = std::dynamic_pointer_cast<ParkingPerceptionOutput>(node_output);
  if (parking_perception_output && debug_info_) {
    std::stringstream ss;
    ss << "Output from";
    if (parking_perception_output->image_msg_header_) {
      ss << ", frame_id: " << parking_perception_output->image_msg_header_->frame_id
         << ", stamp: " << parking_perception_output->image_msg_header_->stamp.sec << "."
         << parking_perception_output->image_msg_header_->stamp.nanosec;
    }
    RCLCPP_INFO(rclcpp::get_logger("parking_perception"), "%s",
                ss.str().c_str());
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  const auto &outputs = node_output->outputs;

  if (outputs.empty() ||
      static_cast<int32_t>(outputs.size()) < model_output_count_) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"), "Invalid outputs");
    return -1;
  }
  

  // get result
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(
      new ai_msgs::msg::PerceptionTargets());
  if (parking_perception_output->image_msg_header_) {
    pub_data->header.set__stamp(parking_perception_output->image_msg_header_->stamp);
    pub_data->header.set__frame_id(parking_perception_output->image_msg_header_->frame_id);
  }
  ai_msgs::msg::Capture capture;

  auto *det_result =
      dynamic_cast<ParkingPerceptionResult *>(outputs[output_index_].get());
  if (!det_result) {
    RCLCPP_DEBUG(rclcpp::get_logger("PostProcess"), "invalid cast");
    return 0;
  }

  // render with each task
  auto &seg = det_result->perception.seg;
  auto &dets = det_result->perception.det;
  if (dump_render_img_){
    RenderParkingPerception(node_output, pub_data, seg, dets);
  }

  // update segmentation ai_msgs
  capture.features.swap(seg.data);
  capture.img.height = seg.height;
  capture.img.width = seg.width;
  capture.img.step = model_input_width_ / seg.width;
  ai_msgs::msg::Target target;
  target.set__type("parkingspace");
  target.captures.emplace_back(std::move(capture));
  pub_data->targets.emplace_back(std::move(target));

  if (debug_info_){
    RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
              "features size: %d, width: %d, height: %d, channel: %d",
              capture.features.size(),
              capture.img.width,
              capture.img.height,
              capture.img.step);
  }

  // update rois ai_msgs
  for (size_t idx = 0; idx < dets.size(); idx++){

    auto &det = dets[idx];
    
    ai_msgs::msg::Target target;
    target.set__type(det.class_name);
    target.set__track_id(idx);
    ai_msgs::msg::Roi roi;
    roi.type = det.class_name;
    roi.rect.set__x_offset(det.bbox.xmin);
    roi.rect.set__y_offset(det.bbox.ymin);
    roi.rect.set__width(det.bbox.xmax - det.bbox.xmin);
    roi.rect.set__height(det.bbox.ymax - det.bbox.ymin);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }


  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);

  // preprocess
  ai_msgs::msg::Perf perf_preprocess;
  perf_preprocess.set__type("preprocess");
  perf_preprocess.set__stamp_start(
      ConvertToRosTime(parking_perception_output->preprocess_timespec_start));
  perf_preprocess.set__stamp_end(
      ConvertToRosTime(parking_perception_output->preprocess_timespec_end));
  perf_preprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_preprocess.stamp_start, perf_preprocess.stamp_end));
  pub_data->perfs.emplace_back(perf_preprocess);

  // predict
  if (node_output->rt_stat) {
    ai_msgs::msg::Perf perf;
    perf.set__type("predict_infer");
    perf.set__stamp_start(
        ConvertToRosTime(node_output->rt_stat->infer_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(node_output->rt_stat->infer_timespec_end));
    perf.set__time_ms_duration(node_output->rt_stat->infer_time_ms);
    pub_data->perfs.push_back(perf);

    perf.set__type(model_name_ + "_predict_parse");
    perf.set__stamp_start(
        ConvertToRosTime(node_output->rt_stat->parse_timespec_start));
    perf.set__stamp_end(
        ConvertToRosTime(node_output->rt_stat->parse_timespec_end));
    perf.set__time_ms_duration(node_output->rt_stat->parse_time_ms);
    pub_data->perfs.push_back(perf);
  }

  // postprocess
  ai_msgs::msg::Perf perf_postprocess;
  perf_postprocess.set__type("postprocess");
  perf_postprocess.set__stamp_start(ConvertToRosTime(time_start));
  clock_gettime(CLOCK_REALTIME, &time_now);
  perf_postprocess.set__stamp_end(ConvertToRosTime(time_now));
  perf_postprocess.set__time_ms_duration(CalTimeMsDuration(
      perf_postprocess.stamp_start, perf_postprocess.stamp_end));
  pub_data->perfs.emplace_back(perf_postprocess);

  // 从发布图像到发布AI结果的延迟
  ai_msgs::msg::Perf perf_pipeline;
  perf_pipeline.set__type("pipeline");
  perf_pipeline.set__stamp_start(pub_data->header.stamp);
  perf_pipeline.set__stamp_end(perf_postprocess.stamp_end);
  perf_pipeline.set__time_ms_duration(
      CalTimeMsDuration(perf_pipeline.stamp_start, perf_pipeline.stamp_end));
  pub_data->perfs.push_back(perf_pipeline);

  if (debug_info_){
    std::stringstream ss;
    ss << "Publish frame_id: "
        << parking_perception_output->image_msg_header_->frame_id
        << ", time_stamp: " << std::to_string(pub_data->header.stamp.sec)
        << "_" << std::to_string(pub_data->header.stamp.nanosec) << "\n";
    RCLCPP_INFO(
        rclcpp::get_logger("parking_perception"), "%s", ss.str().c_str());
  }

  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int ParkingPerceptionNode::Predict(
    std::vector<std::shared_ptr<DNNInput>> &inputs,
    const std::shared_ptr<std::vector<hbDNNRoi>> rois,
    std::shared_ptr<DnnNodeOutput> dnn_output) {
  return Run(inputs, dnn_output, rois, is_sync_mode_ == 1);
}

void ParkingPerceptionNode::RosImgProcess(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  std::stringstream ss;
  ss << "Recved img encoding: " << img_msg->encoding
     << ", h: " << img_msg->height << ", w: " << img_msg->width
     << ", step: " << img_msg->step
     << ", frame_id: " << img_msg->header.frame_id
     << ", stamp: " << img_msg->header.stamp.sec << "."
     << img_msg->header.stamp.nanosec
     << ", data size: " << img_msg->data.size();
  RCLCPP_INFO(rclcpp::get_logger("parking_perception"), "%s",
              ss.str().c_str());

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("rgb8" == img_msg->encoding) {
#ifdef CV_BRIDGE_PKG_ENABLED
    auto cv_img =
        cv_bridge::cvtColorForDisplay(cv_bridge::toCvShare(img_msg), "bgr8");
    {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
                   "after cvtColorForDisplay cost ms: %d", interval);
    }

    pyramid = ImageUtils::GetNV12Pyramid(cv_img->image, model_input_height_,
                                         model_input_width_);
#else
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Unsupport cv bridge");
#endif
  } else if ("nv12" == img_msg->encoding) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Get Nv12 pym fail");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
              "Dnn node begin to predict");
  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<ParkingPerceptionOutput>();
  dnn_output->src_img_width_ = img_msg->width;
  dnn_output->src_img_height_ = img_msg->height;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id(img_msg->header.frame_id);
  dnn_output->image_msg_header_->set__stamp(img_msg->header.stamp);
  dnn_output->image_name_ = "";

  // 3. 开始预测
  uint32_t ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
                 "after Predict cost ms: %d", interval);
  }
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "predict img failed!");
  }
  return;
}

int ParkingPerceptionNode::PredictByImage(
        const std::string &image) {
  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  // bgr img，支持将图片resize到模型输入size
  pyramid = ImageUtils::GetNV12Pyramid(
          image, ImageType::BGR,
          model_input_height_, model_input_width_);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Get Nv12 pym fail with image: %s", image.c_str());
    return -1;
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<ParkingPerceptionOutput>();
  dnn_output->src_img_width_ = 1920;
  dnn_output->src_img_height_ = 1080;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id("test_frame");
  dnn_output->image_msg_header_->set__stamp(rclcpp::Time());
  dnn_output->image_name_ = image;

  dnn_output->pyramid = pyramid;
  
  // 3. 开始预测
  uint32_t ret = Predict(inputs, nullptr, dnn_output);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "predict img failed!");
  }
  RCLCPP_INFO(rclcpp::get_logger("parking_perception"), "Finsh PredictByImage");
  return ret;
}

void ParkingPerceptionNode::RenderParkingPerception(
    const std::shared_ptr<DnnNodeOutput> &node_output,
    const ai_msgs::msg::PerceptionTargets::UniquePtr &ai_msg,
    Parsing &seg,
    std::vector<Detection> &dets){

  // 1. get pyramid data
  auto parking_perception_output =
      std::dynamic_pointer_cast<ParkingPerceptionOutput>(node_output);
  auto pyramid = parking_perception_output->pyramid;
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"), "Invalid pyramid!");
    // return -1;
    return;
  }

  int parsing_width = seg.width;
  int parsing_height = seg.height;

  char *y_img = reinterpret_cast<char *>(pyramid->y_vir_addr);
  char *uv_img = reinterpret_cast<char *>(pyramid->uv_vir_addr);
  auto height = pyramid->height;
  auto width = pyramid->width;
  
  auto img_y_size = height * width;
  auto img_uv_size = img_y_size / 2;
  char *buf = new char[img_y_size + img_uv_size];
  memcpy(buf, y_img, img_y_size);
  memcpy(buf + img_y_size, uv_img, img_uv_size);
  cv::Mat nv12(height * 3 / 2, width, CV_8UC1, buf);
  cv::Mat mat;                               //  get bgr mat from pyramid
  cv::cvtColor(nv12, mat, CV_YUV2BGR_NV12);  //  nv12 to bgr
  delete[] buf;

  if(height != model_input_height_ && width != model_input_width_){
    cv::Mat mat_tmp;
    mat_tmp.create(model_input_height_, model_input_width_, mat.type());
    cv::resize(mat, mat_tmp, mat_tmp.size(), 0, 0);
    swap(mat, mat_tmp);
  }
  
  // 2. render segmentation result

  /*
  * Segmentation: tensor[0] NCHW: [1x1x160x320],  data is class
   * class as follows:
   *    0: [road]          1: [background]
   *    2: [lane_marking]  3: [sign_line]
   *    4: [parking_lane]  5: [parking_space]
   *    6: [parking_rod]   7: [parking_lock]
  */
 
  static uint8_t bgr_putpalette[] = {
      0, 255, 0, 0, 0,  0, 
      255, 255, 255,  0, 0, 255, 
      255, 97, 0, 255, 255, 0, 
      255, 80, 0, 160, 32, 240};

  cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
  uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();

  for (int h = 0; h < parsing_height; ++h) {
    for (int w = 0; w < parsing_width; ++w) {
      auto id = seg.seg[h * parsing_width + w];
      *parsing_img_ptr++ = bgr_putpalette[id * 3];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
      *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
    }
  }

  // resize parsing image
  cv::resize(parsing_img, parsing_img, mat.size(), 0, 0);

  // alpha blending
  float alpha_f = 0.5;
  cv::Mat dst;
  addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
  mat = std::move(dst);

  // 3. render detection result
  static cv::Scalar colors[] = {
      cv::Scalar(255, 0, 0),    // red
      cv::Scalar(255, 255, 0),  // yellow
      cv::Scalar(0, 255, 0),    // green
      cv::Scalar(0, 0, 255),    // blue
      cv::Scalar(0, 255, 255),    // perple
  };

  for (size_t idx = 0; idx < dets.size(); idx++){
    auto &det = dets[idx];
    auto &color = colors[det.id];

    cv::rectangle(mat,
        cv::Point(det.bbox.xmin, det.bbox.ymin),
        cv::Point(det.bbox.xmax, det.bbox.ymax),
        color, 1.5);

    cv::putText(mat,
        det.class_name,
        cv::Point2f(det.bbox.xmin, det.bbox.ymin - 10),
        cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        1);

  }

  std::string saving_path = "result/render_parking_" +
                            ai_msg->header.frame_id + "_" +
                            std::to_string(ai_msg->header.stamp.sec) + ".jpg";

  if (!feed_image_.empty()){
    RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
                "Draw result to file: %s",
                saving_path.c_str());
  }
  cv::imwrite(saving_path, mat);

  // return 0;
}


#ifdef SHARED_MEM_ENABLED
void ParkingPerceptionNode::SharedMemImgProcess(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!img_msg || !rclcpp::ok()) {
    return;
  }

  struct timespec time_start = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_start);

  RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
               "go into shared mem");

  if (debug_info_){
    std::stringstream ss;
    ss << "Recved img encoding: "
      << std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))
      << ", h: " << img_msg->height << ", w: " << img_msg->width
      << ", step: " << img_msg->step << ", index: " << img_msg->index
      << ", stamp: " << img_msg->time_stamp.sec << "_"
      << img_msg->time_stamp.nanosec << ", data size: " << img_msg->data_size;
    RCLCPP_INFO(rclcpp::get_logger("parking_perception"), "%s", ss.str().c_str());
  }

  // dump recved img msg
  // std::ofstream ofs("img_" + std::to_string(img_msg->index) + "." +
  // std::string(reinterpret_cast<const char*>(img_msg->encoding.data())));
  // ofs.write(reinterpret_cast<const char*>(img_msg->data.data()),
  //   img_msg->data_size);

  auto tp_start = std::chrono::system_clock::now();

  // 1. 将图片处理成模型输入数据类型DNNInput
  // 使用图片生成pym，NV12PyramidInput为DNNInput的子类
  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  if ("nv12" ==
      std::string(reinterpret_cast<const char *>(img_msg->encoding.data()))) {
    pyramid = ImageUtils::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char *>(img_msg->data.data()), img_msg->height,
        img_msg->width, model_input_height_, model_input_width_);
  } else {
    RCLCPP_INFO(rclcpp::get_logger("parking_perception"),
                "share mem unsupported img encoding: %s", img_msg->encoding);
  }

  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Get Nv12 pym fail!");
    return;
  }

  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
                 "after GetNV12Pyramid cost ms: %d", interval);
  }

  // 2. 使用pyramid创建DNNInput对象inputs
  // inputs将会作为模型的输入通过RunInferTask接口传入
  auto inputs = std::vector<std::shared_ptr<DNNInput>>{pyramid};
  auto dnn_output = std::make_shared<ParkingPerceptionOutput>();
  dnn_output->src_img_width_ = img_msg->width;
  dnn_output->src_img_height_ = img_msg->height;
  dnn_output->image_msg_header_ = std::make_shared<std_msgs::msg::Header>();
  dnn_output->image_msg_header_->set__frame_id(std::to_string(img_msg->index));
  dnn_output->image_msg_header_->set__stamp(img_msg->time_stamp);

  if(dump_render_img_){
    dnn_output->pyramid = pyramid;
  }
  
  dnn_output->preprocess_timespec_start = time_start;
  struct timespec time_now = {0, 0};
  clock_gettime(CLOCK_REALTIME, &time_now);
  dnn_output->preprocess_timespec_end = time_now;

  // 3. 开始预测
  int ret = Predict(inputs, nullptr, dnn_output);
  {
    auto tp_now = std::chrono::system_clock::now();
    auto interval =
        std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start)
            .count();
    RCLCPP_DEBUG(rclcpp::get_logger("parking_perception"),
                 "after Predict cost ms: %d", interval);
  }

  // 4. 处理预测结果，如渲染到图片或者发布预测结果
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("parking_perception"),
                 "Run predict failed!");
  }
  return;
}
#endif
