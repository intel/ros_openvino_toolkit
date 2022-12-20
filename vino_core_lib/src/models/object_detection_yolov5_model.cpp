/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @brief a header file with declaration of ObjectDetectionYOLOV5vocModel class
 * @file object_detection_model.cpp
 */
#include <string>
#include "vino_core_lib/models/object_detection_yolov5_model.h"
#include "vino_core_lib/slog.h"

Models::ObjectDetectionYolov5Model::ObjectDetectionYolov5Model(const std::string& model_loc, int max_batch_size)
  : ObjectDetectionModel("", model_loc, max_batch_size)
{
}

bool Models::ObjectDetectionYolov5Model::updateLayerProperty(InferenceEngine::CNNNetwork& net_reader)
{
  slog::info << "Checking INPUTS for Model" << getModelName() << slog::endl;

  auto network = net_reader;

  InferenceEngine::InputsDataMap input_info_map(net_reader.getInputsInfo());

  if(input_info_map.size() != 1)
  {
    slog::warn << "This model seems not YoloV5-like, YoloV5 has only one input, but we got "
               << std::to_string(input_info_map.size()) << "inputs" << slog::endl;
    return false;
  }

  input_info_ = input_info_map.begin()->second;
  input_info_->setPrecision(InferenceEngine::Precision::FP32);
  input_info_->getInputData()->setLayout(InferenceEngine::Layout::NCHW);

  addInputInfo("input", input_info_map.begin()->first);

  const InferenceEngine::SizeVector input_dims = input_info_->getTensorDesc().getDims();
  setInputHeight(input_dims[2]);
  setInputWidth(input_dims[3]);

  InferenceEngine::ICNNNetwork::InputShapes inputShapes = network.getInputShapes();
  InferenceEngine::SizeVector& in_size_vector = inputShapes.begin()->second;

  slog::debug << "input size" << inputShapes.size() << slog::endl;
  if (inputShapes.size() != 1)
  {
    slog::warn << "This inference sample should have only one input, but we got" << std::to_string(inputShapes.size())
               << "inputs" << slog::endl;
    return false;
  }

  network.reshape(inputShapes);

  // output configure
  outputs_data_map_ = network.getOutputsInfo();

  // if (outputs_data_map_.size() != 1)
  // {
  //   slog::warn << "This inference sample should have only one output, but we got"
  //              << std::to_string(outputs_data_map_.size()) << "outputs" << slog::endl;
  //   return false;
  // }

  addOutputInfo("output", outputs_data_map_.begin()->first);

  for(auto &output : outputs_data_map_) 
  {
    auto output_data_ptr = output.second;
    output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);

    const InferenceEngine::SizeVector output_dims = output_data_ptr->getTensorDesc().getDims();
    setMaxProposalCount(static_cast<int>(output_dims[2]));
    slog::info << "max proposal count is: " << getMaxProposalCount() << slog::endl;

    auto object_size = static_cast<int>(output_dims[3]);
    // if (object_size != 7)
    // {
    //   slog::warn << "This model is NOT SSDNet-like, whose output data for each detected object"
    //             << "should have 7 dimensions, but was " << std::to_string(object_size) << slog::endl;
    //   return false;
    // }
    setObjectSize(object_size);

    // if (output_dims.size() != 4)
    // {
    //   slog::warn << "This model is not SSDNet-like, output dimensions shoulld be 4, but was"
    //             << std::to_string(output_dims.size()) << slog::endl;
    //   return false;
    // }

    // printAttribute();
  }

  // InferenceEngine::DataPtr& output_data_ptr = outputs_data_map_.begin()->second;
  // output_data_ptr->setPrecision(InferenceEngine::Precision::FP32);
  slog::info << "Checking YoloV5 Object Detection output ... Name=" << outputs_data_map_.begin()->first << slog::endl;

  // TODO set size

  printAttribute();
  slog::info << "This model is Yolov5-like, Layer Property updated!" << slog::endl;

  return true;
}

const std::string Models::ObjectDetectionYolov5Model::getModelCategory() const
{
  return "Object Detection YoloV5";
}

bool Models::ObjectDetectionYolov5Model::enqueue(const std::shared_ptr<Engines::Engine>& engine, const cv::Mat& frame,
                                                 const cv::Rect& input_frame_loc)
{
  setFrameSize(frame.cols, frame.rows); // demo picture is 640 * 640

  if (!matToBlob(frame, input_frame_loc, 1, 0, engine))
  {
    return false;
  }

  return true;
}

bool Models::ObjectDetectionYolov5Model::matToBlob(const cv::Mat& orig_image, const cv::Rect&, float scale_factor,
                                                   int batch_index, const std::shared_ptr<Engines::Engine>& engine)
{
  if (engine == nullptr)
  {
    slog::err << "ObjectDetectionYolov5Model: A frame is trying to be enqueued in a NULL Engine." << slog::endl;
    return false;
  }

  std::string input_name = getInputName();
  slog::debug << "ObjectDetectionYolov5Model: add input image to blob: " << input_name << slog::endl;
  InferenceEngine::Blob::Ptr input_blob = engine->getRequest()->GetBlob(input_name);

  InferenceEngine::LockedMemory<void> blobMapped = InferenceEngine::as<InferenceEngine::MemoryBlob>(input_blob)->wmap();

  auto resized_image(orig_image);

  InferenceEngine::SizeVector blob_size = input_blob->getTensorDesc().getDims();
  const int width = blob_size[3];
  const int height = blob_size[2];
  const int channels = blob_size[1];

  if(width != orig_image.size().width || height != orig_image.size().height)
  {
    cv::resize(orig_image, resized_image, cv::Size(width, height));
  }

  float* blob_data = blobMapped.as<float*>();
  size_t img_size = width*height;

  //nchw
  for(size_t row =0; row < height; row++)
  {
      for(size_t col=0; col < width; col++)
      {
          for(size_t ch =0; ch < channels; ch++)
          {
            blob_data[img_size*ch + row*width + col] = float(resized_image.at<cv::Vec3b>(row,col)[ch])/255.0f;
          }
      }
  }

  slog::debug << "Convert input image to blob: DONE!" << slog::endl;

  return true;
}

bool Models::ObjectDetectionYolov5Model::fetchResults(const std::shared_ptr<Engines::Engine>& engine,
                                                      std::vector<vino_core_lib::ObjectDetectionResult>& results,
                                                      const float& confidence_thresh, const bool& enable_roi_constraint)
{
  // TODO xiansen
  auto confidence_thresh_ = 0.1;
  slog::debug << "fetching Infer Resulsts from the given Yolov5 model" << slog::endl;

  if (engine == nullptr)
  {
    slog::err << "Trying to fetch Yolov5 results from <null> Engines." << slog::endl;
    return false;
  }

  slog::debug << "Fetching Yolov5 Detection Results ..." << slog::endl;

  InferenceEngine::InferRequest::Ptr request = engine->getRequest();
  std::string output = getOutputName();

  std::vector<cv::Rect> origin_rect;
  std::vector<float>    origin_rect_cof;

  int s[3] = {80,40,20};
  int index=0;

  for (auto &output : outputs_data_map_) 
  {
      auto output_name = output.first;
      InferenceEngine::Blob::Ptr blob = request->GetBlob(output_name);
      parseYolov5(blob,s[index], confidence_thresh_ ,origin_rect, origin_rect_cof);
      ++index;
  }

  // TODO xiansen nms_area_threshold
  double nms_area_threshold = 0.5;

  //后处理获得最终检测结果
  std::vector<int> final_id;
  cv::dnn::NMSBoxes(origin_rect, origin_rect_cof,
                confidence_thresh_ ,nms_area_threshold,final_id);
  
    // 根据final_id获取最终结果
    for(int i=0;i<final_id.size();++i)
    {
        cv::Rect resize_rect= origin_rect[final_id[i]];
        vino_core_lib::ObjectDetectionResult result(resize_rect);
        result.setConfidence(origin_rect_cof[final_id[i]]);
        result.setLabel("");
        results.push_back(result);
    }

  slog::debug << "Analyzing YoloV5 Detection results..." << slog::endl;

  return true;
}

std::ostream & operator<<(std::ostream & os,const cv::Rect & rect)
{
    os << "[x: " << rect.x << ", y: " << rect.y 
       << ", width: " << rect.width << ", height: " << rect.height << "]" << std::endl;
    return os;
}

template <class T>
std::ostream & operator<<(std::ostream &os, const std::vector<T> v)
{
  os << "{"; 

  for(auto it: v)
  {
    os << it << " ,";
  }

  os << "}\n";

  return os;
}

//注意此处的阈值是框和物体prob乘积的阈值
bool Models::ObjectDetectionYolov5Model::parseYolov5(const InferenceEngine::Blob::Ptr &blob, int net_grid, float cof_threshold,
                            std::vector<cv::Rect>& o_rect, std::vector<float>& o_rect_cof)
{
  std::vector<int> anchors = getAnchors(net_grid);
  InferenceEngine::LockedMemory<const void> blobMapped = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob)->rmap();
  const float *output_blob = blobMapped.as<float *>();
   //80个类是85,一个类是6,n个类是n+5
   //int item_size = 6;
    int item_size = 85;
    size_t anchor_n = 3;
    for(int n=0;n<anchor_n;++n)
    {
      for(int i=0;i<net_grid;++i)
      {
        for(int j=0;j<net_grid;++j)
        {
            double box_prob = output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j *item_size+ 4];
            box_prob = sigmoid(box_prob);
            //框置信度不满足则整体置信度不满足
            if(box_prob < cof_threshold)
                continue;
            
            //注意此处输出为中心点坐标,需要转化为角点坐标
            double x = output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j*item_size + 0];
            double y = output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j*item_size + 1];
            double w = output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j*item_size + 2];
            double h = output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j *item_size+ 3];
            
            double max_prob = 0;
            int idx=0;
            for(int t=5;t<85;++t){
                double tp= output_blob[n*net_grid*net_grid*item_size + i*net_grid*item_size + j *item_size+ t];
                tp = sigmoid(tp);
                if(tp > max_prob){
                    max_prob = tp;
                    idx = t;
                }
            }
            float cof = box_prob * max_prob;                
            //对于边框置信度小于阈值的边框,不关心其他数值,不进行计算减少计算量
            if(cof < cof_threshold)
                continue;

            x = (sigmoid(x)*2 - 0.5 + j)*640.0f/net_grid;
            y = (sigmoid(y)*2 - 0.5 + i)*640.0f/net_grid;
            w = pow(sigmoid(w)*2,2) * anchors[n*2];
            h = pow(sigmoid(h)*2,2) * anchors[n*2 + 1];

            double r_x = x - w/2;
            double r_y = y - h/2;
            cv::Rect rect(round(r_x),round(r_y),round(w),round(h));
            o_rect.push_back(rect);
            o_rect_cof.push_back(cof);
        }
      }
    }

    if(o_rect.size() == 0) 
    {
      return false;
    }

    return true;
}

//以下为工具函数
// int Models::ObjectDetectionYolov5Model::getEntryIndex(int side, int lcoords, int lclasses, int location, int entry)
// {
//   int n = location / (side * side);
//   int loc = location % (side * side);
//   return n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc;
// }

double Models::ObjectDetectionYolov5Model::sigmoid(double x)
{
  return (1 / (1 + exp(-x)));
}

std::vector<int> Models::ObjectDetectionYolov5Model::getAnchors(int net_grid)
{
    std::vector<int> anchors(6);
    int a80[6] = {10,13, 16,30, 33,23};
    int a40[6] = {30,61, 62,45, 59,119};
    int a20[6] = {116,90, 156,198, 373,326}; 
    if(net_grid == 80)
    {
        anchors.insert(anchors.begin(),a80,a80 + 6);
    }
    else if(net_grid == 40)
    {
        anchors.insert(anchors.begin(),a40,a40 + 6);
    }
    else if(net_grid == 20)
    {
        anchors.insert(anchors.begin(),a20,a20 + 6);
    }
    return anchors;
}


REG_MODEL(ObjectDetectionYolov5Model, "ObjectDetection_YOLOV5s");
