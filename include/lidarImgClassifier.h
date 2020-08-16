#pragma once

#include <opencv2/opencv.hpp>

#include "NvInfer.h"
#include "NvOnnxParser.h"

#include "Utils.h"

static const int IMG_CHANNEL = 1;

// #define CLASSIFIER_PROFILE

class LidarImgClassifier
{
public:
    LidarImgClassifier(std::string onnxFile,
                       std::string trtFile,
                       int input_w,
                       int input_h,
                       int max_batch);
    ~LidarImgClassifier();
    std::vector<int> doInference(std::vector<cv::Mat>& imgs);
private:
    int interpretOutputTensor(float *tensor, int width, int height);
    Logger logger_;

    nvinfer1::IExecutionContext* context_;
    nvinfer1::ICudaEngine* engine_;
    nvinfer1::IRuntime* runtime_;
    std::unique_ptr<float[]> inputData_;
    std::unique_ptr<float[]> outputData_;
    cudaStream_t stream_;
    std::unique_ptr<void*[]> buffers_;

    int inputW_;
    int inputH_;
    int maxBatch_;
}