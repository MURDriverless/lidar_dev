#include <assert.h>
// #include <cublas_v2.h>
// #include <cudnn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "lidarImgClassifier.h"

using namespace nvinfer1;
using namespace std;

LidarImgClassifier::LidarImgClassifier(
    string onnxFile,
    string trtFile,
    int input_w,
    int input_h,
    int max_batch) : logger_(Logger::Severity::kINFO),
                     inputW_(input_w),
                     inputH_(input_h),
                     maxBatch_(max_batch)
{
    runtime_ = createInferRuntime(logger_);
    assert(runtime_ != nullptr);
    runtime_->setDLACore(0);

    engine_ = engineFromFiles(onnxFile, trtFile, runtime_, maxBatch_, logger_, false);

    context_ = engine_->createExecutionContext();

    assert(context_ != nullptr);

    int64_t outputCount = 0;
    int nbBindings = engine_->getNbBindings();
    for (int i = 0; i < nbBindings; ++i)
    {
        if (!engine_->bindingIsInput(i))
        {
            outputCount += volume(engine_->getBindingDimensions(i));
        }
    }

    outputData_.reset(new float[outputCount * maxBatch_]);
    inputData_.reset(new float[inputW_ * inputH_ * IMG_CHANNEL * maxBatch_]);

    CUDA_CHECK(cudaStreamCreate(&stream_));

    buffers_.reset(new void *[nbBindings]);

    for (int b = 0; b < nbBindings; ++b)
    {
        int64_t size = volume(engine_->getBindingDimensions(b));
        CUDA_CHECK(cudaMalloc(&buffers_.get()[b], size * maxBatch_ * sizeof(float)));
    }
}

LidarImgClassifier::~LidarImgClassifier()
{
    // release streams and buffers
    cudaStreamDestroy(stream_);
    for (int b = 0; b < engine_->getNbBindings(); ++b)
    {
        CUDA_CHECK(cudaFree(buffers_.get()[b]));
    }
    // destry the engine_
    context_->destroy();
    engine_->destroy();
    runtime_->destroy();
}

int LidarImgClassifier::interpretOutputTensor(float *tensor)
{
    // TODO: parse outputSize from network
    const int outputSize = 2;
    float* output = tensor;
    float val = 0.0f;
    int idx = 0; // 0 blue, 1 yellow

    // calculate softmax
    float sum{0.0f};
    for (int i = 0; i < outputSize; i++)
    {
        output[i] = exp(output[i]);
        sum += output[i];
    }

    for (int i = 0; i < outputSize; i++)
    {
        output[i] /= sum;
        val = std::max(val, output[i]);
        if (val == output[i])
        {
            idx = i;
        }
    }

    return idx;
}

vector<int> LidarImgClassifier::doInference(vector<cv::Mat> & imgs)
{
    const int outputSize = 2;
    int batchSize = imgs.size();
    float *input = inputData_.get();

    for (auto &img : imgs)
    {
        prepareImage(img, input, inputW_, inputH_, IMG_CHANNEL, false, false);
        input += inputW_ * inputH_ * IMG_CHANNEL;
    }
    auto t_start = chrono::high_resolution_clock::now();

    int nbBindings = engine_->getNbBindings();

    // copy input to GPU, execute batch asynchronously, then copy back
    CUDA_CHECK(cudaMemcpyAsync(
        buffers_.get()[0], 
        inputData_.get(), 
        batchSize * volume(engine_->getBindingDimensions(0)) * sizeof(float), 
        cudaMemcpyHostToDevice, stream_
        ));
    
    context_->enqueue(batchSize, buffers_.get(), stream_, nullptr);

    float *output = outputData_.get();
    for (int b = 0; b < nbBindings; ++b)
    {
        if (!engine_->bindingIsInput(b))
        {
            int64_t size = volume(engine_->getBindingDimensions(b));
            CUDA_CHECK(cudaMemcpyAsync(
                output,
                buffers_.get()[b],
                batchSize *size * sizeof(float),
                cudaMemcpyDeviceToHost,
                stream_
            ));
            output+= maxBatch_ * size;
        }
    }
    cudaStreamSynchronize(stream_);

    auto t_end = std::chrono::high_resolution_clock::now();
    auto total = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    std::cout << "Time take for classification is " << total << " ms." << std::endl;

    output = outputData_.get();
    vector<int> results(batchSize);
    for (int b = 0; b < batchSize; b++)
    {
        results[b] = interpretOutputTensor(output);
        output += outputSize; // ? not sure if this is correct? check with NV exmaples
    }
    return results;
}