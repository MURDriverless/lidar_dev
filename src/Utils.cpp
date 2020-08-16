#include "Utils.h"

using namespace nvinfer1;
using namespace std;

//#define USE_GPU

/**
 * prepareImage
 * Performs image preprocessing before image is fed into the TensorRT network.
 * Resize all input images to be 32 x 32 for LiDAR image classifier
 * 
 * @param img input image from the lidar img_node crop
 * @param data output which will be used for inference
 * @param cvtColor boolean specifying whether to convert HWC to CHW
 * 
 * TODO: perform all image pre-processing first with CPU 
 * later optimise for GPU if necessary
 */
void prepareImage(
    cv::Mat &img, float *data, int w, int h, int c,
    bool cvtColor, bool normalize)
{
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(32, 32));

    if (cvtColor)
    {
        cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    }

    float factor = 1.0;
    if (normalize)
    {
        factor = 1 / 255.0;
    }

    cv::Mat img_float;
    if (c == 3)
    {
        resized.convertTo(img_float, CV_32FC3, factor);
    }
    else
    {
        resized.convertTo(img_float, CV_32FC1, factor);
    }

    // convert HWC to CHW
    cv::Mat input_channels[c];
    cv::split(img_float, input_channels);

    int channelLength = h * w;
    for (int i = 0; i < c; ++i)
    {
        memcpy(data, input_channels[i].data, channelLength * sizeof(float));
        data += channelLength;
    }
}

/**
 * onnxToTRTModel
 * Converts a given ONNX model into TensorRT Model.
 * @param modelFile the ONNX model to convert
 * @param maxBatchSize maximum batch size for converted TensorRT model
 * @param trtModelStream converted TensorRT model
 */
void onnxToTRTModel(
    const std::string &modelFile,
    unsigned int maxBatchSize,
    IHostMemory *&trtModelStream,
    Logger &logger,
    bool useInt8,
    bool markOutput,
    IInt8EntropyCalibrator *calibrator)
{
    IBuilder *builder = createInferBuilder(logger);

    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);

    auto parser = nvonnxparser::createParser(*network, logger);
    parser->parseFromFile(modelFile.c_str(), 2);

    builder->setMaxBatchSize(maxBatchSize);
    builder->setMaxWorkspaceSize(1 << 20);

    if (useInt8 && builder->platformHasFastInt8())
    {
        builder->setInt8Mode(true);
        builder->setInt8Calibrator(calibrator);
    }
    else
    {
        builder->setFp16Mode(true);
    }

    builder->setStrictTypeConstraints(true);
    if (markOutput)
    {
        network->markOutput(*network->getLayer(network->getNbLayers() - 1)->getOutput(0));
    }

    auto config = builder->createBuilderConfig();
    config->setMaxWorkspaceSize(1 << 20);

    ICudaEngine *engine = builder->buildEngineWithConfig(*network, *config);
    assert(engine);

    assert(network->getNbInputs() == 1);
    auto inputDims = network->getInput(0)->getDimensions();
    assert(inputDims.nbDims == 4); // b, c, h, w

    assert(network->getNbOutputs() == 1);
    auto outputDims = network->getOutput(0)->getDimensions();
    assert(inputDims.nbDims == 1);

    parser->destroy();
    trtModelStream = engine->serialize();
    engine->destroy();
    network->destroy();
    builder->destroy();
}

ICudaEngine *engineFromFiles(
    string onnxFile,
    string trtFile,
    IRuntime *runtime,
    int batchSize,
    Logger &logger,
    bool useInt8,
    bool markOutput,
    IInt8EntropyCalibrator *calibrator)
{
    ICudaEngine *engine;
    fstream file;
    file.open(trtFile, ios::binary | ios::in);

    if (!file.is_open())
    {
        // no existing model found, create new tensorrt model
        IHostMemory* trtModelStream{nullptr};
        onnxToTRTModel(onnxFile, batchSize, trtModelStream, logger, useInt8, markOutput, calibrator);
        assert(trtModelStream != nullptr);

        engine = runtime->deserializeCudaEngine(trtModelStream->data(), trtModelStream->size(), nullptr);
        assert(engine != nullptr);
        trtModelStream->destroy();

        IHostMemory* data = engine->serialize();
        std::ofstream save_file;
        save_file.open(trtFile, ios::binary | ios::out);
        save_file.write((const char*)data->data(), data->size());
        save_file.close();
    }
    else
    {
        // existing model found, load existing model
        file.seekg(0, ios::end);
        int length = file.tellg(); // token value used to seek to same place
        file.seekg(0, ios::beg);
        unique_ptr<char[]> data(new char[length]);
        file.read(data.get(), length);
        file.close();

        engine = runtime->deserializeCudaEngine(data.get(), length, nullptr);
        assert(engine != nullptr);
    }
    return engine;
}