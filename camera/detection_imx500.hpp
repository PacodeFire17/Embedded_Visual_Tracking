#pragma once

#include <cstdint>
#include <libcamera/libcamera.h>
#include <vector>

/**
 * @file detection_imx500.hpp
 * @brief Configuration constants and data structures for the IMX500 AI camera and object detection.
 */

namespace imx500
{
    /** @name Metadata Layout Constants */
    ///@{
    /// Maximum number of output tensors supported by CnnOutputTensorInfo.
    constexpr int MAX_TENSOR = 4;
    /// Maximum number of dimensions supported per tensor.
    constexpr int MAX_NUM_DIMENSION = 4;
    ///@}

    /** @name Camera Stream Parameters */
    ///@{
    /// Width of the image stream used by libcamera.
    constexpr int W = 640;
    /// Height of the image stream used by libcamera.
    constexpr int H = 480;
    ///@}

    /** @name Neural Network Input Parameters */
    ///@{
    /// Input dimension for YOLO11n (1:1 aspect ratio).
    constexpr float Input_W = 640.0f;
    ///@}

    /** @name Detection Parameters */
    ///@{
    /// Minimum confidence score for a detection to be kept.
    constexpr float SCORE_THRESHOLD = 0.5f;
    /// Path of the network file (.rpk) loaded on the sensor.
    constexpr const char *MODEL_PATH = "/home/raspberry/imx500-models/imx500_network_yolo11n_pp.rpk";
    ///@}

    /** @name IMX500 Sensor Specifications */
    ///@{
    /// Native sensor width of the IMX500.
    constexpr float SENSOR_W = 4056.0f;
    /// Native sensor height of the IMX500.
    constexpr float SENSOR_H = 3040.0f;
    ///@}

    /** @name Network Cropping Parameters */
    ///@{
    /// Horizontal offset to crop the sensor image to a 1:1 aspect ratio.
    constexpr float NET_CROP_X = (SENSOR_W - SENSOR_H) / 2.0f;
    /// Vertical offset to crop the sensor image to a 1:1 aspect ratio.
    constexpr float NET_CROP_Y = 0.0f;
    /// Size of the square crop (equal to sensor height) to achieve a 1:1 aspect ratio.
    constexpr float NET_CROP_SIZE = SENSOR_H;
    ///@}

    /**
     * @brief Output tensor metadata structure for libcamera.
     * @see https://www.raspberrypi.com/documentation/accessories/ai-camera.html#reference
     */
    struct OutputTensorInfo
    {
        uint32_t tensorDataNum;           ///< Number of data elements in the tensor.
        uint32_t numDimensions;           ///< Number of dimensions of the tensor.
        uint16_t size[MAX_NUM_DIMENSION]; ///< Size of each dimension.
    };

    /**
     * @brief CNN output tensor metadata structure for libcamera.
     * @see https://www.raspberrypi.com/documentation/accessories/ai-camera.html#reference
     */
    struct CnnOutputTensorInfo
    {
        char networkName[64];              ///< Name of the neural network model.
        uint32_t numTensors;               ///< Number of output tensors.
        OutputTensorInfo info[MAX_TENSOR]; ///< Metadata for each output tensor.
    };

    /**
     * @brief High-level representation of a single detected object.
     */
    struct Detection
    {
        int cls;     ///< Class index of the detected object.
        float score; ///< Confidence score of the detection (>= 0.5).
        float x;     ///< X coordinate of the bounding box origin.
        float y;     ///< Y coordinate of the bounding box origin.
        float w;     ///< Width of the bounding box.
        float h;     ///< Height of the bounding box.
        float cx;    ///< X coordinate of the center of the bounding box.
        float cy;    ///< Y coordinate of the center of the bounding box.
        float errX;  ///< Horizontal error (distance) between the image center and the bounding box center.
        float errY;  ///< Vertical error (distance) between the image center and the bounding box center.
    };

    /**
     * @brief Signal handler to gracefully handle termination signals (SIGINT, SIGTERM).
     * @param signum Signal number received.
     */
    void signalHandler(int signum);

    /**
     * @brief Parses CNN output metadata from libcamera and extracts detections.
     * @param metadata ControlList containing camera frame metadata.
     * @return Vector of parsed Detection objects.
     */
    std::vector<Detection> parseDetections(const libcamera::ControlList &metadata);

} // namespace imx500
