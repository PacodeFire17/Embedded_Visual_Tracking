#pragma once

#include <string_view>

/**
 * @file model_spec.hpp
 * @brief What changes from one IMX500 detection model to another.
 */

namespace imx500
{
    /// How the network writes the four box coordinates.
    enum class BoxFormat
    {
        PixelsXY,     ///< x0, y0, x1, y1 in pixels of the network input (YOLO)
        PixelsYX,     ///< y0, x0, y1, x1 in pixels of the network input (EfficientDet-Lite0)
        NormalizedYX, ///< y0, x0, y1, x1 already in [0, 1] (SSD, NanoDet)
    };

    /// Description of one detection model.
    struct ModelSpec
    {
        const char *name;    ///< Short name used on the command line and in the log file name.
        const char *file;    ///< .rpk file in MODEL_DIR.
        float inputSize;     ///< Network input side in pixels (square input).
        BoxFormat boxFormat; ///< Layout of the box coordinates.
        bool labels90;       ///< True: class index in the 90-entry COCO list with gaps (SSD).
    };

    /// Models that can be compared. The first one is the default.
    inline constexpr ModelSpec MODELS[] = {
        {"yolo11n", "imx500_network_yolo11n_pp.rpk", 640.0f, BoxFormat::PixelsXY, false},
        {"yolov8n", "imx500_network_yolov8n_pp.rpk", 640.0f, BoxFormat::PixelsXY, false},
        {"efficientdet", "imx500_network_efficientdet_lite0_pp.rpk", 320.0f, BoxFormat::PixelsYX, true},
        {"ssd", "imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk", 320.0f, BoxFormat::NormalizedYX, true},
        {"nanodet", "imx500_network_nanodet_plus_416x416_pp.rpk", 416.0f, BoxFormat::NormalizedYX, false},
    };

    /// Model with the given name, or nullptr if there is none.
    constexpr const ModelSpec *findModel(std::string_view name)
    {
        for (const ModelSpec &m : MODELS)
        {
            if (name == m.name)
            {
                return &m;
            }
        }
        return nullptr;
    }

    /// Positions of the 10 unused ids ("-") in the 90-entry COCO label list used by SSD.
    inline constexpr int COCO90_GAPS[] = {11, 25, 28, 29, 44, 65, 67, 68, 70, 82};

    /// Converts a 90-entry COCO index to the 80-entry one used by YOLO (-1 for an unused id).
    constexpr int coco90To80(int cls90)
    {
        int shift = 0;
        for (int gap : COCO90_GAPS)
        {
            if (cls90 == gap)
            {
                return -1;
            }
            if (cls90 > gap)
            {
                ++shift;
            }
        }
        return cls90 - shift;
    }
} // namespace imx500
