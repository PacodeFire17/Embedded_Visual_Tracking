/**
 * @file detection_imx500.cpp
 * @brief Implementation of IMX500 AI camera detection pipeline and acquisition loop.
 */

#include "detection_imx500.hpp"
#include "frame_timer.hpp"
#include "imx500_network.hpp"
#include "frame_logger.hpp"
#include "model_spec.hpp"


#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/control_ids.h>

using namespace libcamera;

namespace
{
    ///
    constexpr bool ENABLE_TIMER = false; ///< Flag to enable frame timing logs for performance monitoring.
    /// Flag indicating whether image acquisition should continue.
    std::atomic<bool> running{true};
    /// Becomes true at the first frame carrying the CNN output: the network is running on the sensor.
    std::atomic<bool> networkReady{false};
    /// Shared smart pointer to the Camera object representing the physical IMX500 sensor.
    std::shared_ptr<Camera> camera;


    /// CSV log of the run; created in main, written by the request callback.
    std::unique_ptr<imx500::FrameLogger> logger;
    /// Progressive number of the completed frames.
    std::uint64_t frameCount = 0;
    /// Also print the detections on screen (the CSV log has them anyway).
    constexpr bool PRINT_DETECTIONS = true;

    /// Detection model chosen on the command line.
    const imx500::ModelSpec *model = &imx500::MODELS[0];

} // namespace

namespace imx500
{
    /**
     * @brief Signal handler to gracefully stop acquisition on SIGINT or SIGTERM.
     * @param signum Signal number received.
     */
    void signalHandler(int signum)
    {
        if (signum == SIGINT || signum == SIGTERM)
        {
            running = false;
        }
    }

    /**
     * @brief Parses CNN output metadata from libcamera and extracts detections.
     * @param metadata ControlList containing camera frame metadata.
     * @param model The model specification to use for parsing.
     * @return Vector of parsed Detection objects.
     */
    std::vector<Detection> parseDetections(const ControlList &metadata, const ModelSpec &model)
    {
        // 1. Extraction and verification of binary metadata
        std::vector<Detection> results;

        auto tensorOpt = metadata.get(controls::rpi::CnnOutputTensor);
        auto tensorInfoOpt = metadata.get(controls::rpi::CnnOutputTensorInfo);

        if (!tensorOpt || !tensorInfoOpt || tensorInfoOpt->size() != sizeof(CnnOutputTensorInfo))
        {
            return results;
        }

        const auto *info = reinterpret_cast<const CnnOutputTensorInfo *>(tensorInfoOpt->data());
        [[maybe_unused]] const auto *data = tensorOpt->data();

        if (info->numTensors < 3)
        {
            return results;
        }

        // 2. Splitting the single buffer into three arrays (Slicing)

        // Number of box coordinates
        size_t boxesLen = info->info[0].tensorDataNum;
        // Number of scores
        size_t scoresLen = info->info[1].tensorDataNum;
        // Number of detections
        size_t numDet = boxesLen / 4;

        // The buffer must hold boxes, scores and classes
        if (tensorOpt->size() < boxesLen + scoresLen + numDet)
        {
            return results;
        }

        // Pointer to the boxes array
        const float *boxes = data;
        // Pointer to the scores array
        const float *scores = data + boxesLen;
        // Pointer to the classes array
        const float *classes = data + boxesLen + scoresLen;

        // 3. Reading the video stream crop window (ScalerCrop)
        // Query the camera for the ScalerCrop metadata, which defines the region
        // of interest in the sensor's field of view used for the prediction.
        // If this metadata is not available, we assume the entire sensor area is used.
        float cropX = 0.0f, cropY = 0.0f, cropW = SENSOR_W, cropH = SENSOR_H;
        if (auto scalerCrop = metadata.get(controls::ScalerCrop))
        {
            cropX = static_cast<float>(scalerCrop->x);
            cropY = static_cast<float>(scalerCrop->y);
            cropW = static_cast<float>(scalerCrop->width);
            cropH = static_cast<float>(scalerCrop->height);
        }

        // 4. Loop over detections, filtering and normalization
        for (size_t i = 0; i < numDet; i++)
        {
            float score = scores[i];
            if (score < SCORE_THRESHOLD)
            {
                continue; // Skip detections with low confidence
            }

            int cls = static_cast<int>(classes[i]);
            if (model.labels90)
            {
                cls = coco90To80(cls); // same class numbers for every model
                if (cls < 0)
                {
                    continue; // unused COCO id
                }
            }

            // 5: Dual coordinate transformation (Network -> Sensor -> Video)

            // 5.1 The four box values of detection i (the tensor is stored column by column,
            //     like picamera2's reshape(order='F'): first all the 1st values, then all the 2nd, ...)
            const float c0 = boxes[0 * numDet + i];
            const float c1 = boxes[1 * numDet + i];
            const float c2 = boxes[2 * numDet + i];
            const float c3 = boxes[3 * numDet + i];

            // Normalized [0, 1] coordinates, whatever the model's layout
            float x0, y0, x1, y1;
            if (model.boxFormat == BoxFormat::PixelsXY)
            {
                // YOLO: x0, y0, x1, y1 in pixels of the network input
                x0 = c0 / model.inputSize;
                y0 = c1 / model.inputSize;
                x1 = c2 / model.inputSize;
                y1 = c3 / model.inputSize;
            }
            else if (model.boxFormat == BoxFormat::PixelsYX)
            {
                // EfficientDet-Lite0: y0, x0, y1, x1 in pixels of the network input
                y0 = c0 / model.inputSize;
                x0 = c1 / model.inputSize;
                y1 = c2 / model.inputSize;
                x1 = c3 / model.inputSize;
            }
            else
            {
                // SSD, NanoDet: y0, x0, y1, x1 already normalized
                y0 = c0;
                x0 = c1;
                y1 = c2;
                x1 = c3;
            }

            // 5.2 Geometric remapping: from network space (1:1) to sensor space (4:3)
            // The network input region is the whole sensor (set in main with setInferenceRoi),
            // squashed to the square network input: x is scaled by 4056 (SENSOR_W) and y by 3040 (SENSOR_H), with no offset.
            // We obtain sx0, sy0, sx1, sy1 (float): absolute bounding box coordinates on the physical sensor.
            float sx0 = x0 * SENSOR_W;
            float sy0 = y0 * SENSOR_H;
            float sx1 = x1 * SENSOR_W;
            float sy1 = y1 * SENSOR_H;

            // 5.3 From physical sensor to video frame (640 × 480)
            // Calculates the relative position (from 0.0 to 1.0)
            // of the object within the visible window of the video stream.
            // ix0, iy0, ix1, iy1 (float): final coordinates in pixels on the 640 × 480 frame
            // (top-left and bottom-right corners).
            float ix0 = std::clamp(((sx0 - cropX) / cropW) * W, 0.0f, static_cast<float>(W));
            float iy0 = std::clamp(((sy0 - cropY) / cropH) * H, 0.0f, static_cast<float>(H));
            float ix1 = std::clamp(((sx1 - cropX) / cropW) * W, 0.0f, static_cast<float>(W));
            float iy1 = std::clamp(((sy1 - cropY) / cropH) * H, 0.0f, static_cast<float>(H));

            // 6. Tracking metric computation and storage
            // Bounding box width
            float w = ix1 - ix0;
            // Bounding box height
            float h = iy1 - iy0;
            // Skip boxes left empty by the clamp (completely outside the video frame)
            if (w <= 0.0f || h <= 0.0f)
            {
                continue;
            }
            // X coordinate of the bounding box center
            float cx = ix0 + w / 2.0f;
            // Y coordinate of the bounding box center
            float cy = iy0 + h / 2.0f;

            results.push_back(Detection{
                cls,
                score,
                ix0, iy0, w, h,
                cx, cy,
                // Horizontal offset (error) from the image center
                cx - W / 2.0f,
                // Vertical offset (error) from the image center
                cy - H / 2.0f});
        }

        return results;
    }

    /**
     * @brief Callback invoked when a camera request is completed.
     *
     * Processes metadata to print detections and requeues the request
     * to keep the camera streaming.
     * @param request Pointer to the completed libcamera Request.
     */
    void onRequestCompleted(Request *request)
    {
        // Check if the request was cancelled or if acquisition has ended
        if (request->status() == Request::RequestCancelled || !running)
        {
            return;
        }

        const ControlList &metadata = request->metadata();

        // Capture time of this frame (CLOCK_BOOTTIME, ns) and whether it carries a network result
        const std::int64_t tSensor = metadata.get(controls::SensorTimestamp).value_or(0);
        const bool hasTensor = metadata.get(controls::rpi::CnnOutputTensor).has_value();
        // Run times measured by the sensor itself (µs): network, and DSP pre/post-processing
        std::int32_t dnnUs = -1, dspUs = -1;
        if (auto kpi = metadata.get(controls::rpi::CnnKpiInfo))
        {
            dnnUs = (*kpi)[0];
            dspUs = (*kpi)[1];
        }
        // The first output tensor means the network upload to the sensor is complete
        if (!networkReady && hasTensor)
        {
            networkReady = true;
        }


        // Optional timing of the parsing, for performance monitoring
        if constexpr (ENABLE_TIMER)
        {
            startFrameTimer();
        }

        const std::int64_t tParseStart = nowBoottimeNs();
        const auto detections = parseDetections(metadata, *model);
        // End of processing, on the same clock as tSensor: tProc - tSensor is the latency
        const std::int64_t tProc = nowBoottimeNs();

        if constexpr (ENABLE_TIMER)
        {
            endFrameTimer(8);
        }


        if (logger)
        {
            logger->logFrame(frameCount++, tSensor, tProc, tProc - tParseStart, hasTensor, dnnUs, dspUs, detections);
        }

        // Optional: also print the detections on screen (the CSV log has them anyway)
        if constexpr (PRINT_DETECTIONS)
        {
            for (const auto &det : detections)
            {
                std::printf("classe %2d  score %.2f  "
                            "box x=%.0f y=%.0f w=%.0f h=%.0f  "
                            "centro=(%.0f,%.0f)  errore=(%+.0f,%+.0f) px\n",
                            det.cls, det.score, det.x, det.y, det.w, det.h,
                            det.cx, det.cy, det.errX, det.errY);
            }
        }

        if (running)
        {
            // Re-queue the clean Request to the end of the camera hardware queue,
            // closing the loop and ensuring the video stream continues without interruption
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
        }
    }

} // namespace imx500

int main(int argc, char **argv)
{
    // Model choice: ./rilevamento_imx500 [--model yolo11n|yolov8n|efficientdet|ssd|nanodet]
    for (int a = 1; a < argc; ++a)
    {
        const std::string_view arg = argv[a];
        if (arg == "--model" && a + 1 < argc)
        {
            model = imx500::findModel(argv[++a]);
            if (!model)
            {
                std::cerr << "unknown model " << argv[a] << " (yolo11n, yolov8n, efficientdet, ssd, nanodet)\n";
                return 1;
            }
        }
        else
        {
            std::cerr << "usage: " << argv[0] << " [--model yolo11n|yolov8n|efficientdet|ssd|nanodet]\n";
            return 1;
        }
    }
    const std::string modelPath = std::string(imx500::MODEL_DIR) + model->file;
    std::cerr << "model: " << model->name << " (" << modelPath << ")\n";

    // Register the signal handler for interrupt (SIGINT) and termination (SIGTERM) signals
    std::signal(SIGINT, imx500::signalHandler);
    std::signal(SIGTERM, imx500::signalHandler);

    // Load the network on the sensor (before libcamera starts, like picamera2)
    std::unique_ptr<imx500::NetworkLoader> net;
    try
    {
        net = std::make_unique<imx500::NetworkLoader>();
        net->loadNetwork(modelPath);
        net->setInferenceRoi(0, 0, static_cast<unsigned>(imx500::SENSOR_W), static_cast<unsigned>(imx500::SENSOR_H));
        std::cerr << "network loaded via " << net->devicePath() << "\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }


    // Per-frame CSV log (one file per run in logs/)
    try
    {
        logger = std::make_unique<imx500::FrameLogger>(model->name);
        std::cerr << "log: " << logger->path() << "\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << "\n";
        return 1;
    }


    // Central libcamera manager. Also starts an internal background thread to handle hardware events.
    CameraManager cm;
    if (cm.start() != 0 || cm.cameras().empty())
    {
        std::cerr << "Errore: nessuna camera rilevata da libcamera.\n";
        return 1;
    }

    // Search for and acquire exclusive access to the IMX500 camera
    for (const auto &cam : cm.cameras())
    {
        if (cam->id().find("imx500") != std::string::npos)
        {
            camera = cam;
            break;
        }
    }
    if (!camera)
    {
        camera = cm.cameras()[0];
    }

    if (camera->acquire() != 0)
    {
        std::cerr << "Errore: impossibile acquisire la camera.\n";
        return 1;
    }

    // Configure the video stream (640x480)
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({StreamRole::Viewfinder});
    config->at(0).size = {imx500::W, imx500::H};
    config->validate();

    if (camera->configure(config.get()) != 0)
    {
        std::cerr << "Errore nella configurazione dello stream.\n";
        camera->release();
        return 1;
    }

    // Buffer allocation and Request creation
    // Logical pointer to the configured video stream
    Stream *stream = config->at(0).stream();
    // Buffer allocator for the video stream
    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(stream) < 0)
    {
        std::cerr << "Errore nell'allocazione dei buffer.\n";
        camera->release();
        return 1;
    }
    // Represents a single capture: holds one or more FrameBuffers to fill
    // and, once completed, will contain the metadata ControlList
    std::vector<std::unique_ptr<Request>> requests;
    for (const auto &buffer : allocator.buffers(stream))
    {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request || request->addBuffer(stream, buffer.get()) < 0)
        {
            std::cerr << "Errore nella creazione della Request.\n";
            return 1;
        }
        requests.push_back(std::move(request));
    }

    // Connect the camera's request-completed signal to the callback function that processes metadata
    camera->requestCompleted.connect(imx500::onRequestCompleted);

    // Start video capture
    if (camera->start() != 0)
    {
        std::cerr << "Errore nell'avvio della camera.\n";
        return 1;
    }

    // Queue requests for processing
    for (auto &request : requests)
    {
        camera->queueRequest(request.get());
    }

    // Main thread has finished setup; wait for Ctrl+C to terminate acquisition
    // The network is uploaded to the sensor when the stream starts: announce it at the first CNN output
    bool readyShown = false;
    while (running)
    {
        if (networkReady && !readyShown)
        {
            std::cerr << "Rete caricata\n";
            readyShown = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop camera and release all resources
    camera->stop();
    logger.reset();
    allocator.free(stream);
    camera->release();
    camera.reset();
    cm.stop();

    return 0;
}
