/**
 * @file detection_imx500.cpp
 * @brief Implementation of IMX500 AI camera detection pipeline and acquisition loop.
 */

#include "detection_imx500.hpp"
#include "imx500_network.hpp"

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
    /// Flag indicating whether image acquisition should continue.
    std::atomic<bool> running{true};
    /// Becomes true at the first frame carrying the CNN output: the network is running on the sensor.
    std::atomic<bool> networkReady{false};
    /// Shared smart pointer to the Camera object representing the physical IMX500 sensor.
    std::shared_ptr<Camera> camera;
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
     * @return Vector of parsed Detection objects.
     */
    std::vector<Detection> parseDetections(const ControlList &metadata)
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

            // 5: Dual coordinate transformation (Network -> Sensor -> Video)

            // 5.1 Transforming pixel coordinates to normalized coordinates in the range [0, 1]
            // The network outputs each box as x0, y0, x1, y1 in pixels (0..640), so we divide by Input_W.
            // Order and range verified on the Pi with trial.py on 17/11/2026.
            // The tensor is stored column by column (Fortran order, like picamera2's reshape(order='F')):
            // first all x0, then all y0, then all x1, then all y1.
            float x0 = boxes[0 * numDet + i] / Input_W;
            float y0 = boxes[1 * numDet + i] / Input_W;
            float x1 = boxes[2 * numDet + i] / Input_W;
            float y1 = boxes[3 * numDet + i] / Input_W;

            // 5.2 Geometric remapping: from network space (1:1) to sensor space (4:3)
            // The network input region is the whole sensor (set in main with setInferenceRoi),
            // squashed to 640 × 640: x is scaled by 4056 (SENSOR_W) and y by 3040 (SENSOR_H), with no offset.
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
                static_cast<int>(classes[i]),
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

        // The first output tensor means the network upload to the sensor is complete
        if (!networkReady && request->metadata().get(controls::rpi::CnnOutputTensor))
        {
            networkReady = true;
        }

        // Process metadata and print detections
        for (const auto &det : parseDetections(request->metadata()))
        {
            std::printf("classe %2d  score %.2f  "
                        "box x=%.0f y=%.0f w=%.0f h=%.0f  "
                        "centro=(%.0f,%.0f)  errore=(%+.0f,%+.0f) px\n",
                        det.cls, det.score, det.x, det.y, det.w, det.h,
                        det.cx, det.cy, det.errX, det.errY);
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

int main()
{
    // Register the signal handler for interrupt (SIGINT) and termination (SIGTERM) signals
    std::signal(SIGINT, imx500::signalHandler);
    std::signal(SIGTERM, imx500::signalHandler);

    // Load the network on the sensor (before libcamera starts, like picamera2)
    std::unique_ptr<imx500::NetworkLoader> net;
    try
    {
        net = std::make_unique<imx500::NetworkLoader>();
        net->loadNetwork(imx500::MODEL_PATH);
        net->setInferenceRoi(0, 0, static_cast<unsigned>(imx500::SENSOR_W), static_cast<unsigned>(imx500::SENSOR_H));
        std::cerr << "network loaded via " << net->devicePath() << "\n";
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
    allocator.free(stream);
    camera->release();
    camera.reset();
    cm.stop();

    return 0;
}
