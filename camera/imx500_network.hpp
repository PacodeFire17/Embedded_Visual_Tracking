#pragma once

#include <string>

namespace imx500
{
    /**
     * @brief Handle to the IMX500 V4L2 sub-device, used to load the network on the sensor.
     *
     * Create it BEFORE the libcamera CameraManager starts (as picamera2 does) and keep it
     * alive until the camera is released.
     */
    class NetworkLoader
    {
    public:
        /// Finds /dev/v4l-subdevN of the IMX500 and opens it. Throws std::runtime_error on failure.
        NetworkLoader();
        ~NetworkLoader();
        NetworkLoader(const NetworkLoader &) = delete;
        NetworkLoader &operator=(const NetworkLoader &) = delete;

        /// Gives the .rpk file to the driver; the upload to the sensor starts with the stream.
        void loadNetwork(const std::string &rpkPath);

        /// Sets the part of the sensor that feeds the network (full-sensor coordinates).
        void setInferenceRoi(unsigned x, unsigned y, unsigned w, unsigned h);

        const std::string &devicePath() const { return path_; }

    private:
        int fd_ = -1;
        std::string path_;
    };
} // namespace imx500
