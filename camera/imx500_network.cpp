#include "imx500_network.hpp"

// Implementation for the IMX500 Network Loader
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stdexcept>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace // Anonymous namespace for internal linkage 
{
    // Private V4L2 controls of the imx500 kernel driver (same values as picamera2)
    constexpr std::uint32_t ROI_CTRL_ID = 0x00982900; // The part of the sensor that feeds the network (full-sensor coordinates)
    constexpr std::uint32_t NETWORK_FW_FD_CTRL_ID = 0x00982901; // The .rpk file to be uploaded to the sensor (the upload starts with the stream)

    std::string errnoText() { return std::strerror(errno); } // Returns a human-readable string describing the last error number
} // namespace

namespace imx500
{
    NetworkLoader::NetworkLoader() 
    {
        // Same search as picamera2: the sub-device whose kernel module is "imx500"
        for (int i = 0; i < 32; ++i)
        {
            // Construct the path to the module
            const fs::path module = "/sys/class/video4linux/v4l-subdev" + std::to_string(i) + "/device/driver/module"; //costruct the path to the module
            std::error_code ec; // ec will be set if an error occurs, but we ignore it and continue searching
            if (!fs::is_symlink(module, ec)) // if the module path is not a symlink, it's not the right device
                continue;
            if (fs::read_symlink(module, ec).string().find("imx500") == std::string::npos) // if the symlink does not contain "imx500", it's not the right device
                continue;

            path_ = "/dev/v4l-subdev" + std::to_string(i); // found the right sub-device, construct its path
            fd_ = ::open(path_.c_str(), O_RDWR); // open the sub-device for reading and writing
            if (fd_ < 0)
                throw std::runtime_error("IMX500: cannot open " + path_ + ": " + errnoText());
            return;
        }
        throw std::runtime_error("IMX500: sub-device not found (is the AI camera connected?)");
    }

    NetworkLoader::~NetworkLoader()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    void NetworkLoader::loadNetwork(const std::string &rpkPath)
    {
        const int fileFd = ::open(rpkPath.c_str(), O_RDONLY);
        if (fileFd < 0)
            throw std::runtime_error("IMX500: cannot open " + rpkPath + ": " + errnoText());

        v4l2_control ctrl{};
        ctrl.id = NETWORK_FW_FD_CTRL_ID; // the .rpk file to be uploaded to the sensor (the upload starts with the stream)
        ctrl.value = fileFd; // the driver reads the firmware through this file descriptor
        const int ret = ::ioctl(fd_, VIDIOC_S_CTRL, &ctrl); // the driver reads the firmware through this file descriptor
        const std::string why = errnoText();
        ::close(fileFd); // picamera2 also closes it right after the ioctl
        if (ret < 0)
            throw std::runtime_error("IMX500: cannot set network " + rpkPath + ": " + why);
    }

    void NetworkLoader::setInferenceRoi(unsigned x, unsigned y, unsigned w, unsigned h)
    {
        std::uint32_t roi[4] = {x, y, w, h};

        v4l2_ext_control c{};
        c.id = ROI_CTRL_ID;
        c.size = sizeof(roi); // 16 bytes
        c.p_u32 = roi;

        v4l2_ext_controls ctrls{};
        ctrls.count = 1;
        ctrls.controls = &c;

        if (::ioctl(fd_, VIDIOC_S_EXT_CTRLS, &ctrls) < 0)
            throw std::runtime_error("IMX500: cannot set inference ROI: " + errnoText());
    }
} // namespace imx500
