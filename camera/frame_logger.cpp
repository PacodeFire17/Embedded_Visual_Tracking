#include "frame_logger.hpp"

#include <cinttypes>
#include <ctime>
#include <filesystem>
#include <stdexcept>

#include <time.h>

namespace imx500
{
    std::int64_t nowBoottimeNs()
    {
        timespec ts{};
        clock_gettime(CLOCK_BOOTTIME, &ts);
        return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
    }

    FrameLogger::FrameLogger(const std::string &tag, const std::string &directory)
    {
        // One file per run, named after the start date and time and the tag (model name)
        std::filesystem::create_directories(directory);
        char name[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(name, sizeof(name), "%Y%m%d_%H%M%S", std::localtime(&now));
        path_ = directory + "/" + name + "_" + tag + ".csv";

        file_ = std::fopen(path_.c_str(), "w");
        if (!file_)
        {
            throw std::runtime_error("cannot create " + path_);
        }
        std::fprintf(file_, "frame,t_sensor_ns,t_proc_ns,parse_ns,has_tensor,dnn_us,dsp_us,cls,score,x,y,w,h,err_x,err_y\n");
    }

    FrameLogger::~FrameLogger()
    {
        // fclose also writes out what is still in the buffer
        if (file_)
        {
            std::fclose(file_);
        }
    }

    void FrameLogger::logFrame(std::uint64_t frame, std::int64_t tSensorNs, std::int64_t tProcNs, std::int64_t parseNs,
                      bool hasTensor, std::int32_t dnnUs, std::int32_t dspUs,
                      const std::vector<Detection> &detections)
    {
        if (detections.empty())
        {
            // Frame without detections: keep the timing, leave the detection fields empty
            std::fprintf(file_, "%" PRIu64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%d,%d,%d,,,,,,,,\n",
                         frame, tSensorNs, tProcNs, parseNs, hasTensor ? 1 : 0, dnnUs, dspUs);
            return;
        }
        for (const Detection &d : detections)
        {
            std::fprintf(file_, "%" PRIu64 ",%" PRId64 ",%" PRId64 ",%" PRId64 ",%d,%d,%d,%d,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                     frame, tSensorNs, tProcNs, parseNs, hasTensor ? 1 : 0, dnnUs, dspUs,
                     d.cls, d.score, d.x, d.y, d.w, d.h, d.errX, d.errY);
        }
    }
} // namespace imx500
