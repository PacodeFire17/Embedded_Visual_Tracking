#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "detection_imx500.hpp"

/**
 * @file frame_logger.hpp
 * @brief Per-frame CSV log of the detections, for the measurements done afterwards on the PC.
 */

namespace imx500
{
    /**
     * @brief Current time in nanoseconds on CLOCK_BOOTTIME.
     *
     * It is the clock of libcamera's SensorTimestamp, so the two can be subtracted.
     */
    std::int64_t nowBoottimeNs();

    /**
     * @brief Writes one CSV row per detection, or one row with empty detection fields
     *        for a frame without detections.
     */
    class FrameLogger
    {
    public:
        /// Creates <directory>/<YYYYMMDD_HHMMSS>.csv and writes the header. Throws std::runtime_error on failure.
        explicit FrameLogger(const std::string &directory = "logs");
        ~FrameLogger();
        FrameLogger(const FrameLogger &) = delete;
        FrameLogger &operator=(const FrameLogger &) = delete;

        /**
         * @param frame      Progressive frame number.
         * @param tSensorNs  Capture time from the frame metadata (SensorTimestamp), CLOCK_BOOTTIME.
         * @param tProcNs    Time when processing of the frame ended, CLOCK_BOOTTIME.
         * @param parseNs    Time spent in parseDetections on the Pi, ns.
         * @param hasTensor  True if the frame carried the CNN output.
         * @param dnnUs      Network run time on the sensor (CnnKpiInfo[0], µs), -1 if not available.
         * @param dspUs      DSP run time on the sensor (CnnKpiInfo[1], µs), -1 if not available.
         * @param detections Detections of the frame (may be empty).
         */
                void logFrame(std::uint64_t frame, std::int64_t tSensorNs, std::int64_t tProcNs, std::int64_t parseNs,
                      bool hasTensor, std::int32_t dnnUs, std::int32_t dspUs,
                      const std::vector<Detection> &detections);

        /// Path of the CSV file being written.
        const std::string &path() const { return path_; }

    private:
        std::FILE *file_ = nullptr; ///< Open CSV file.
        std::string path_;          ///< Its path.
    };
} // namespace imx500
