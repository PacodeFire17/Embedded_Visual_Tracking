#pragma once

#include <cmath>
#include <optional>
#include <vector>

#include "detection_imx500.hpp"

/**
 * @file tracking.hpp
 * @brief Target selection and proportional controller (pure logic, no hardware).
 */

namespace tracking
{
    /// COCO-80 class of the target.
    constexpr int TARGET_CLASS = 0;

    /// Returns the detection of class cls with the highest score, if any.
    inline std::optional<imx500::Detection> selectTarget(const std::vector<imx500::Detection> &dets,
                                                         int cls = TARGET_CLASS)
    {
        std::optional<imx500::Detection> best;
        for (const auto &d : dets)
        {
            if (d.cls == cls && (!best || d.score > best->score))
            {
                best = d;
            }
        }
        return best;
    }

    /// Parameters of the proportional controller of one axis.
    struct PConfig
    {
        float kpUsPerPx;  ///< Correction per pixel of error, µs/px.
        float deadbandPx; ///< No correction when |error| is at most this, px.
    };

    /**
     * @brief Error from the image centre (px) -> servo correction (µs, image direction).
     *
     * Positive error (target right of / below the centre) gives a positive correction,
     * i.e. "move the view towards the target". ServoAxis applies invert and the limits.
     */
    inline float pCorrection(float errPx, const PConfig &c)
    {
        if (std::fabs(errPx) <= c.deadbandPx)
        {
            return 0.0f;
        }
        return c.kpUsPerPx * errPx;
    }
} // namespace tracking
