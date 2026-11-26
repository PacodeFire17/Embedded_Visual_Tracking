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
    /// COCO-80 classes that can be the target: 15 = cat, 16 = dog, 77 = teddy bear.
    inline constexpr int TARGET_CLASSES[] = {15, 16, 77};

    /// While locked, the target centre may move at most this much between two network results, px.
    constexpr float MAX_JUMP_PX = 150.0f;

    /// Network results in a row without the target after which the lock is released.
    constexpr int LOST_RESULTS = 10;

    /// True if cls is one of TARGET_CLASSES.
    constexpr bool isTargetClass(int cls)
    {
        for (int c : TARGET_CLASSES)
        {
            if (c == cls)
            {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Chooses the target among the detections of one network result.
     *
     * Only boxes of a class in TARGET_CLASSES are considered.
     * - No lastTarget (not locked): the box with the highest score.
     * - lastTarget (locked): the box whose centre is nearest to lastTarget's centre,
     *   but only within MAX_JUMP_PX; a farther box is another object, so none is returned.
     */
    inline std::optional<imx500::Detection> selectTarget(const std::vector<imx500::Detection> &dets,
                                                         const std::optional<imx500::Detection> &lastTarget)
    {
        std::optional<imx500::Detection> best;
        float bestDist = MAX_JUMP_PX;
        for (const auto &d : dets)
        {
            if (!isTargetClass(d.cls))
            {
                continue;
            }
            if (lastTarget)
            {
                const float dist = std::hypot(d.cx - lastTarget->cx, d.cy - lastTarget->cy);
                if (dist <= bestDist)
                {
                    best = d;
                    bestDist = dist;
                }
            }
            else if (!best || d.score > best->score)
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
