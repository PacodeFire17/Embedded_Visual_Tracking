#pragma once

#include <string>

/**
 * @file servo.hpp
 * @brief One servo axis driven by the Pi 5 hardware PWM through sysfs (/sys/class/pwm).
 *
 * Every command to the servos goes through this class, which enforces the calibrated limits
 * and a maximum step per command. The PWM is disabled when the object is destroyed.
 */

namespace tracking
{
    /// Calibration and safety parameters of one axis.
    struct ServoConfig
    {
        int chip;      ///< N in /sys/class/pwm/pwmchipN.
        int channel;   ///< PWM channel (GPIO 12 -> 0, GPIO 13 -> 1 with dtoverlay=pwm-2chan).
        int minUs;     ///< Lowest pulse width allowed, µs.
        int maxUs;     ///< Highest pulse width allowed, µs.
        int centerUs;  ///< Pulse width with the camera looking straight ahead, µs.
        bool invert;   ///< True if more µs moves the view towards -x (pan) or -y (tilt).
        int maxStepUs; ///< Largest change allowed in a single command, µs (slew limit).
    };

    class ServoAxis
    {
    public:
        /// Exports the PWM channel, sets 50 Hz, moves to centerUs and enables the output.
        /// Throws std::runtime_error if sysfs cannot be written (run with sudo).
        explicit ServoAxis(const ServoConfig &cfg);
        /// Disables the PWM output (the servo stops holding its position).
        ~ServoAxis();
        ServoAxis(const ServoAxis &) = delete;
        ServoAxis &operator=(const ServoAxis &) = delete;

        /**
         * @brief Moves the axis by deltaUs in IMAGE direction: > 0 means "move the view
         *        towards +x (right)" for pan and "towards +y (down)" for tilt.
         *        invert, the slew limit and the min/max clamp are applied here.
         */
        void moveBy(float deltaUs);

        /// Current pulse width, µs.
        int positionUs() const { return us_; }
        /// True if the axis sits on minUs or maxUs.
        bool atLimit() const { return us_ == cfg_.minUs || us_ == cfg_.maxUs; }

    private:
        void writeUs(int us);

        ServoConfig cfg_;
        std::string dir_; ///< /sys/class/pwm/pwmchipN/pwmM
        int us_;          ///< Last pulse width written, µs.
    };
} // namespace tracking
