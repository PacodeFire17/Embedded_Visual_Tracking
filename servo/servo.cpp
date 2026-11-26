/**
 * @file servo.cpp
 * @brief Implementation of ServoAxis (sysfs hardware PWM).
 */

#include "servo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>

namespace
{
    /// PWM period for hobby servos: 20 ms (50 Hz).
    constexpr long PERIOD_NS = 20'000'000;

    /// Writes one value to a sysfs file; throws if it fails.
    void writeSysfs(const std::string &path, long value)
    {
        std::ofstream f(path);
        if (!f)
        {
            throw std::runtime_error("cannot open " + path + " (run with sudo?)");
        }
        f << value;
        f.flush();
        if (!f)
        {
            throw std::runtime_error("cannot write " + path);
        }
    }
} // namespace

namespace tracking
{
    ServoAxis::ServoAxis(const ServoConfig &cfg) : cfg_(cfg), us_(cfg.centerUs)
    {
        if (cfg_.minUs >= cfg_.maxUs || cfg_.centerUs < cfg_.minUs || cfg_.centerUs > cfg_.maxUs ||
            cfg_.maxStepUs <= 0)
        {
            throw std::invalid_argument("invalid servo limits");
        }

        const std::string chip = "/sys/class/pwm/pwmchip" + std::to_string(cfg_.chip);
        dir_ = chip + "/pwm" + std::to_string(cfg_.channel);

        // Export the channel if needed; the kernel creates its files shortly afterwards
        if (!std::filesystem::exists(dir_))
        {
            writeSysfs(chip + "/export", cfg_.channel);
            for (int i = 0; i < 20 && !std::filesystem::exists(dir_ + "/period"); ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        // Period first: the kernel refuses a duty cycle longer than the period
        writeSysfs(dir_ + "/period", PERIOD_NS);
        writeUs(us_);
        writeSysfs(dir_ + "/enable", 1);
    }

    ServoAxis::~ServoAxis()
    {
        try
        {
            writeSysfs(dir_ + "/enable", 0);
        }
        catch (...)
        {
            // Never throw from a destructor
        }
    }

    void ServoAxis::moveBy(float deltaUs)
    {
        // 1. Slew limit: never more than maxStepUs in a single command
        const float maxStep = static_cast<float>(cfg_.maxStepUs);
        float d = std::clamp(deltaUs, -maxStep, maxStep);
        // 2. From image direction to servo direction
        if (cfg_.invert)
        {
            d = -d;
        }
        // 3. Range limit from the calibration
        const int target = std::clamp(us_ + static_cast<int>(std::lround(d)), cfg_.minUs, cfg_.maxUs);
        if (target != us_)
        {
            writeUs(target);
        }
    }

    void ServoAxis::writeUs(int us)
    {
        us = std::clamp(us, cfg_.minUs, cfg_.maxUs);
        writeSysfs(dir_ + "/duty_cycle", static_cast<long>(us) * 1000); // µs -> ns
        us_ = us;
    }
} // namespace tracking
