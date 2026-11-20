#pragma once

#include <cstdint>
#include <cstdio>

inline std::uint64_t readBoardTicks() noexcept
{
    std::uint64_t ticks = 0;

    // Read the ARMv8 system counter (CNTVCT_EL0) to get the current tick count.
    asm volatile("mrs %0, cntvct_el0" : "=r"(ticks));
    return ticks;
}

inline std::uint64_t readBoardFrequency() noexcept
{
    std::uint64_t freq = 0;

    // Read the ARMv8 system counter frequency (CNTFRQ_EL0) to get the ticks per second.
    asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
    return freq;
}

inline void logFrameTime(std::uint32_t printEvery = 1) noexcept
{
    // False on first call, true on subsequent calls
    static bool initialized = false;
    // Static variables to track previous tick count and call count for frame timing
    static std::uint64_t prevTicks = 0;
    // Static variable to count the interval between frame have been printed
    static std::uint32_t callCount = 0;
    // Constant value to convert ticks to milliseconds based on the board frequency
    static const double ticksToMs = 1000.0 / static_cast<double>(readBoardFrequency());
    // Tick count at the current call to logFrameTime
    const std::uint64_t now = readBoardTicks();

    // Increment the call count and check if it's time to print the frame time
    if (!initialized)
    {
        prevTicks = now;
        initialized = true;
        return;
    }

    const double elapsedMs = static_cast<double>(now - prevTicks) * ticksToMs;
    // Resetting the previous tick count to the current tick count for the next call
    prevTicks = now;

    if (printEvery > 0 && (++callCount % printEvery == 0))
    {
        // Calculate frames per second (FPS) based on the elapsed time
        const double fps = (elapsedMs > 0.0) ? (1000.0 / elapsedMs) : 0.0;
        // Print the elapsed time and FPS to the console with formatting
        std::printf("[Pi5 Timer] Elapsed Time: %.3f ms (%.1f FPS)\n", elapsedMs, fps);
    }
}
