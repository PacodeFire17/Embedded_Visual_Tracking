#pragma once

#include <cstdint>
#include <cstdio>

namespace imx500
{

    inline std::uint64_t readBoardTicks() noexcept
    {
        // Variable to store the current hardware tick count
        std::uint64_t ticks = 0;

        // Synchronize the instruction pipeline (ISB) and read the ARMv8 system counter (CNTVCT_EL0) to get the current tick count
        asm volatile("isb; mrs %0, cntvct_el0" : "=r"(ticks));
        return ticks;
    }

    inline std::uint64_t readBoardFreq() noexcept
    {
        // Variable to store the hardware timer frequency
        std::uint64_t freq = 0;

        // Read the ARMv8 system counter frequency (CNTFRQ_EL0) to get the ticks per second
        asm volatile("mrs %0, cntfrq_el0" : "=r"(freq));
        return freq;
    }

    namespace detail
    {
        // Shared inline variable to store the tick count recorded when frame processing starts
        inline std::uint64_t frameStartTicks = 0;
    } // namespace detail

    inline void startFrameTimer() noexcept
    {
        // Record the current tick count right before frame processing begins
        detail::frameStartTicks = readBoardTicks();
    }

    inline void endFrameTimer(std::uint32_t printEvery = 8) noexcept
    {
        // Read the current tick count immediately after frame processing finishes, before any calculation or printing
        const std::uint64_t endTicks = readBoardTicks();

        // Static variable to count how many frames have been measured
        static std::uint32_t callCount = 0;
        // Constant value to convert ticks to microseconds based on the board frequency
        static const double ticksToUs = 1'000'000.0 / static_cast<double>(readBoardFreq());

        // Calculate the elapsed processing time in microseconds for the current frame
        const double elapsedUs = static_cast<double>(endTicks - detail::frameStartTicks) * ticksToUs;

        // Increment the call count and check if it's time to print the processing time
        if (printEvery > 0 && (++callCount % printEvery == 0))
        {
            // Print the elapsed processing time in microseconds and milliseconds to the console with formatting
            std::printf("[Pi5 Timer] Elaborazione frame: %.2f us (%.4f ms)\n",
                        elapsedUs, elapsedUs / 1000.0);
        }
    }

} // namespace imx500