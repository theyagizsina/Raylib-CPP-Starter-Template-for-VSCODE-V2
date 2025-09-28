#pragma once

#include <cstdint>

namespace engine {

struct FrameDiagnosticsEvent {
    double deltaTime = 0.0;           // Variable frame delta time in seconds
    double fixedTimeStep = 0.0;       // Configured fixed timestep in seconds
    std::uint32_t fixedStepCount = 0; // Number of fixed updates processed this frame
    double totalTime = 0.0;           // Accumulated engine runtime in seconds
    std::uint64_t frameIndex = 0;     // Index of the current frame
    double accumulator = 0.0;         // Remaining accumulated time after fixed steps
};

} // namespace engine
