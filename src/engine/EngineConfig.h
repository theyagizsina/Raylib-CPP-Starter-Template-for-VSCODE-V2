#pragma once

#include <cstdint>

namespace engine {

struct EngineConfig {
    double fixedTimeStep = 1.0 / 60.0;          // seconds
    std::uint32_t maxFixedStepsPerFrame = 5;    // safety valve for catch-up
    double maxDeltaTime = 0.25;                 // clamp to avoid huge jumps (seconds)
};

} // namespace engine
