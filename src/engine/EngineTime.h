#pragma once

#include <cstdint>

namespace engine {

struct EngineTime {
	double deltaTime = 0.0;           // Last frame's delta time
	double fixedTimeStep = 1.0 / 60.0; // Fixed time step used for deterministic updates
	double totalTime = 0.0;           // Accumulated runtime in seconds
	std::uint64_t frameIndex = 0;     // Number of frames processed
	std::uint32_t fixedStepCount = 0; // Fixed updates executed during the last frame
};

} // namespace engine
