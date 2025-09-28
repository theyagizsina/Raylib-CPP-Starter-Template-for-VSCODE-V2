#include "Engine.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace engine {

Engine::Engine()
    : context{*this},
      hasLastTick(false),
      accumulator(0.0),
      initialized(false),
      started(false)
{
    time.fixedTimeStep = config.fixedTimeStep;
}

void Engine::setConfig(const EngineConfig& newConfig)
{
    config = newConfig;
    time.fixedTimeStep = config.fixedTimeStep;
}

bool Engine::initialize()
{
    if (initialized) {
        return true;
    }

    for (auto& subsystem : subsystems) {
        if (!subsystem->onInitialize(context)) {
            std::cerr << "Subsystem initialization failed: " << subsystem->getName() << std::endl;
            // Roll back previously initialized subsystems
            for (auto& initializedSubsystem : subsystems) {
                if (initializedSubsystem.get() == subsystem.get()) {
                    break;
                }
                initializedSubsystem->onShutdown(context);
            }
            return false;
        }
    }

    initialized = true;
    return true;
}

void Engine::start()
{
    if (!initialized || started) {
        return;
    }

    for (auto& subsystem : subsystems) {
        subsystem->onStart(context);
    }

    started = true;
    synchronizeClock();
}

void Engine::shutdown()
{
    if (!initialized) {
        return;
    }

    for (auto it = subsystems.rbegin(); it != subsystems.rend(); ++it) {
        (*it)->onShutdown(context);
    }

    subsystems.clear();
    initialized = false;
    started = false;
    hasLastTick = false;
    accumulator = 0.0;
    time = EngineTime{};
    time.fixedTimeStep = config.fixedTimeStep;
}

void Engine::registerSubsystem(std::unique_ptr<ISubsystem> subsystem)
{
    subsystems.emplace_back(std::move(subsystem));
}

void Engine::tick(double overrideDeltaTime)
{
    if (!started) {
        return;
    }

    double deltaTime = computeDeltaTime(overrideDeltaTime);
    accumulator += deltaTime;
    time.deltaTime = deltaTime;
    time.totalTime += deltaTime;
    time.frameIndex += 1;

    // Fixed step updates
    unsigned int steps = 0;
    while (accumulator + 1e-12 >= config.fixedTimeStep && steps < config.maxFixedStepsPerFrame) {
        for (auto& subsystem : subsystems) {
            subsystem->onFixedUpdate(context, config.fixedTimeStep);
        }
        accumulator -= config.fixedTimeStep;
        steps++;
    }

    // Prevent runaway accumulator when frame time spikes
    if (accumulator > config.fixedTimeStep * 2.0) {
        accumulator = std::fmod(accumulator, config.fixedTimeStep);
    }

    // Variable step updates
    for (auto& subsystem : subsystems) {
        subsystem->onUpdate(context, deltaTime);
    }
}

void Engine::synchronizeClock()
{
    hasLastTick = false;
    accumulator = 0.0;
}

double Engine::computeDeltaTime(double overrideDeltaTime)
{
    double deltaTime = overrideDeltaTime;
    auto now = Clock::now();

    if (deltaTime < 0.0) {
        if (!hasLastTick) {
            lastTickTime = now;
            hasLastTick = true;
            return 0.0;
        }

        deltaTime = std::chrono::duration<double>(now - lastTickTime).count();
        lastTickTime = now;
    } else {
        lastTickTime = now;
        hasLastTick = true;
    }

    if (config.maxDeltaTime > 0.0) {
        deltaTime = std::min(deltaTime, config.maxDeltaTime);
    }

    if (deltaTime < 0.0) {
        deltaTime = 0.0;
    }

    return deltaTime;
}

} // namespace engine
