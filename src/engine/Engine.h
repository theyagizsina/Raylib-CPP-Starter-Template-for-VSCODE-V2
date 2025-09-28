#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "EngineConfig.h"
#include "EngineTime.h"
#include "EventBus.h"
#include "Subsystem.h"

namespace engine {

class Engine {
public:
    Engine();

    void setConfig(const EngineConfig& config);
    const EngineConfig& getConfig() const { return config; }

    bool initialize();
    void start();
    void shutdown();

    void registerSubsystem(std::unique_ptr<ISubsystem> subsystem);

    void tick(double overrideDeltaTime = -1.0);

    const EngineTime& getTime() const { return time; }
    EventBus& getEventBus() { return eventBus; }
    const EventBus& getEventBus() const { return eventBus; }

    void synchronizeClock();

private:
    using Clock = std::chrono::steady_clock;

    EngineConfig config;
    EngineTime time;
    EventBus eventBus;
    std::vector<std::unique_ptr<ISubsystem>> subsystems;
    EngineContext context;

    Clock::time_point lastTickTime;
    bool hasLastTick;
    double accumulator;
    bool initialized;
    bool started;

    double computeDeltaTime(double overrideDeltaTime);
};

} // namespace engine
