#pragma once

#include <string>

namespace engine {

class Engine;
struct EngineTime;
class EventBus;

struct EngineContext {
    Engine& engine;
    EventBus& eventBus;
};

class ISubsystem {
public:
    virtual ~ISubsystem() = default;

    virtual const char* getName() const = 0;

    virtual bool onInitialize(EngineContext& /*context*/) { return true; }
    virtual void onStart(EngineContext& /*context*/) {}
    virtual void onShutdown(EngineContext& /*context*/) {}
    virtual void onUpdate(EngineContext& /*context*/, double /*deltaTime*/) {}
    virtual void onFixedUpdate(EngineContext& /*context*/, double /*fixedDeltaTime*/) {}
};

} // namespace engine
