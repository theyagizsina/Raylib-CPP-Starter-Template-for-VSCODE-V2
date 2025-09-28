#include "EventBus.h"

#include <algorithm>

namespace engine {

EventBus::EventBus()
    : nextId(1)
{
    queuedEvents.reserve(32);
}

void EventBus::clear()
{
    listeners.clear();
    queuedEvents.clear();
    nextId = 1;
}

void EventBus::unsubscribe(const ListenerHandle& handle)
{
    if (!handle.valid()) {
        return;
    }

    auto it = listeners.find(handle.type);
    if (it == listeners.end()) {
        return;
    }

    auto& list = it->second;
    list.erase(std::remove_if(list.begin(), list.end(), [&](const auto& pair) {
                    return pair.first == handle.id;
                }),
                list.end());

    if (list.empty()) {
        listeners.erase(it);
    }
}

void EventBus::pump()
{
    for (std::size_t idx = 0; idx < queuedEvents.size(); ++idx) {
        queuedEvents[idx].dispatcher();
    }
    queuedEvents.clear();
}

void EventBus::dispatchImmediate(const std::type_index& type, const void* eventData)
{
    auto it = listeners.find(type);
    if (it == listeners.end()) {
        return;
    }

    auto& list = it->second;
    for (const auto& entry : list) {
        entry.second(eventData);
    }
}

} // namespace engine
