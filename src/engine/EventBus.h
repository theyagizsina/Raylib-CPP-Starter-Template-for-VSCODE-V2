#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace engine {

class EventBus {
public:
    using ListenerId = std::uint64_t;

    struct ListenerHandle {
        ListenerHandle() : type(typeid(void)), id(0) {}
        ListenerHandle(std::type_index typeIndex, ListenerId listenerId)
            : type(typeIndex), id(listenerId) {}

        std::type_index type;
        ListenerId id;

        [[nodiscard]] bool valid() const { return id != 0; }
    };

    EventBus();

    void clear();

    template <typename EventT, typename Callable>
    ListenerHandle subscribe(Callable&& callback)
    {
        const std::type_index type = std::type_index(typeid(EventT));
        auto wrapped = std::function<void(const void*)>(
            [func = std::function<void(const EventT&)>(std::forward<Callable>(callback))](const void* data) {
                func(*static_cast<const EventT*>(data));
            });

        const ListenerId listenerId = nextId++;
        listeners[type].emplace_back(listenerId, std::move(wrapped));
        return ListenerHandle{type, listenerId};
    }

    void unsubscribe(const ListenerHandle& handle);

    template <typename EventT>
    void publish(EventT event)
    {
        auto payload = std::make_shared<EventT>(std::move(event));
        queuedEvents.push_back(QueuedEvent{
            std::type_index(typeid(EventT)),
            [this, payload]() {
                dispatchImmediate(std::type_index(typeid(EventT)), payload.get());
            }
        });
    }

    void pump();

private:
    using ListenerList = std::vector<std::pair<ListenerId, std::function<void(const void*)>>>;

    struct QueuedEvent {
        std::type_index type;
        std::function<void()> dispatcher;
    };

    std::unordered_map<std::type_index, ListenerList> listeners;
    std::vector<QueuedEvent> queuedEvents;
    ListenerId nextId;

    void dispatchImmediate(const std::type_index& type, const void* eventData);
};

} // namespace engine
