#pragma once

#include "cambyses/core/types.hpp"
#include "foundation/containers/dynamic_array.hpp"

#include <functional>
#include <new>
#include <typeinfo>
#include <utility>

namespace cambyses {

class EventDispatcher {
public:
    using SubscriptionId = foundation::u32;

    EventDispatcher() = default;

    ~EventDispatcher() {
        clear();
    }

    EventDispatcher(const EventDispatcher&) = delete;
    EventDispatcher& operator=(const EventDispatcher&) = delete;

    EventDispatcher(EventDispatcher&& other) noexcept
        : lists_(std::move(other.lists_)) {
        other.lists_.clear();
    }

    EventDispatcher& operator=(EventDispatcher&& other) noexcept {
        if (this != &other) {
            clear();
            lists_ = std::move(other.lists_);
            other.lists_.clear();
        }
        return *this;
    }

    template <typename Event>
    SubscriptionId subscribe(std::function<void(const Event&)> callback) {
        auto& list = get_or_create_list<Event>();
        SubscriptionId id = list.next_id++;
        list.entries.push_back({id, std::move(callback)});
        return id;
    }

    template <typename Event>
    void unsubscribe(SubscriptionId id) {
        auto* list = find_list<Event>();
        if (!list) {
            return;
        }
        for (foundation::size_t i = 0; i < list->entries.size(); ++i) {
            if (list->entries[i].id == id) {
                list->entries.erase(list->entries.begin() + static_cast<ptrdiff_t>(i));
                return;
            }
        }
    }

    template <typename Event>
    void publish(const Event& event) {
        auto* list = find_list<Event>();
        if (!list) {
            return;
        }
        list->invoke(&event);
    }

    void clear() {
        for (auto& [hash, list] : lists_) {
            delete list;
        }
        lists_.clear();
    }

private:
    struct CallbackListBase {
        virtual ~CallbackListBase() = default;
        virtual void invoke(const void* event) = 0;
    };

    template <typename Event>
    struct CallbackList : CallbackListBase {
        struct Entry {
            SubscriptionId id;
            std::function<void(const Event&)> callback;
        };
        foundation::DynamicArray<Entry> entries;
        SubscriptionId next_id = 1;

        void invoke(const void* event) override {
            const auto& e = *static_cast<const Event*>(event);
            for (auto& entry : entries) {
                entry.callback(e);
            }
        }
    };

    foundation::DynamicArray<std::pair<foundation::size_t, CallbackListBase*>> lists_;

    template <typename Event>
    CallbackList<Event>& get_or_create_list() {
        foundation::size_t hash = typeid(Event).hash_code();
        for (auto& [h, list] : lists_) {
            if (h == hash) {
                return *static_cast<CallbackList<Event>*>(list);
            }
        }
        auto* new_list = new CallbackList<Event>();
        lists_.emplace_back(hash, new_list);
        return *new_list;
    }

    template <typename Event>
    CallbackList<Event>* find_list() const {
        foundation::size_t hash = typeid(Event).hash_code();
        for (const auto& [h, list] : lists_) {
            if (h == hash) {
                return static_cast<CallbackList<Event>*>(list);
            }
        }
        return nullptr;
    }
};

} // namespace cambyses
