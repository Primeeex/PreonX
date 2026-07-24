#pragma once

#include "primeon/world/rigid_body.hpp"

namespace primeon::math {

// ── World Callbacks ───────────────────────────────────────────────────────
//
// Callback interfaces for simulation events.
// Users register callbacks; the world calls them at the appropriate time.
// Callbacks are simple function pointers for maximum flexibility.

/// Contact event types.
enum class ContactEventType : u32 {
    Begin   = 0,  ///< First frame of contact between two bodies
    Persist = 1,  ///< Continued contact
    End     = 2,  ///< Contact broken
};

/// Data for a contact event.
struct ContactEvent {
    ContactEventType type;
    u32 bodyIDA;
    u32 bodyIDB;
    Vector3 contactPoint;
    Vector3 contactNormal;
    f32 penetration;
};

/// Function signatures for world callbacks.
using BodyCreatedCallback    = void(*)(u32 bodyID, void* userData);
using BodyDestroyedCallback  = void(*)(u32 bodyID, void* userData);
using ContactCallback        = void(*)(const ContactEvent& event, void* userData);

/// Set of callbacks for world events.
struct WorldCallbacks {
    BodyCreatedCallback onBodyCreated   = nullptr;
    BodyDestroyedCallback onBodyDestroyed = nullptr;
    ContactCallback onContactBegin      = nullptr;
    ContactCallback onContactPersist    = nullptr;
    ContactCallback onContactEnd        = nullptr;

    void* userData = nullptr;

    constexpr WorldCallbacks() noexcept = default;
};

} // namespace primeon::math
