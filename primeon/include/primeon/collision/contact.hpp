#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"

namespace primeon::math {

/// Maximum contacts per manifold (for solver stability).
inline constexpr u32 kMaxContactPoints = 4;

// ── Feature Types ──────────────────────────────────────────────────────────

/// Geometric feature type for contact identification.
enum class FeatureType : u32 {
    None  = 0,
    Face  = 1,
    Edge  = 2,
    Vertex = 3,
};

// ── Contact Feature ────────────────────────────────────────────────────────

/// Identifies the geometric feature on a shape that produced a contact.
/// Used for contact persistence: matching features across frames.
struct ContactFeature {
    FeatureType type = FeatureType::None;
    u32 indexA = 0;  // feature index on shape A
    u32 indexB = 0;  // feature index on shape B

    constexpr ContactFeature() noexcept = default;
    constexpr ContactFeature(FeatureType type, u32 indexA, u32 indexB) noexcept
        : type(type), indexA(indexA), indexB(indexB) {}

    [[nodiscard]] constexpr bool operator==(const ContactFeature& o) const noexcept {
        return type == o.type && indexA == o.indexA && indexB == o.indexB;
    }
    [[nodiscard]] constexpr bool operator!=(const ContactFeature& o) const noexcept {
        return !(*this == o);
    }
};

/// Generates a stable contact ID from two features.
/// The ID is order-independent: ID(a,b) == ID(b,a).
[[nodiscard]] inline u32 makeContactID(const ContactFeature& a, const ContactFeature& b) noexcept {
    // Combine feature hashes. Order-independent by XORing with sorted indices.
    u32 ta = static_cast<u32>(a.type);
    u32 tb = static_cast<u32>(b.type);
    u32 ia = a.indexA * 73856093u ^ a.indexB * 19349663u;
    u32 ib = b.indexA * 73856093u ^ b.indexB * 19349663u;
    if (ia > ib) { std::swap(ia, ib); std::swap(ta, tb); }
    return ia ^ ib ^ (ta * 83492791u) ^ (tb * 34219801u);
}

// ── Contact Point ──────────────────────────────────────────────────────────

/// A single contact point between two colliding shapes.
/// All quantities are in world space.
struct ContactPoint {
    Vector3 point = kVector3Zero;
    f32 penetration = 0.0f;

    // Feature identification for persistence
    ContactFeature featureA;
    ContactFeature featureB;
    u32 contactID = 0;

    // Accumulated impulses (reserved for solver; zero-initialized)
    f32 normalImpulse = 0.0f;
    f32 frictionImpulseU = 0.0f;
    f32 frictionImpulseV = 0.0f;

    // Tangent basis for friction (computed when contact is created)
    Vector3 tangentU = kVector3Zero;
    Vector3 tangentV = kVector3Zero;

    /// Compute contactID from features (call after setting features).
    void computeID() noexcept {
        contactID = makeContactID(featureA, featureB);
    }

    [[nodiscard]] constexpr bool hasSameID(const ContactPoint& other) const noexcept {
        return contactID == other.contactID && contactID != 0;
    }
};

// ── Contact Manifold ───────────────────────────────────────────────────────

/// A set of contact points sharing a common separating normal.
/// The normal points from shape B toward shape A (i.e., the direction to push A out of B).
struct ContactManifold {
    Vector3 normal = kVector3Zero;
    ContactPoint contacts[kMaxContactPoints] = {};
    u32 contactCount = 0;

    // Reference/incident shape identification
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;

    constexpr ContactManifold() noexcept = default;

    /// Adds a contact point. Returns false if manifold is full.
    constexpr bool addContact(const ContactPoint& cp) noexcept {
        if (contactCount >= kMaxContactPoints) return false;
        contacts[contactCount++] = cp;
        return true;
    }

    /// Removes a contact point at the given index by shifting.
    constexpr void removeContact(u32 index) noexcept {
        if (index >= contactCount) return;
        for (u32 i = index; i + 1 < contactCount; ++i)
            contacts[i] = contacts[i + 1];
        --contactCount;
    }

    /// Clears all contacts but preserves normal and body IDs.
    constexpr void clear() noexcept { contactCount = 0; }

    [[nodiscard]] constexpr bool empty() const noexcept { return contactCount == 0; }
    [[nodiscard]] constexpr bool full() const noexcept { return contactCount >= kMaxContactPoints; }

    [[nodiscard]] constexpr f32 maxPenetration() const noexcept {
        f32 maxP = -std::numeric_limits<f32>::max();
        for (u32 i = 0; i < contactCount; ++i)
            maxP = max(maxP, contacts[i].penetration);
        return maxP;
    }

    /// Returns the index of the deepest contact, or contactCount if empty.
    [[nodiscard]] constexpr u32 deepestIndex() const noexcept {
        u32 best = 0;
        f32 bestDepth = -std::numeric_limits<f32>::max();
        for (u32 i = 0; i < contactCount; ++i) {
            if (contacts[i].penetration > bestDepth) {
                bestDepth = contacts[i].penetration;
                best = i;
            }
        }
        return best;
    }

    /// Compute tangent basis vectors from the contact normal.
    void computeTangents() noexcept {
        Vector3 tangent;
        if (abs(normal.x) < 0.9f) {
            tangent = normal.cross(kVector3UnitX).normalized();
        } else {
            tangent = normal.cross(kVector3UnitY).normalized();
        }
        Vector3 bitangent = normal.cross(tangent);
        for (u32 i = 0; i < contactCount; ++i) {
            contacts[i].tangentU = tangent;
            contacts[i].tangentV = bitangent;
        }
    }
};

// ── Collision Result ───────────────────────────────────────────────────────

/// Complete result of a narrowphase collision test.
struct CollisionResult {
    bool colliding = false;
    ContactManifold manifold;
};

// ── Contact Pair ───────────────────────────────────────────────────────────

/// A pair of colliding bodies with their contact manifold.
struct ContactPair {
    u32 bodyIDA = 0;
    u32 bodyIDB = 0;
    ContactManifold manifold;

    constexpr ContactPair() noexcept = default;
    constexpr ContactPair(u32 idA, u32 idB, const ContactManifold& m) noexcept
        : bodyIDA(idA), bodyIDB(idB), manifold(m) {}

    [[nodiscard]] constexpr bool matches(u32 a, u32 b) const noexcept {
        return (bodyIDA == a && bodyIDB == b) || (bodyIDA == b && bodyIDB == a);
    }
};

// ── Ray Hit ────────────────────────────────────────────────────────────────

/// Result of a raycast query.
struct RayHit {
    bool hit = false;
    f32 distance = std::numeric_limits<f32>::max();
    Vector3 point = kVector3Zero;
    Vector3 normal = kVector3Zero;
};

} // namespace primeon::math
