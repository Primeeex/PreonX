#pragma once

#include "primeon/collision/contact.hpp"

namespace primeon::math {

// ── Contact Persistence ────────────────────────────────────────────────────
//
// Manifold updating matches new contacts against cached contacts by feature ID.
// Matched contacts inherit accumulated impulses for warm starting.
// Unmatched contacts start with zero impulses.
//
// The algorithm:
// 1. Mark all cached contacts as unmatched.
// 2. For each new contact, find a cached contact with the same ID.
// 3. If found, transfer cached impulses and update position/depth.
// 4. If not found, the contact is new (zero impulses).
// 5. Any unmatched cached contacts are removed.

/// Match tolerance: maximum distance for two contacts to be considered the same feature.
inline constexpr f32 kContactMatchDistance = 0.02f;

/// Matches a new contact against cached contacts by contact ID.
/// Returns true if a match was found, transferring cached impulses to the new contact.
[[nodiscard]] inline bool matchContact(ContactPoint& newContact,
                                        const ContactPoint* cached, u32 cachedCount) noexcept {
    if (newContact.contactID == 0) return false;

    for (u32 i = 0; i < cachedCount; ++i) {
        if (cached[i].contactID == newContact.contactID) {
            // Feature match found. Transfer accumulated impulses.
            newContact.normalImpulse = cached[i].normalImpulse;
            newContact.frictionImpulseU = cached[i].frictionImpulseU;
            newContact.frictionImpulseV = cached[i].frictionImpulseV;
            return true;
        }
    }
    return false;
}

/// Updates a cached manifold with new contact data.
/// Preserves impulse history for matched contacts; discards unmatched.
/// The output manifold replaces the cached one.
inline void updateManifold(ContactManifold& output,
                           const ContactManifold& cached,
                           const ContactManifold& fresh) noexcept {
    output.normal = fresh.normal;
    output.bodyIDA = fresh.bodyIDA;
    output.bodyIDB = fresh.bodyIDB;
    output.contactCount = 0;

    // Mark all cached contacts as unmatched
    bool matched[kMaxContactPoints] = {};
    for (u32 i = 0; i < cached.contactCount; ++i)
        matched[i] = false;

    // For each fresh contact, try to match against cached
    for (u32 f = 0; f < fresh.contactCount; ++f) {
        ContactPoint cp = fresh.contacts[f];

        for (u32 c = 0; c < cached.contactCount; ++c) {
            if (matched[c]) continue;
            if (cp.contactID == cached.contacts[c].contactID && cp.contactID != 0) {
                // Match found: transfer accumulated impulses
                cp.normalImpulse = cached.contacts[c].normalImpulse;
                cp.frictionImpulseU = cached.contacts[c].frictionImpulseU;
                cp.frictionImpulseV = cached.contacts[c].frictionImpulseV;
                matched[c] = true;
                break;
            }
        }

        output.addContact(cp);
    }

    output.computeTangents();
}

/// Removes contacts with very small penetration (degenerate contacts).
inline void pruneDegenerateContacts(ContactManifold& manifold,
                                     f32 minPenetration = -0.001f) noexcept {
    u32 writeIdx = 0;
    for (u32 i = 0; i < manifold.contactCount; ++i) {
        if (manifold.contacts[i].penetration > minPenetration) {
            if (writeIdx != i)
                manifold.contacts[writeIdx] = manifold.contacts[i];
            ++writeIdx;
        }
    }
    manifold.contactCount = writeIdx;
}

/// Removes redundant contacts that are too close together.
inline void removeRedundantContacts(ContactManifold& manifold,
                                     f32 minSpacing = 0.01f) noexcept {
    u32 writeIdx = 0;
    for (u32 i = 0; i < manifold.contactCount; ++i) {
        bool redundant = false;
        for (u32 j = 0; j < writeIdx; ++j) {
            f32 distSq = (manifold.contacts[i].point - manifold.contacts[j].point).lengthSq();
            if (distSq < minSpacing * minSpacing) {
                // Keep the deeper contact
                if (manifold.contacts[i].penetration > manifold.contacts[j].penetration) {
                    manifold.contacts[j] = manifold.contacts[i];
                }
                redundant = true;
                break;
            }
        }
        if (!redundant) {
            if (writeIdx != i)
                manifold.contacts[writeIdx] = manifold.contacts[i];
            ++writeIdx;
        }
    }
    manifold.contactCount = writeIdx;
}

/// Sorts contacts by penetration depth (deepest first) for stable solver ordering.
inline void sortByPenetration(ContactManifold& manifold) noexcept {
    for (u32 i = 1; i < manifold.contactCount; ++i) {
        ContactPoint key = manifold.contacts[i];
        int j = static_cast<int>(i) - 1;
        while (j >= 0 && manifold.contacts[j].penetration < key.penetration) {
            manifold.contacts[j + 1] = manifold.contacts[j];
            --j;
        }
        manifold.contacts[j + 1] = key;
    }
}

/// Full manifold post-processing pipeline:
/// 1. Prune degenerate contacts
/// 2. Remove redundant (too-close) contacts
/// 3. Sort by penetration depth
/// 4. Clamp to kMaxContactPoints
inline void finalizeManifold(ContactManifold& manifold) noexcept {
    pruneDegenerateContacts(manifold);
    removeRedundantContacts(manifold);
    sortByPenetration(manifold);
    if (manifold.contactCount > kMaxContactPoints)
        manifold.contactCount = kMaxContactPoints;
}

} // namespace primeon::math
