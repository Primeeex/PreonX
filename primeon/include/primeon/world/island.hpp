#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include <vector>

namespace primeon::math {

// ── Island ────────────────────────────────────────────────────────────────
//
// Islands partition the simulation into independent groups.
// If body A contacts body B, they are in the same island.
// Each island can be solved independently (or in parallel).
//
// Implementation: Union-Find (Disjoint Set Union) with path compression
// and union-by-rank. O(α(n)) amortized per operation (effectively constant).

/// A simulation island: a connected group of bodies and contacts.
struct Island {
    std::vector<u32> bodies;
    std::vector<u32> contacts;
    bool isAwake = true;

    constexpr Island() noexcept = default;

    void addBody(u32 bodyID) noexcept { bodies.push_back(bodyID); }
    void addContact(u32 contactIndex) noexcept { contacts.push_back(contactIndex); }

    void clear() noexcept {
        bodies.clear();
        contacts.clear();
        isAwake = true;
    }
};

/// Union-Find for island generation.
struct UnionFind {
    std::vector<u32> parent;
    std::vector<u32> rank;
    u32 count = 0;

    UnionFind() = default;

    explicit UnionFind(u32 maxElements) noexcept {
        resize(maxElements);
    }

    void resize(u32 maxElements) noexcept {
        parent.resize(maxElements);
        rank.resize(maxElements, 0);
        count = maxElements;
        for (u32 i = 0; i < maxElements; ++i) {
            parent[i] = i;
        }
    }

    void reset(u32 n) noexcept {
        count = n;
        for (u32 i = 0; i < n; ++i) {
            parent[i] = i;
            rank[i] = 0;
        }
    }

    [[nodiscard]] u32 find(u32 x) noexcept {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];  // path compression
            x = parent[x];
        }
        return x;
    }

    void unite(u32 x, u32 y) noexcept {
        u32 rootX = find(x);
        u32 rootY = find(y);
        if (rootX == rootY) return;

        // Union by rank
        if (rank[rootX] < rank[rootY]) {
            parent[rootX] = rootY;
        } else if (rank[rootX] > rank[rootY]) {
            parent[rootY] = rootX;
        } else {
            parent[rootY] = rootX;
            ++rank[rootX];
        }
    }

    [[nodiscard]] bool connected(u32 x, u32 y) noexcept {
        return find(x) == find(y);
    }
};

// ── Island Builder ─────────────────────────────────────────────────────────
//
// Builds islands from the contact graph.
// Input: list of (bodyA, bodyB) contact pairs.
// Output: list of Islands, each containing body IDs and contact indices.

/// Island builder that partitions the simulation into independent groups.
struct IslandBuilder {
    UnionFind uf;
    std::vector<Island> islands;

    IslandBuilder() = default;

    explicit IslandBuilder(u32 maxBodies) noexcept : uf(maxBodies) {}

    /// Builds islands from a set of contact pairs.
    /// contactPairs[i] = {bodyIDA, bodyIDB}
    /// The builder groups bodies connected by contacts into islands.
    [[nodiscard]] const std::vector<Island>& build(
        u32 bodyCount,
        const std::pair<u32, u32>* contactPairs,
        u32 contactCount,
        const RigidBody* bodies) noexcept
    {
        islands.clear();
        if (bodyCount == 0) return islands;

        uf.reset(bodyCount);

        // Unite bodies connected by contacts
        for (u32 i = 0; i < contactCount; ++i) {
            u32 a = contactPairs[i].first;
            u32 b = contactPairs[i].second;
            if (a < bodyCount && b < bodyCount) {
                uf.unite(a, b);
            }
        }

        // Group bodies by root
        // First pass: count islands and assign indices
        std::vector<u32> rootIsland(bodyCount, 0xFFFFFFFF);
        std::vector<u32> rootMap;
        rootMap.reserve(bodyCount);

        for (u32 i = 0; i < bodyCount; ++i) {
            if (bodies[i].id == kInvalidBody || !bodies[i].enabled) continue;

            u32 root = uf.find(i);
            if (rootIsland[root] == 0xFFFFFFFF) {
                rootIsland[root] = static_cast<u32>(rootMap.size());
                rootMap.push_back(root);
                islands.emplace_back();
            }
        }

        // Second pass: assign bodies to islands
        for (u32 i = 0; i < bodyCount; ++i) {
            if (bodies[i].id == kInvalidBody || !bodies[i].enabled) continue;

            u32 root = uf.find(i);
            u32 islandIdx = rootIsland[root];
            if (islandIdx < islands.size()) {
                islands[islandIdx].addBody(i);

                // Wake the island if any body is awake
                if (bodies[i].isAwake()) {
                    islands[islandIdx].isAwake = true;
                }
            }
        }

        // Third pass: assign contacts to islands
        for (u32 i = 0; i < contactCount; ++i) {
            u32 a = contactPairs[i].first;
            u32 b = contactPairs[i].second;
            if (a >= bodyCount || b >= bodyCount) continue;

            u32 root = uf.find(a);
            u32 islandIdx = rootIsland[root];
            if (islandIdx < islands.size()) {
                islands[islandIdx].addContact(i);
            }
        }

        return islands;
    }

    void clear() noexcept {
        islands.clear();
    }
};

} // namespace primeon::math
