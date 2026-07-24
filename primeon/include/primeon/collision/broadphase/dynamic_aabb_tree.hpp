#pragma once

#include "primeon/math/scalar/scalar.hpp"
#include "primeon/math/vector/vector3.hpp"
#include "primeon/geometry/primitives/aabb.hpp"
#include <vector>
#include <algorithm>
#include <limits>

namespace primeon::collision {

using math::f32;
using math::u32;
using foundation::i32;

// ── Dynamic AABB Tree ────────────────────────────────────────────────────────
//
// A balanced binary tree of AABBs used for broadphase queries. Each leaf
// stores a body's AABB; internal nodes are the union of their children's AABBs.
//
// Supports O(log n) insert/remove/update and O(log n + k) overlap queries.
// A free-list of node indices manages memory without dynamic allocation.

inline constexpr u32 kInvalidNode = 0xFFFFFFFF;

/// AABB tree node.
struct AABBTreeNode {
    math::AABB aabb = {};
    i32 parent = -1;
    i32 children[2] = {-1, -1};
    i32 height = 0;
    bool isLeaf() const noexcept { return children[0] == -1 && children[1] == -1; }
};

/// Helper: return an AABB that encloses nothing (inverted min/max for merge accumulation).
[[nodiscard]] inline math::AABB aabbIdentity() noexcept {
    f32 big = std::numeric_limits<f32>::max();
    return math::AABB{math::Vector3(big, big, big), math::Vector3(-big, -big, -big)};
}

/// Helper: test if two AABBs overlap.
[[nodiscard]] inline bool aabbsOverlap(const math::AABB& a, const math::AABB& b) noexcept {
    if (a.max.x < b.min.x || a.min.x > b.max.x) return false;
    if (a.max.y < b.min.y || a.min.y > b.max.y) return false;
    if (a.max.z < b.min.z || a.min.z > b.max.z) return false;
    return true;
}

/// Helper: test if AABB `outer` fully contains AABB `inner`.
[[nodiscard]] inline bool aabbContains(const math::AABB& outer, const math::AABB& inner) noexcept {
    return inner.min.x >= outer.min.x && inner.max.x <= outer.max.x &&
           inner.min.y >= outer.min.y && inner.max.y <= outer.max.y &&
           inner.min.z >= outer.min.z && inner.max.z <= outer.max.z;
}

/// Dynamic AABB tree.
struct DynamicAABBTree {
    std::vector<AABBTreeNode> nodes = {};
    std::vector<u32> freeList = {};
    std::vector<u32> queryResults = {};
    i32 root = -1;
    u32 capacity = 0;
    u32 size = 0;

    explicit DynamicAABBTree(u32 maxNodes = 256) noexcept : capacity(maxNodes) {
        nodes.resize(maxNodes);
        freeList.reserve(maxNodes);
        queryResults.reserve(maxNodes);
        for (u32 i = maxNodes; i > 0; --i)
            freeList.push_back(i - 1);
    }

    [[nodiscard]] i32 allocateNode() noexcept {
        if (!freeList.empty()) {
            i32 idx = static_cast<i32>(freeList.back());
            freeList.pop_back();
            nodes[idx] = AABBTreeNode{};
            return idx;
        }
        if (size >= capacity) return -1;
        i32 idx = static_cast<i32>(size);
        nodes.push_back(AABBTreeNode{});
        capacity = static_cast<u32>(nodes.size());
        return idx;
    }

    void freeNode(i32 idx) noexcept {
        if (idx < 0 || static_cast<u32>(idx) >= capacity) return;
        nodes[idx] = AABBTreeNode{};
        freeList.push_back(static_cast<u32>(idx));
    }

    [[nodiscard]] i32 balance(i32 nodeIdx) noexcept {
        if (nodeIdx < 0 || static_cast<u32>(nodeIdx) >= capacity) return nodeIdx;
        AABBTreeNode& A = nodes[nodeIdx];
        if (A.isLeaf()) return nodeIdx;

        i32 iB = A.children[0];
        i32 iC = A.children[1];
        if (iB < 0 || iC < 0) return nodeIdx;

        AABBTreeNode& B = nodes[iB];
        AABBTreeNode& C = nodes[iC];

        i32 bal = C.height - B.height;

        if (bal > 1) {
            i32 iF = C.children[0];
            i32 iG = C.children[1];
            if (iF < 0 || iG < 0) return nodeIdx;
            AABBTreeNode& F = nodes[iF];
            AABBTreeNode& G = nodes[iG];

            C.children[1] = nodeIdx;
            C.parent = A.parent;
            A.parent = iC;

            if (C.parent >= 0) {
                AABBTreeNode& par = nodes[C.parent];
                if (par.children[0] == nodeIdx)
                    par.children[0] = iC;
                else
                    par.children[1] = iC;
            } else {
                root = iC;
            }

            if (F.height > G.height) {
                C.children[0] = iF;
                C.children[1] = nodeIdx;
                A.children[1] = iG;
                G.parent = nodeIdx;
                A.aabb = B.aabb.merged(G.aabb);
                C.aabb = A.aabb.merged(F.aabb);
                A.height = 1 + std::max(B.height, G.height);
                C.height = 1 + std::max(A.height, F.height);
            } else {
                C.children[0] = nodeIdx;
                C.children[1] = iF;
                A.children[1] = iG;
                G.parent = nodeIdx;
                A.aabb = B.aabb.merged(G.aabb);
                C.aabb = A.aabb.merged(F.aabb);
                A.height = 1 + std::max(B.height, G.height);
                C.height = 1 + std::max(A.height, F.height);
            }
            return iC;
        }

        if (bal < -1) {
            i32 iD = B.children[0];
            i32 iE = B.children[1];
            if (iD < 0 || iE < 0) return nodeIdx;
            AABBTreeNode& D = nodes[iD];
            AABBTreeNode& E = nodes[iE];

            B.children[1] = nodeIdx;
            B.parent = A.parent;
            A.parent = iB;

            if (B.parent >= 0) {
                AABBTreeNode& par = nodes[B.parent];
                if (par.children[0] == nodeIdx)
                    par.children[0] = iB;
                else
                    par.children[1] = iB;
            } else {
                root = iB;
            }

            if (D.height > E.height) {
                B.children[0] = iD;
                B.children[1] = nodeIdx;
                A.children[0] = iE;
                E.parent = nodeIdx;
                A.aabb = C.aabb.merged(E.aabb);
                B.aabb = A.aabb.merged(D.aabb);
                A.height = 1 + std::max(C.height, E.height);
                B.height = 1 + std::max(A.height, D.height);
            } else {
                B.children[0] = nodeIdx;
                B.children[1] = iD;
                A.children[0] = iE;
                E.parent = nodeIdx;
                A.aabb = C.aabb.merged(E.aabb);
                B.aabb = A.aabb.merged(D.aabb);
                A.height = 1 + std::max(C.height, E.height);
                B.height = 1 + std::max(A.height, D.height);
            }
            return iB;
        }

        return nodeIdx;
    }

    [[nodiscard]] i32 insert(u32 /*bodyId*/, const math::AABB& aabb) noexcept {
        i32 idx = allocateNode();
        if (idx < 0) return -1;
        AABBTreeNode& node = nodes[idx];
        node.aabb = aabb;
        node.aabb.min = node.aabb.min - math::Vector3(math::kEpsilon, math::kEpsilon, math::kEpsilon);
        node.aabb.max = node.aabb.max + math::Vector3(math::kEpsilon, math::kEpsilon, math::kEpsilon);
        node.height = 0;
        node.children[0] = -1;
        node.children[1] = -1;
        node.parent = -1;

        if (root < 0) {
            root = idx;
            size++;
            return idx;
        }

        i32 bestSibling = root;
        f32 bestCost = nodes[root].aabb.surfaceArea();

        struct SearchFrame { i32 nodeIdx; f32 cost; };
        std::vector<SearchFrame> stack;
        stack.push_back({root, 0.0f});

        while (!stack.empty()) {
            SearchFrame frame = stack.back();
            stack.pop_back();
            i32 n = frame.nodeIdx;
            if (n < 0) continue;

            AABBTreeNode& nodeN = nodes[n];
            math::AABB combined = nodeN.aabb.merged(aabb);
            f32 combinedCost = combined.surfaceArea();
            f32 cost = combinedCost + frame.cost;

            if (cost < bestCost) {
                bestCost = cost;
                bestSibling = n;
            }

            if (nodeN.isLeaf()) continue;

            f32 newCost = combinedCost - nodeN.aabb.surfaceArea() + frame.cost;
            if (newCost < bestCost) {
                stack.push_back({nodeN.children[0], newCost});
                stack.push_back({nodeN.children[1], newCost});
            }
        }

        i32 oldParent = nodes[bestSibling].parent;
        i32 newParent = allocateNode();
        if (newParent < 0) return -1;

        AABBTreeNode& par = nodes[newParent];
        par.children[0] = bestSibling;
        par.children[1] = idx;
        par.aabb = nodes[bestSibling].aabb.merged(aabb);
        par.parent = oldParent;
        par.height = nodes[bestSibling].height + 1;

        nodes[bestSibling].parent = newParent;
        nodes[idx].parent = newParent;

        if (oldParent < 0) {
            root = newParent;
        } else {
            AABBTreeNode& oldPar = nodes[oldParent];
            if (oldPar.children[0] == bestSibling)
                oldPar.children[0] = newParent;
            else
                oldPar.children[1] = newParent;
        }

        i32 current = newParent;
        while (current >= 0) {
            current = balance(current);
            AABBTreeNode& nd = nodes[current];
            i32 c0 = nd.children[0];
            i32 c1 = nd.children[1];
            nd.height = 1 + std::max(
                c0 >= 0 ? nodes[c0].height : 0,
                c1 >= 0 ? nodes[c1].height : 0);
            nd.aabb = aabbIdentity();
            if (c0 >= 0) nd.aabb = nd.aabb.merged(nodes[c0].aabb);
            if (c1 >= 0) nd.aabb = nd.aabb.merged(nodes[c1].aabb);
            current = nd.parent;
        }

        size++;
        return idx;
    }

    void remove(i32 idx) noexcept {
        if (idx < 0 || static_cast<u32>(idx) >= capacity) return;
        if (nodes[idx].isLeaf() && nodes[idx].parent < 0 && root != idx) return;

        if (root == idx) {
            root = -1;
            freeNode(idx);
            if (size > 0) size--;
            return;
        }

        i32 parentIdx = nodes[idx].parent;
        i32 grandparent = nodes[parentIdx].parent;
        i32 sibling = (nodes[parentIdx].children[0] == idx)
                        ? nodes[parentIdx].children[1]
                        : nodes[parentIdx].children[0];

        if (grandparent >= 0) {
            if (nodes[grandparent].children[0] == parentIdx)
                nodes[grandparent].children[0] = sibling;
            else
                nodes[grandparent].children[1] = sibling;
            nodes[sibling].parent = grandparent;
            freeNode(parentIdx);

            i32 current = grandparent;
            while (current >= 0) {
                current = balance(current);
                AABBTreeNode& nd = nodes[current];
                i32 c0 = nd.children[0];
                i32 c1 = nd.children[1];
                nd.height = 1 + std::max(
                    c0 >= 0 ? nodes[c0].height : 0,
                    c1 >= 0 ? nodes[c1].height : 0);
                nd.aabb = aabbIdentity();
                if (c0 >= 0) nd.aabb = nd.aabb.merged(nodes[c0].aabb);
                if (c1 >= 0) nd.aabb = nd.aabb.merged(nodes[c1].aabb);
                current = nd.parent;
            }
        } else {
            root = sibling;
            nodes[sibling].parent = -1;
            freeNode(parentIdx);
        }

        freeNode(idx);
        if (size > 0) size--;
    }

    void update(i32 idx, const math::AABB& newAABB) noexcept {
        if (idx < 0 || static_cast<u32>(idx) >= capacity) return;
        math::AABB expanded = newAABB;
        expanded.min = expanded.min - math::Vector3(math::kEpsilon, math::kEpsilon, math::kEpsilon);
        expanded.max = expanded.max + math::Vector3(math::kEpsilon, math::kEpsilon, math::kEpsilon);

        if (aabbContains(expanded, nodes[idx].aabb)) return;

        remove(idx);
        (void)insert(0, expanded);
    }

    void queryAABB(const math::AABB& aabb) noexcept {
        queryResults.clear();
        if (root < 0) return;
        std::vector<i32> stk;
        stk.push_back(root);
        while (!stk.empty()) {
            i32 idx = stk.back();
            stk.pop_back();
            if (idx < 0) continue;
            AABBTreeNode& node = nodes[idx];
            if (!aabbsOverlap(node.aabb, aabb)) continue;
            if (node.isLeaf()) {
                queryResults.push_back(static_cast<u32>(idx));
            } else {
                stk.push_back(node.children[0]);
                stk.push_back(node.children[1]);
            }
        }
    }

    void querySphere(const math::Vector3& center, f32 radius) noexcept {
        math::AABB aabb;
        aabb.min = center - math::Vector3(radius, radius, radius);
        aabb.max = center + math::Vector3(radius, radius, radius);
        queryAABB(aabb);
    }

    void queryRay(const math::Vector3& origin, const math::Vector3& dir, f32 maxDist) noexcept {
        queryResults.clear();
        if (root < 0) return;

        f32 tMin = 0.0f;
        f32 tMax = maxDist;
        for (int i = 0; i < 3; ++i) {
            f32 invD = 1.0f / ((&dir.x)[i] != 0.0f ? (&dir.x)[i] : math::kEpsilon);
            f32 t0 = ((&nodes[root].aabb.min.x)[i] - (&origin.x)[i]) * invD;
            f32 t1 = ((&nodes[root].aabb.max.x)[i] - (&origin.x)[i]) * invD;
            if (t0 > t1) std::swap(t0, t1);
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
            if (tMin > tMax) return;
        }

        struct RayFrame { i32 idx; f32 tMin; f32 tMax; };
        std::vector<RayFrame> stk;
        stk.push_back({root, tMin, tMax});

        while (!stk.empty()) {
            RayFrame frame = stk.back();
            stk.pop_back();
            i32 idx = frame.idx;
            if (idx < 0) continue;
            AABBTreeNode& node = nodes[idx];
            if (node.isLeaf()) {
                queryResults.push_back(static_cast<u32>(idx));
                continue;
            }
            for (int c = 0; c < 2; ++c) {
                i32 child = node.children[c];
                if (child < 0) continue;
                AABBTreeNode& childNode = nodes[child];
                f32 t0 = frame.tMin;
                f32 t1 = frame.tMax;
                for (int i = 0; i < 3; ++i) {
                    f32 invD = 1.0f / ((&dir.x)[i] != 0.0f ? (&dir.x)[i] : math::kEpsilon);
                    f32 tNear = ((&childNode.aabb.min.x)[i] - (&origin.x)[i]) * invD;
                    f32 tFar  = ((&childNode.aabb.max.x)[i] - (&origin.x)[i]) * invD;
                    if (tNear > tFar) std::swap(tNear, tFar);
                    t0 = std::max(t0, tNear);
                    t1 = std::min(t1, tFar);
                    if (t0 > t1) break;
                }
                if (t0 <= t1)
                    stk.push_back({child, t0, t1});
            }
        }
    }

    [[nodiscard]] constexpr u32 nodeCount() const noexcept { return size; }
    [[nodiscard]] constexpr bool empty() const noexcept { return size == 0; }

    void clear() noexcept {
        for (u32 i = capacity; i > 0; --i)
            freeList.push_back(i - 1);
        root = -1;
        size = 0;
        queryResults.clear();
        for (auto& n : nodes) n = AABBTreeNode{};
    }
};

} // namespace primeon::collision
