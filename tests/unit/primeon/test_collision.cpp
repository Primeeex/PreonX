#include <gtest/gtest.h>

#include "primeon/collision/contact.hpp"
#include "primeon/collision/shapes/support.hpp"
#include "primeon/collision/narrowphase/sphere_sphere.hpp"
#include "primeon/collision/narrowphase/sphere_plane.hpp"
#include "primeon/collision/narrowphase/sphere_aabb.hpp"
#include "primeon/collision/narrowphase/sphere_capsule.hpp"
#include "primeon/collision/narrowphase/aabb_aabb.hpp"
#include "primeon/collision/narrowphase/obb_obb.hpp"
#include "primeon/collision/narrowphase/capsule_capsule.hpp"
#include "primeon/collision/narrowphase/raycast.hpp"
#include "primeon/collision/algorithms/sat.hpp"
#include "primeon/collision/algorithms/gjk.hpp"
#include "primeon/collision/algorithms/epa.hpp"
#include "primeon/collision/broadphase/sweep_and_prune.hpp"
#include "primeon/collision/broadphase/dynamic_aabb_tree.hpp"
#include "primeon/collision/ccd/tof.hpp"
#include "primeon/collision/queries/collision_query.hpp"

using namespace primeon::math;
using namespace primeon::collision;

// ── Contact structures ───────────────────────────────────────────────────────

TEST(Contact, CollisionResultDefaults) {
    CollisionResult r;
    EXPECT_FALSE(r.colliding);
    EXPECT_EQ(r.manifold.normal, kVector3Zero);
    EXPECT_EQ(r.manifold.contactCount, 0u);
}

TEST(Contact, RayHitDefaults) {
    RayHit h;
    EXPECT_FALSE(h.hit);
    EXPECT_FLOAT_EQ(h.distance, std::numeric_limits<f32>::max());
}

// ── Support mappings ─────────────────────────────────────────────────────────

TEST(Support, Sphere) {
    Sphere s{Vector3(1, 0, 0), 1.0f};
    Vector3 sup = supportSphere(s, Vector3(1, 0, 0));
    EXPECT_NEAR(sup.x, 2.0f, kEpsilon);
    EXPECT_NEAR(sup.y, 0.0f, kEpsilon);
    EXPECT_NEAR(sup.z, 0.0f, kEpsilon);
}

TEST(Support, AABB) {
    AABB a{Vector3(-1, -1, -1), Vector3(1, 1, 1)};
    Vector3 sup = supportAABB(a, Vector3(1, 1, 1));
    EXPECT_EQ(sup, Vector3(1, 1, 1));
    Vector3 sup2 = supportAABB(a, Vector3(-1, -1, -1));
    EXPECT_EQ(sup2, Vector3(-1, -1, -1));
}

TEST(Support, OBB) {
    OBB o{Vector3(0, 0, 0), Vector3(1, 2, 3), Quaternion()};
    Vector3 sup = supportOBB(o, Vector3(1, 0, 0));
    EXPECT_NEAR(sup.x, 1.0f, kEpsilon);
    EXPECT_NEAR(sup.y, 2.0f, kEpsilon);
    EXPECT_NEAR(sup.z, 3.0f, kEpsilon);
}

TEST(Support, Capsule) {
    Capsule c{Vector3(-5, 0, 0), Vector3(5, 0, 0), 1.0f};
    Vector3 sup = supportCapsule(c, Vector3(1, 0, 0));
    EXPECT_NEAR(sup.x, 6.0f, kEpsilon);
    Vector3 sup2 = supportCapsule(c, Vector3(-1, 0, 0));
    EXPECT_NEAR(sup2.x, -6.0f, kEpsilon);
}

TEST(Support, Triangle) {
    Triangle t{Vector3(0, 0, 0), Vector3(2, 0, 0), Vector3(1, 2, 0)};
    Vector3 sup = supportTriangle(t, Vector3(0, 1, 0));
    EXPECT_NEAR(sup.y, 2.0f, kEpsilon);
}

TEST(Support, Minkowski) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(0, 0, 0), 1.0f};
    Vector3 sup = supportMinkowski(
        [&](const Vector3& d) { return supportSphere(a, d); },
        [&](const Vector3& d) { return supportSphere(b, d); },
        Vector3(1, 0, 0));
    EXPECT_NEAR(sup.x, 2.0f, kEpsilon);
}

TEST(Support, GenericDispatch) {
    Sphere s{Vector3(0, 0, 0), 1.0f};
    Vector3 sup = supportPrimitive(s, Vector3(1, 0, 0));
    EXPECT_NEAR(sup.x, 1.0f, kEpsilon);
}

// ── Sphere-Sphere narrowphase ────────────────────────────────────────────────

TEST(Narrowphase, SphereSphereOverlap) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(1.5f, 0, 0), 1.0f};
    CollisionResult r = sphereSphere(a, b);
    EXPECT_TRUE(r.colliding);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
}

TEST(Narrowphase, SphereSphereNoOverlap) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(5, 0, 0), 1.0f};
    CollisionResult r = sphereSphere(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, SphereSphereTouching) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(2, 0, 0), 1.0f};
    CollisionResult r = sphereSphere(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, SphereSphereContainment) {
    Sphere a{Vector3(0, 0, 0), 5.0f};
    Sphere b{Vector3(1, 0, 0), 1.0f};
    CollisionResult r = sphereSphere(a, b);
    EXPECT_TRUE(r.colliding);
}

// ── Sphere-Plane narrowphase ─────────────────────────────────────────────────

TEST(Narrowphase, SpherePlaneOverlap) {
    Sphere s{Vector3(0, 0.5f, 0), 1.0f};
    Plane p = Plane::fromPointNormal(Vector3(0, 0, 0), Vector3(0, 1, 0));
    CollisionResult r = spherePlane(s, p);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, SpherePlaneNoOverlap) {
    Sphere s{Vector3(0, 5, 0), 1.0f};
    Plane p = Plane::fromPointNormal(Vector3(0, 0, 0), Vector3(0, 1, 0));
    CollisionResult r = spherePlane(s, p);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, SpherePlaneTouching) {
    Sphere s{Vector3(0, 1, 0), 1.0f};
    Plane p = Plane::fromPointNormal(Vector3(0, 0, 0), Vector3(0, 1, 0));
    CollisionResult r = spherePlane(s, p);
    EXPECT_FALSE(r.colliding);
}

// ── Sphere-AABB narrowphase ──────────────────────────────────────────────────

TEST(Narrowphase, SphereAABBOverlap) {
    Sphere s{Vector3(0.5f, 0.5f, 0), 1.0f};
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    CollisionResult r = sphereAABB(s, a);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, SphereAABBNoOverlap) {
    Sphere s{Vector3(5, 5, 0), 1.0f};
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    CollisionResult r = sphereAABB(s, a);
    EXPECT_FALSE(r.colliding);
}

// ── Sphere-Capsule narrowphase ───────────────────────────────────────────────

TEST(Narrowphase, SphereCapsuleOverlap) {
    Sphere s{Vector3(2, 1, 0), 1.0f};
    Capsule c{Vector3(0, 0, 0), Vector3(5, 0, 0), 0.5f};
    CollisionResult r = sphereCapsule(s, c);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, SphereCapsuleNoOverlap) {
    Sphere s{Vector3(0, 5, 0), 1.0f};
    Capsule c{Vector3(0, 0, 0), Vector3(5, 0, 0), 0.5f};
    CollisionResult r = sphereCapsule(s, c);
    EXPECT_FALSE(r.colliding);
}

// ── AABB-AABB narrowphase ────────────────────────────────────────────────────

TEST(Narrowphase, AABBAABBOverlap) {
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    AABB b{Vector3(1, 1, 1), Vector3(3, 3, 3)};
    CollisionResult r = aabbAABB(a, b);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, AABBAABBNoOverlap) {
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(5, 5, 5), Vector3(6, 6, 6)};
    CollisionResult r = aabbAABB(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, AABBAABBSideBySide) {
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(1, 0, 0), Vector3(2, 1, 1)};
    CollisionResult r = aabbAABB(a, b);
    EXPECT_FALSE(r.colliding);
}

// ── OBB-OBB narrowphase ──────────────────────────────────────────────────────

TEST(Narrowphase, OBBOBBIdentical) {
    OBB o{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    CollisionResult r = obbOBB(o, o);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, OBBOBBNoOverlap) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    OBB b{Vector3(5, 0, 0), Vector3(1, 1, 1), Quaternion()};
    CollisionResult r = obbOBB(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, OBBOBBRotated) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    Quaternion rot = Quaternion::fromAxisAngle(Vector3(0, 1, 0), 0.1f);
    OBB b{Vector3(0.5f, 0, 0), Vector3(1, 1, 1), rot};
    CollisionResult r = obbOBB(a, b);
    EXPECT_TRUE(r.colliding);
}

// ── Capsule-Capsule narrowphase ──────────────────────────────────────────────

TEST(Narrowphase, CapsuleCapsuleOverlap) {
    Capsule a{Vector3(0, 0, 0), Vector3(5, 0, 0), 1.0f};
    Capsule b{Vector3(2, 0.5f, 0), Vector3(7, 0.5f, 0), 1.0f};
    CollisionResult r = capsuleCapsule(a, b);
    EXPECT_TRUE(r.colliding);
}

TEST(Narrowphase, CapsuleCapsuleNoOverlap) {
    Capsule a{Vector3(0, 0, 0), Vector3(1, 0, 0), 0.5f};
    Capsule b{Vector3(5, 0, 0), Vector3(6, 0, 0), 0.5f};
    CollisionResult r = capsuleCapsule(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Narrowphase, CapsuleCapsulePerpendicular) {
    Capsule a{Vector3(0, 0, 0), Vector3(5, 0, 0), 1.0f};
    Capsule b{Vector3(2.5f, -3, 0), Vector3(2.5f, 3, 0), 1.0f};
    CollisionResult r = capsuleCapsule(a, b);
    EXPECT_TRUE(r.colliding);
}

// ── SAT ──────────────────────────────────────────────────────────────────────

TEST(SAT, OBBOBBOverlap) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    OBB b{Vector3(0.5f, 0, 0), Vector3(1, 1, 1), Quaternion()};
    Vector3 normal;
    f32 depth = satOBBFull(a, b, normal);
    EXPECT_GT(depth, 0.0f);
}

TEST(SAT, OBBOBBSeparated) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    OBB b{Vector3(5, 0, 0), Vector3(1, 1, 1), Quaternion()};
    Vector3 normal;
    f32 depth = satOBBFull(a, b, normal);
    EXPECT_LE(depth, 0.0f);
}

// ── GJK ──────────────────────────────────────────────────────────────────────

TEST(GJK, SphereSphereOverlap) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(1.5f, 0, 0), 1.0f};
    auto supportFunc = [&](const Vector3& d) {
        return supportPrimitive(a, d) - supportPrimitive(b, -d);
    };
    EXPECT_TRUE(gjkIntersect(supportFunc));
}

TEST(GJK, SphereSphereSeparated) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(5, 0, 0), 1.0f};
    auto supportFunc = [&](const Vector3& d) {
        return supportPrimitive(a, d) - supportPrimitive(b, -d);
    };
    EXPECT_FALSE(gjkIntersect(supportFunc));
}

TEST(GJK, AABBAABBOverlap) {
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    AABB b{Vector3(1, 1, 1), Vector3(3, 3, 3)};
    auto supportFunc = [&](const Vector3& d) {
        return supportPrimitive(a, d) - supportPrimitive(b, -d);
    };
    EXPECT_TRUE(gjkIntersect(supportFunc));
}

TEST(GJK, OBBOBBOverlap) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion()};
    OBB b{Vector3(0.5f, 0, 0), Vector3(1, 1, 1), Quaternion()};
    auto supportFunc = [&](const Vector3& d) {
        return supportPrimitive(a, d) - supportPrimitive(b, -d);
    };
    EXPECT_TRUE(gjkIntersect(supportFunc));
}

// ── EPA ──────────────────────────────────────────────────────────────────────

TEST(EPA, SphereSphere) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(1.5f, 0, 0), 1.0f};
    auto supportFunc = [&](const Vector3& d) {
        return supportPrimitive(a, d) - supportPrimitive(b, -d);
    };

    GJKSimplex simplex;
    simplex.count = 4;
    simplex.vertices[0] = supportFunc(Vector3(1, 0, 0));
    simplex.vertices[1] = supportFunc(Vector3(0, 1, 0));
    simplex.vertices[2] = supportFunc(Vector3(0, 0, 1));
    simplex.vertices[3] = supportFunc(Vector3(-1, -1, -1));

    EPAResult r = epa(simplex, supportFunc);
    EXPECT_NEAR(r.depth, 0.5f, 0.05f);
    EXPECT_NEAR(std::abs(r.normal.x), 1.0f, 0.05f);
}

// ── Sweep and Prune ──────────────────────────────────────────────────────────

TEST(Broadphase, SAPInsertAndQuery) {
    SAPBroadphase sap;
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f)};
    AABB c{Vector3(5, 5, 5), Vector3(6, 6, 6)};
    sap.insert(1, a);
    sap.insert(2, b);
    sap.insert(3, c);
    EXPECT_EQ(sap.numBodies(), 3u);
    sap.computePairs();
    EXPECT_EQ(sap.pairCount(), 1u);
    EXPECT_TRUE(sap.pairs[0].bodyA == 1 || sap.pairs[0].bodyA == 2);
}

TEST(Broadphase, SAPRemove) {
    SAPBroadphase sap;
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f)};
    sap.insert(1, a);
    sap.insert(2, b);
    sap.remove(1);
    EXPECT_EQ(sap.numBodies(), 1u);
    sap.computePairs();
    EXPECT_EQ(sap.pairCount(), 0u);
}

TEST(Broadphase, SAPUpdate) {
    SAPBroadphase sap;
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(0.5f, 0.5f, 0.5f), Vector3(1.5f, 1.5f, 1.5f)};
    sap.insert(1, a);
    sap.insert(2, b);
    sap.computePairs();
    EXPECT_EQ(sap.pairCount(), 1u);
    AABB a2{Vector3(10, 10, 10), Vector3(11, 11, 11)};
    sap.update(1, a2);
    sap.computePairs();
    EXPECT_EQ(sap.pairCount(), 0u);
}

// ── Dynamic AABB Tree ────────────────────────────────────────────────────────

TEST(DynamicAABBTree, InsertAndQuery) {
    DynamicAABBTree tree(32);
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(2, 2, 2), Vector3(3, 3, 3)};
    i32 idA = tree.insert(1, a);
    i32 idB = tree.insert(2, b);
    EXPECT_GE(idA, 0);
    EXPECT_GE(idB, 0);
    EXPECT_EQ(tree.nodeCount(), 2u);
    AABB q{Vector3(0.5f, 0.5f, 0.5f), Vector3(2.5f, 2.5f, 2.5f)};
    tree.queryAABB(q);
    EXPECT_GE(tree.queryResults.size(), 1u);
}

TEST(DynamicAABBTree, Remove) {
    DynamicAABBTree tree(32);
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    i32 idA = tree.insert(1, a);
    tree.remove(idA);
    EXPECT_EQ(tree.nodeCount(), 0u);
}

TEST(DynamicAABBTree, QuerySphere) {
    DynamicAABBTree tree(32);
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    (void)tree.insert(1, a);
    tree.querySphere(Vector3(5, 5, 5), 1.0f);
    EXPECT_EQ(tree.queryResults.size(), 0u);
    tree.querySphere(Vector3(0, 0, 0), 2.0f);
    EXPECT_GE(tree.queryResults.size(), 1u);
}

TEST(DynamicAABBTree, QueryRay) {
    DynamicAABBTree tree(32);
    AABB a{Vector3(2, -1, -1), Vector3(3, 1, 1)};
    (void)tree.insert(1, a);
    Vector3 origin(0, 0, 0);
    Vector3 dir(1, 0, 0);
    tree.queryRay(origin, dir, 10.0f);
    EXPECT_GE(tree.queryResults.size(), 1u);
}

TEST(DynamicAABBTree, Clear) {
    DynamicAABBTree tree(32);
    (void)tree.insert(1, AABB{Vector3(0, 0, 0), Vector3(1, 1, 1)});
    (void)tree.insert(2, AABB{Vector3(2, 2, 2), Vector3(3, 3, 3)});
    tree.clear();
    EXPECT_TRUE(tree.empty());
}

TEST(DynamicAABBTree, BalanceIntegrityAfterManyInserts) {
    DynamicAABBTree tree(256);
    for (u32 i = 0; i < 50; ++i) {
        f32 x = static_cast<f32>(i) * 2.0f;
        (void)tree.insert(i + 1, AABB{Vector3(x, 0, 0), Vector3(x + 1, 1, 1)});
    }
    EXPECT_EQ(tree.nodeCount(), 50u);
    for (u32 i = 0; i < 50; ++i) {
        f32 x = static_cast<f32>(i) * 2.0f;
        AABB q{Vector3(x - 0.5f, -0.5f, -0.5f), Vector3(x + 1.5f, 1.5f, 1.5f)};
        tree.queryAABB(q);
        EXPECT_GE(tree.queryResults.size(), 1u);
    }
}

TEST(DynamicAABBTree, ParentChildConsistency) {
    DynamicAABBTree tree(256);
    for (u32 i = 0; i < 30; ++i) {
        f32 x = static_cast<f32>(i);
        (void)tree.insert(i + 1, AABB{Vector3(x, 0, 0), Vector3(x + 0.5f, 0.5f, 0.5f)});
    }
    if (tree.root != -1) {
        std::vector<u32> stack;
        stack.push_back(static_cast<u32>(tree.root));
        while (!stack.empty()) {
            u32 idx = stack.back();
            stack.pop_back();
            const auto& node = tree.nodes[idx];
            if (node.children[0] != -1) {
                EXPECT_EQ(tree.nodes[node.children[0]].parent, static_cast<i32>(idx));
                stack.push_back(static_cast<u32>(node.children[0]));
            }
            if (node.children[1] != -1) {
                EXPECT_EQ(tree.nodes[node.children[1]].parent, static_cast<i32>(idx));
                stack.push_back(static_cast<u32>(node.children[1]));
            }
        }
    }
}

TEST(DynamicAABBTree, NodePoolExhaustionDoesNotCrash) {
    DynamicAABBTree tree(8);
    for (u32 i = 0; i < 4; ++i) {
        f32 x = static_cast<f32>(i) * 10.0f;
        i32 id = tree.insert(i + 1, AABB{Vector3(x, 0, 0), Vector3(x + 1, 1, 1)});
        EXPECT_GE(id, 0);
    }
    EXPECT_EQ(tree.nodeCount(), 4u);
}

// ── CCD / TOI ────────────────────────────────────────────────────────────────

TEST(CCD, TOISphereSphere) {
    Vector3 posA0(0, 0, 0);
    Vector3 posB0(5, 0, 0);
    Vector3 posA1(4, 0, 0);
    Vector3 posB1(1, 0, 0);
    TOIResult r = toiSphereSphere(posA0, 1.0f, posB0, 1.0f, posA1, posB1);
    EXPECT_TRUE(r.hit);
    EXPECT_GT(r.toi, 0.0f);
    EXPECT_LT(r.toi, 1.0f);
}

TEST(CCD, TOISphereSphereNoHit) {
    Vector3 posA0(0, 0, 0);
    Vector3 posB0(0, 10, 0);
    Vector3 posA1(0, 0.1f, 0);
    Vector3 posB1(0, 10, 0);
    TOIResult r = toiSphereSphere(posA0, 1.0f, posB0, 1.0f, posA1, posB1);
    EXPECT_FALSE(r.hit);
}

TEST(CCD, TOISpherePlane) {
    Vector3 pos0(0, 5, 0);
    Vector3 pos1(0, -5, 0);
    Plane p = Plane::fromPointNormal(Vector3(0, 0, 0), Vector3(0, 1, 0));
    TOIResult r = toiSpherePlane(pos0, 1.0f, pos1, p.normal, p.distance);
    EXPECT_TRUE(r.hit);
    EXPECT_GT(r.toi, 0.0f);
    EXPECT_LT(r.toi, 1.0f);
}

TEST(CCD, TOISphereAABB) {
    Vector3 sphere0(0, 0, 0);
    Vector3 sphere1(5, 0, 0);
    AABB box{Vector3(3, -1, -1), Vector3(4, 1, 1)};
    TOIResult r = toiSphereAABB(sphere0, 1.0f, sphere1, box);
    EXPECT_TRUE(r.hit);
    EXPECT_GT(r.toi, 0.0f);
}

TEST(CCD, TOISphereCapsule) {
    Vector3 sphere0(0, 0, 0);
    Vector3 sphere1(5, 0, 0);
    Capsule c{Vector3(3, -3, 0), Vector3(3, 3, 0), 1.0f};
    TOIResult r = toiSphereCapsule(sphere0, 1.0f, sphere1,
                                    c.start, c.end, c.start, c.end, c.radius);
    EXPECT_TRUE(r.hit);
}

// ── Raycast ──────────────────────────────────────────────────────────────────

TEST(Raycast, RaySphereHit) {
    Sphere s{Vector3(3, 0, 0), 1.0f};
    Ray r{Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RayHit h = raySphere(r, s);
    EXPECT_TRUE(h.hit);
    EXPECT_NEAR(h.distance, 2.0f, kEpsilon);
}

TEST(Raycast, RaySphereMiss) {
    Sphere s{Vector3(3, 3, 0), 1.0f};
    Ray r{Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RayHit h = raySphere(r, s);
    EXPECT_FALSE(h.hit);
}

TEST(Raycast, RayPlaneHit) {
    Plane p = Plane::fromPointNormal(Vector3(0, 5, 0), Vector3(0, 1, 0));
    Ray r{Vector3(0, 0, 0), Vector3(0, 1, 0)};
    RayHit h = rayPlane(r, p);
    EXPECT_TRUE(h.hit);
    EXPECT_NEAR(h.distance, 5.0f, kEpsilon);
}

TEST(Raycast, RayAABBHit) {
    AABB a{Vector3(2, -1, -1), Vector3(3, 1, 1)};
    Ray r{Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RayHit h = rayAABB(r, a);
    EXPECT_TRUE(h.hit);
    EXPECT_NEAR(h.distance, 2.0f, kEpsilon);
}

TEST(Raycast, RayAABBMiss) {
    AABB a{Vector3(2, 2, 2), Vector3(3, 3, 3)};
    Ray r{Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RayHit h = rayAABB(r, a);
    EXPECT_FALSE(h.hit);
}

TEST(Raycast, RayTriangleHit) {
    Triangle t{Vector3(-1, 0, -1), Vector3(1, 0, -1), Vector3(0, 0, 1)};
    Ray r{Vector3(0, 5, 0), Vector3(0, -1, 0)};
    RayHit h = rayTriangle(r, t);
    EXPECT_TRUE(h.hit);
    EXPECT_NEAR(h.distance, 5.0f, kEpsilon);
}

// ── Query dispatch ───────────────────────────────────────────────────────────

TEST(Query, IntersectsSphereSphere) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(1.5f, 0, 0), 1.0f};
    EXPECT_TRUE(intersects(a, b));
    Sphere c{Vector3(5, 0, 0), 1.0f};
    EXPECT_FALSE(intersects(a, c));
}

TEST(Query, ContactSphereAABB) {
    Sphere s{Vector3(0.5f, 0.5f, 0), 1.0f};
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    CollisionResult r = contact(s, a);
    EXPECT_TRUE(r.colliding);
}

TEST(Query, RaycastAABB) {
    AABB a{Vector3(2, -1, -1), Vector3(3, 1, 1)};
    Ray r{Vector3(0, 0, 0), Vector3(1, 0, 0)};
    RayHit h = raycast(r, a);
    EXPECT_TRUE(h.hit);
}
