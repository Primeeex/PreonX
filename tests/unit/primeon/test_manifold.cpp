#include <gtest/gtest.h>

#include "primeon/collision/contact.hpp"
#include "primeon/collision/manifold/contact_generator.hpp"
#include "primeon/collision/manifold/contact_persistence.hpp"
#include "primeon/constraints/contact_constraint.hpp"
#include "primeon/constraints/jacobian.hpp"

using namespace primeon::math;

// ═══════════════════════════════════════════════════════════════════════════
// ContactFeature & makeContactID
// ═══════════════════════════════════════════════════════════════════════════

TEST(ContactFeature, DefaultConstruction) {
    ContactFeature f;
    EXPECT_EQ(f.type, FeatureType::None);
    EXPECT_EQ(f.indexA, 0u);
    EXPECT_EQ(f.indexB, 0u);
}

TEST(ContactFeature, Equality) {
    ContactFeature a(FeatureType::Face, 1, 2);
    ContactFeature b(FeatureType::Face, 1, 2);
    ContactFeature c(FeatureType::Edge, 1, 2);
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(MakeContactID, OrderIndependent) {
    ContactFeature a(FeatureType::Face, 3, 7);
    ContactFeature b(FeatureType::Edge, 5, 2);
    u32 idAB = makeContactID(a, b);
    u32 idBA = makeContactID(b, a);
    EXPECT_EQ(idAB, idBA);
}

TEST(MakeContactID, DifferentFeaturesDifferentIDs) {
    ContactFeature a(FeatureType::Face, 1, 2);
    ContactFeature b(FeatureType::Face, 3, 4);
    EXPECT_NE(makeContactID(a, b), makeContactID(a, a));
}

// ═══════════════════════════════════════════════════════════════════════════
// ContactPoint
// ═══════════════════════════════════════════════════════════════════════════

TEST(ContactPoint, ComputeID) {
    ContactPoint cp;
    cp.featureA = ContactFeature(FeatureType::Vertex, 0, 0);
    cp.featureB = ContactFeature(FeatureType::Face, 1, 1);
    cp.computeID();
    EXPECT_NE(cp.contactID, 0u);
}

TEST(ContactPoint, HasSameID) {
    ContactPoint a, b;
    a.featureA = ContactFeature(FeatureType::Face, 2, 3);
    a.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    a.computeID();
    b.featureA = a.featureA;
    b.featureB = a.featureB;
    b.computeID();
    EXPECT_TRUE(a.hasSameID(b));

    ContactPoint c;
    c.computeID(); // default features -> ID = 0
    EXPECT_FALSE(a.hasSameID(c));
}

// ═══════════════════════════════════════════════════════════════════════════
// ContactManifold
// ═══════════════════════════════════════════════════════════════════════════

TEST(ContactManifold, AddAndRemove) {
    ContactManifold m;
    EXPECT_TRUE(m.empty());

    ContactPoint cp;
    cp.penetration = 0.5f;
    EXPECT_TRUE(m.addContact(cp));
    EXPECT_EQ(m.contactCount, 1u);
    EXPECT_FALSE(m.empty());

    m.removeContact(0);
    EXPECT_TRUE(m.empty());
}

TEST(ContactManifold, Full) {
    ContactManifold m;
    for (u32 i = 0; i < kMaxContactPoints; ++i) {
        EXPECT_TRUE(m.addContact(ContactPoint()));
    }
    EXPECT_TRUE(m.full());
    ContactPoint extra;
    EXPECT_FALSE(m.addContact(extra));
}

TEST(ContactManifold, DeepestIndex) {
    ContactManifold m;
    ContactPoint a; a.penetration = 0.1f;
    ContactPoint b; b.penetration = 0.5f;
    ContactPoint c; c.penetration = 0.3f;
    m.addContact(a);
    m.addContact(b);
    m.addContact(c);
    EXPECT_EQ(m.deepestIndex(), 1u);
}

TEST(ContactManifold, MaxPenetration) {
    ContactManifold m;
    ContactPoint a; a.penetration = 0.1f;
    ContactPoint b; b.penetration = 0.5f;
    m.addContact(a);
    m.addContact(b);
    EXPECT_FLOAT_EQ(m.maxPenetration(), 0.5f);
}

TEST(ContactManifold, Clear) {
    ContactManifold m;
    m.addContact(ContactPoint());
    m.addContact(ContactPoint());
    m.normal = kVector3UnitY;
    m.bodyIDA = 42;
    m.bodyIDB = 99;
    m.clear();
    EXPECT_TRUE(m.empty());
    EXPECT_EQ(m.normal, kVector3UnitY);
    EXPECT_EQ(m.bodyIDA, 42u);
}

TEST(ContactManifold, ComputeTangents) {
    ContactManifold m;
    m.normal = kVector3UnitY;
    ContactPoint cp;
    m.addContact(cp);
    m.computeTangents();
    // Tangent should be perpendicular to normal
    f32 dotU = primeon::math::abs(m.contacts[0].tangentU.dot(m.normal));
    f32 dotV = primeon::math::abs(m.contacts[0].tangentV.dot(m.normal));
    EXPECT_LT(dotU, kEpsilon);
    EXPECT_LT(dotV, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// ContactPair
// ═══════════════════════════════════════════════════════════════════════════

TEST(ContactPair, Matches) {
    ContactPair p(10, 20, ContactManifold());
    EXPECT_TRUE(p.matches(10, 20));
    EXPECT_TRUE(p.matches(20, 10));
    EXPECT_FALSE(p.matches(10, 30));
}

// ═══════════════════════════════════════════════════════════════════════════
// Contact Generators
// ═══════════════════════════════════════════════════════════════════════════

// --- Sphere-Sphere ---

TEST(Generator, SphereSphereOverlap) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(1, 0, 0), 1.0f};
    auto r = generateSphereSphere(a, b, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_EQ(r.manifold.contactCount, 1u);
    EXPECT_EQ(r.manifold.bodyIDA, 1u);
    EXPECT_EQ(r.manifold.bodyIDB, 2u);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
}

TEST(Generator, SphereSphereSeparated) {
    Sphere a{Vector3(0, 0, 0), 1.0f};
    Sphere b{Vector3(5, 0, 0), 1.0f};
    auto r = generateSphereSphere(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Generator, SphereSphereConcentric) {
    Sphere a{Vector3(0, 0, 0), 2.0f};
    Sphere b{Vector3(0, 0, 0), 1.0f};
    auto r = generateSphereSphere(a, b);
    EXPECT_TRUE(r.colliding);
    EXPECT_FLOAT_EQ(r.manifold.contacts[0].penetration, 3.0f);
}

// --- Sphere-Plane ---

TEST(Generator, SpherePlaneOverlap) {
    Sphere s{Vector3(0, 0.5f, 0), 1.0f};
    Plane p{kVector3UnitY, 0.0f};
    auto r = generateSpherePlane(s, p, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_EQ(r.manifold.contactCount, 1u);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
    EXPECT_EQ(r.manifold.contacts[0].featureB.type, FeatureType::Face);
}

TEST(Generator, SpherePlaneAbove) {
    Sphere s{Vector3(0, 5.0f, 0), 1.0f};
    Plane p{kVector3UnitY, 0.0f};
    auto r = generateSpherePlane(s, p);
    EXPECT_FALSE(r.colliding);
}

// --- Sphere-AABB ---

TEST(Generator, SphereAABBOverlap) {
    Sphere s{Vector3(2, 0, 0), 1.5f};
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    auto r = generateSphereAABB(s, a, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_EQ(r.manifold.contactCount, 1u);
}

TEST(Generator, SphereAABBInside) {
    Sphere s{Vector3(0, 0, 0), 0.5f};
    AABB a{Vector3(-2, -2, -2), Vector3(2, 2, 2)};
    auto r = generateSphereAABB(s, a);
    EXPECT_TRUE(r.colliding);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
}

TEST(Generator, SphereAABBSeparated) {
    Sphere s{Vector3(10, 0, 0), 1.0f};
    AABB a{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    auto r = generateSphereAABB(s, a);
    EXPECT_FALSE(r.colliding);
}

// --- Sphere-Capsule ---

TEST(Generator, SphereCapsuleOverlap) {
    Sphere s{Vector3(0, 0, 0), 1.0f};
    Capsule c{Vector3(1, 0, 0), Vector3(1, 4, 0), 0.5f};
    auto r = generateSphereCapsule(s, c, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_EQ(r.manifold.contacts[0].featureB.type, FeatureType::Edge);
}

TEST(Generator, SphereCapsuleSeparated) {
    Sphere s{Vector3(10, 0, 0), 1.0f};
    Capsule c{Vector3(0, 0, 0), Vector3(0, 2, 0), 0.5f};
    auto r = generateSphereCapsule(s, c);
    EXPECT_FALSE(r.colliding);
}

// --- AABB-AABB ---

TEST(Generator, AABBAABBOverlap) {
    AABB a{Vector3(-1, -1, -1), Vector3(1, 1, 1)};
    AABB b{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    auto r = generateAABBAABB(a, b, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_GE(r.manifold.contactCount, 1u);
    EXPECT_LE(r.manifold.contactCount, kMaxContactPoints);
    EXPECT_EQ(r.manifold.bodyIDA, 1u);
    EXPECT_EQ(r.manifold.bodyIDB, 2u);
}

TEST(Generator, AABBAABBSeparated) {
    AABB a{Vector3(0, 0, 0), Vector3(1, 1, 1)};
    AABB b{Vector3(5, 5, 5), Vector3(6, 6, 6)};
    auto r = generateAABBAABB(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Generator, AABBAABBNested) {
    AABB a{Vector3(-1, -1, -1), Vector3(1, 1, 1)};
    AABB b{Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f)};
    auto r = generateAABBAABB(a, b);
    EXPECT_TRUE(r.colliding);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
}

TEST(Generator, AABBAABBPointsHaveFeatures) {
    AABB a{Vector3(-1, -1, -1), Vector3(1, 1, 1)};
    AABB b{Vector3(0, 0, 0), Vector3(2, 2, 2)};
    auto r = generateAABBAABB(a, b);
    for (u32 i = 0; i < r.manifold.contactCount; ++i) {
        EXPECT_NE(r.manifold.contacts[i].contactID, 0u);
        EXPECT_EQ(r.manifold.contacts[i].featureA.type, FeatureType::Face);
    }
}

// --- Capsule-Capsule ---

TEST(Generator, CapsuleCapsuleOverlap) {
    Capsule a{Vector3(0, 0, 0), Vector3(2, 0, 0), 0.5f};
    Capsule b{Vector3(1, 0.5f, 0), Vector3(3, 0.5f, 0), 0.5f};
    auto r = generateCapsuleCapsule(a, b, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_EQ(r.manifold.contactCount, 1u);
    EXPECT_GT(r.manifold.contacts[0].penetration, 0.0f);
}

TEST(Generator, CapsuleCapsuleSeparated) {
    Capsule a{Vector3(0, 0, 0), Vector3(2, 0, 0), 0.5f};
    Capsule b{Vector3(0, 5, 0), Vector3(2, 5, 0), 0.5f};
    auto r = generateCapsuleCapsule(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Generator, CapsuleCapsuleParallel) {
    Capsule a{Vector3(0, 0, 0), Vector3(4, 0, 0), 0.5f};
    Capsule b{Vector3(0, 0.8f, 0), Vector3(4, 0.8f, 0), 0.5f};
    auto r = generateCapsuleCapsule(a, b);
    EXPECT_TRUE(r.colliding);
}

// --- OBB-OBB ---

TEST(Generator, OBBOBBOverlap) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion::identity()};
    OBB b{Vector3(1.5f, 0, 0), Vector3(1, 1, 1), Quaternion::identity()};
    auto r = generateOBBOBB(a, b, 1, 2);
    EXPECT_TRUE(r.colliding);
    EXPECT_GE(r.manifold.contactCount, 1u);
    EXPECT_EQ(r.manifold.bodyIDA, 1u);
    EXPECT_EQ(r.manifold.bodyIDB, 2u);
}

TEST(Generator, OBBOBBSeparated) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion::identity()};
    OBB b{Vector3(5, 0, 0), Vector3(1, 1, 1), Quaternion::identity()};
    auto r = generateOBBOBB(a, b);
    EXPECT_FALSE(r.colliding);
}

TEST(Generator, OBBOBBRotated) {
    OBB a{Vector3(0, 0, 0), Vector3(1, 1, 1), Quaternion::identity()};
    Quaternion rot = Quaternion::fromAxisAngle(kVector3UnitY, kPi / 4.0f);
    OBB b{Vector3(1.5f, 0, 0), Vector3(1, 1, 1), rot};
    auto r = generateOBBOBB(a, b);
    EXPECT_TRUE(r.colliding);
}

// ═══════════════════════════════════════════════════════════════════════════
// Contact Persistence
// ═══════════════════════════════════════════════════════════════════════════

TEST(Persistence, MatchContact) {
    ContactPoint cached;
    cached.featureA = ContactFeature(FeatureType::Face, 1, 0);
    cached.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    cached.computeID();
    cached.normalImpulse = 5.0f;
    cached.frictionImpulseU = 2.0f;
    cached.frictionImpulseV = 1.0f;

    ContactPoint fresh;
    fresh.featureA = cached.featureA;
    fresh.featureB = cached.featureB;
    fresh.computeID();

    bool matched = matchContact(fresh, &cached, 1);
    EXPECT_TRUE(matched);
    EXPECT_FLOAT_EQ(fresh.normalImpulse, 5.0f);
    EXPECT_FLOAT_EQ(fresh.frictionImpulseU, 2.0f);
    EXPECT_FLOAT_EQ(fresh.frictionImpulseV, 1.0f);
}

TEST(Persistence, MatchContactNoMatch) {
    ContactPoint cached;
    cached.featureA = ContactFeature(FeatureType::Face, 1, 0);
    cached.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    cached.computeID();
    cached.normalImpulse = 5.0f;

    ContactPoint fresh;
    fresh.featureA = ContactFeature(FeatureType::Edge, 3, 0);
    fresh.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    fresh.computeID();

    bool matched = matchContact(fresh, &cached, 1);
    EXPECT_FALSE(matched);
    EXPECT_FLOAT_EQ(fresh.normalImpulse, 0.0f);
}

TEST(Persistence, MatchContactZeroID) {
    ContactPoint cached;
    cached.featureA = ContactFeature(FeatureType::Face, 1, 0);
    cached.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    cached.computeID();
    cached.normalImpulse = 5.0f;

    ContactPoint fresh; // default features -> ID = 0
    bool matched = matchContact(fresh, &cached, 1);
    EXPECT_FALSE(matched);
}

TEST(Persistence, UpdateManifold) {
    // Cached: 2 contacts with impulses
    ContactManifold cached;
    cached.normal = kVector3UnitY;
    ContactPoint c1;
    c1.featureA = ContactFeature(FeatureType::Face, 1, 0);
    c1.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    c1.computeID();
    c1.normalImpulse = 10.0f;
    c1.penetration = 0.3f;
    cached.addContact(c1);

    ContactPoint c2;
    c2.featureA = ContactFeature(FeatureType::Face, 2, 0);
    c2.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    c2.computeID();
    c2.normalImpulse = 8.0f;
    c2.penetration = 0.5f;
    cached.addContact(c2);

    // Fresh: same ID for c1, new contact c3
    ContactManifold fresh;
    fresh.normal = kVector3UnitY;

    ContactPoint f1;
    f1.featureA = c1.featureA;
    f1.featureB = c1.featureB;
    f1.computeID();
    f1.penetration = 0.4f; // deeper than cached

    ContactPoint f2;
    f2.featureA = ContactFeature(FeatureType::Face, 5, 0);
    f2.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
    f2.computeID();
    f2.penetration = 0.2f;
    fresh.addContact(f1);
    fresh.addContact(f2);

    ContactManifold output;
    updateManifold(output, cached, fresh);
    EXPECT_EQ(output.contactCount, 2u);

    // First contact: matched, should have cached impulse
    bool found = false;
    for (u32 i = 0; i < output.contactCount; ++i) {
        if (output.contacts[i].contactID == c1.contactID) {
            EXPECT_FLOAT_EQ(output.contacts[i].normalImpulse, 10.0f);
            found = true;
        }
    }
    EXPECT_TRUE(found);

    // Second contact: new, should have zero impulse
    bool foundNew = false;
    for (u32 i = 0; i < output.contactCount; ++i) {
        if (output.contacts[i].contactID == f2.contactID) {
            EXPECT_FLOAT_EQ(output.contacts[i].normalImpulse, 0.0f);
            foundNew = true;
        }
    }
    EXPECT_TRUE(foundNew);
}

TEST(Persistence, PruneDegenerateContacts) {
    ContactManifold m;
    ContactPoint good; good.penetration = 0.5f;
    ContactPoint degenerate; degenerate.penetration = -0.01f;
    m.addContact(good);
    m.addContact(degenerate);
    EXPECT_EQ(m.contactCount, 2u);

    pruneDegenerateContacts(m);
    EXPECT_EQ(m.contactCount, 1u);
    EXPECT_FLOAT_EQ(m.contacts[0].penetration, 0.5f);
}

TEST(Persistence, RemoveRedundantContacts) {
    ContactManifold m;
    ContactPoint a;
    a.point = Vector3(0, 0, 0);
    a.penetration = 0.3f;
    ContactPoint b;
    b.point = Vector3(0.001f, 0, 0); // very close
    b.penetration = 0.5f; // deeper
    m.addContact(a);
    m.addContact(b);
    removeRedundantContacts(m);
    EXPECT_EQ(m.contactCount, 1u);
    // Should keep the deeper one
    EXPECT_FLOAT_EQ(m.contacts[0].penetration, 0.5f);
}

TEST(Persistence, SortByPenetration) {
    ContactManifold m;
    ContactPoint a; a.penetration = 0.1f;
    ContactPoint b; b.penetration = 0.5f;
    ContactPoint c; c.penetration = 0.3f;
    m.addContact(a);
    m.addContact(b);
    m.addContact(c);
    sortByPenetration(m);
    EXPECT_FLOAT_EQ(m.contacts[0].penetration, 0.5f);
    EXPECT_FLOAT_EQ(m.contacts[1].penetration, 0.3f);
    EXPECT_FLOAT_EQ(m.contacts[2].penetration, 0.1f);
}

TEST(Persistence, FinalizeManifold) {
    ContactManifold m;
    ContactPoint degenerate; degenerate.penetration = -0.01f;
    ContactPoint good1; good1.penetration = 0.3f;
    ContactPoint good1dup; good1dup.point = Vector3(0.001f, 0, 0); good1dup.penetration = 0.5f;
    ContactPoint good2; good2.penetration = 0.1f;
    m.addContact(degenerate);
    m.addContact(good1);
    m.addContact(good1dup);
    m.addContact(good2);

    finalizeManifold(m);
    // degenerate removed, good1/good1dup merged, good2 kept -> 2 contacts
    EXPECT_LE(m.contactCount, 3u);
    // Should be sorted deepest first
    for (u32 i = 1; i < m.contactCount; ++i) {
        EXPECT_LE(m.contacts[i].penetration, m.contacts[i - 1].penetration);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Contact Constraints
// ═══════════════════════════════════════════════════════════════════════════

TEST(ContactConstraint, Defaults) {
    ContactConstraint cc;
    EXPECT_EQ(cc.bodyIDA, 0u);
    EXPECT_EQ(cc.bodyIDB, 0u);
    EXPECT_FLOAT_EQ(cc.restitution, 0.0f);
    EXPECT_FLOAT_EQ(cc.friction, 0.0f);
    EXPECT_FLOAT_EQ(cc.slop, 0.005f);
    EXPECT_FLOAT_EQ(cc.baumgarte, 0.2f);
    EXPECT_FLOAT_EQ(cc.accumulatedNormalImpulse, 0.0f);
}

TEST(ContactConstraint, RelativeVelocity) {
    ContactConstraint cc;
    cc.pointA = Vector3(1, 0, 0);
    cc.pointB = Vector3(-1, 0, 0);
    cc.normal = Vector3(1, 0, 0);

    // A moving toward B, B stationary
    Vector3 vA(-1, 0, 0);
    Vector3 wA = kVector3Zero;
    Vector3 vB = kVector3Zero;
    Vector3 wB = kVector3Zero;

    f32 relVel = cc.relativeVelocity(vA, wA, vB, wB);
    EXPECT_LT(relVel, 0.0f); // approaching

    // A moving away from B
    Vector3 vA2(1, 0, 0);
    f32 relVel2 = cc.relativeVelocity(vA2, wA, vB, wB);
    EXPECT_GT(relVel2, 0.0f); // separating
}

TEST(FrictionConstraint, Defaults) {
    FrictionConstraint fc;
    EXPECT_EQ(fc.bodyIDA, 0u);
    EXPECT_FLOAT_EQ(fc.friction, 0.0f);
    EXPECT_FLOAT_EQ(fc.accumulatedFrictionImpulse, 0.0f);
}

TEST(RestitutionConstraint, Defaults) {
    RestitutionConstraint rc;
    EXPECT_FALSE(rc.active);
    EXPECT_FLOAT_EQ(rc.velocityThreshold, 1.0f);
}

TEST(RestitutionConstraint, ComputeTargetVelocity) {
    RestitutionConstraint rc;
    rc.restitution = 0.5f;
    rc.velocityThreshold = 1.0f;

    // Slow approach: no bounce
    rc.computeTargetVelocity(-0.5f);
    EXPECT_FALSE(rc.active);
    EXPECT_FLOAT_EQ(rc.targetVelocity, 0.0f);

    // Fast approach: bounce
    rc.computeTargetVelocity(-5.0f);
    EXPECT_TRUE(rc.active);
    EXPECT_NEAR(rc.targetVelocity, 2.5f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Jacobians
// ═══════════════════════════════════════════════════════════════════════════

TEST(JacobianLinear, Direction) {
    JacobianLinear jl(Vector3(1, 0, 0));
    EXPECT_EQ(jl.a, Vector3(1, 0, 0));
    EXPECT_EQ(jl.b, Vector3(-1, 0, 0));
}

TEST(JacobianAngular, CrossProduct) {
    Vector3 rA(1, 0, 0);
    Vector3 rB(0, 1, 0);
    Vector3 n(0, 0, 1);
    JacobianAngular ja(rA, rB, n);
    // rA × n = (1,0,0) × (0,0,1) = (0,-1,0)
    EXPECT_NEAR(ja.a.x, 0.0f, kEpsilon);
    EXPECT_NEAR(ja.a.y, -1.0f, kEpsilon);
    EXPECT_NEAR(ja.a.z, 0.0f, kEpsilon);
    // -(rB × n) = -((0,1,0) × (0,0,1)) = -(1,0,0) = (-1,0,0)
    EXPECT_NEAR(ja.b.x, -1.0f, kEpsilon);
    EXPECT_NEAR(ja.b.y, 0.0f, kEpsilon);
    EXPECT_NEAR(ja.b.z, 0.0f, kEpsilon);
}

TEST(Jacobian, NormalBuild) {
    Vector3 rA(0, 1, 0);
    Vector3 rB(0, -1, 0);
    Vector3 n(0, 1, 0);
    Jacobian j = Jacobian::normal(rA, rB, n);
    EXPECT_EQ(j.linear.a, n);
    EXPECT_EQ(j.linear.b, -n);
}

TEST(Jacobian, TangentBuild) {
    Vector3 rA = kVector3Zero;
    Vector3 rB = kVector3Zero;
    Vector3 t(1, 0, 0);
    Jacobian j = Jacobian::tangent(rA, rB, t);
    EXPECT_EQ(j.linear.a, t);
    EXPECT_EQ(j.linear.b, -t);
    // Cross products of zero vectors -> zero
    EXPECT_EQ(j.angular.a, kVector3Zero);
    EXPECT_EQ(j.angular.b, kVector3Zero);
}

TEST(Jacobian, EffectiveMass) {
    InverseMassData mA;
    mA.inverseMass = 1.0f;
    mA.inverseInertiaDiag = Vector3(1, 1, 1);
    InverseMassData mB;
    mB.inverseMass = 1.0f;
    mB.inverseInertiaDiag = Vector3(1, 1, 1);

    Vector3 rA = kVector3Zero;
    Vector3 rB = kVector3Zero;
    Vector3 n(0, 1, 0);
    Jacobian j = Jacobian::normal(rA, rB, n);
    f32 mass = j.effectiveMass(mA, mB);
    // K = 1/1 + 1/1 = 2, effective mass = 1/2
    EXPECT_NEAR(mass, 0.5f, kEpsilon);
}

TEST(ContactJacobianPair, FromConstraint) {
    ContactConstraint cc;
    cc.pointA = Vector3(0, 1, 0);
    cc.pointB = Vector3(0, -1, 0);
    cc.normal = Vector3(0, 1, 0);
    cc.massA.inverseMass = 1.0f;
    cc.massA.inverseInertiaDiag = Vector3(1, 1, 1);
    cc.massB.inverseMass = 1.0f;
    cc.massB.inverseInertiaDiag = Vector3(1, 1, 1);

    Vector3 tU(1, 0, 0);
    Vector3 tV(0, 0, 1);
    auto jp = ContactJacobianPair::fromConstraint(cc, tU, tV);

    EXPECT_GT(jp.normalMass, 0.0f);
    EXPECT_GT(jp.tangentUMass, 0.0f);
    EXPECT_GT(jp.tangentVMass, 0.0f);
}

TEST(ContactJacobianPair, ComputeBias) {
    ContactJacobianPair jp;
    f32 bias = jp.computeBias(0.1f, 0.005f, 0.2f, 1.0f / 60.0f);
    EXPECT_GT(bias, 0.0f);
    // bias = baumgarte/dt * max(0, penetration + slop)
    f32 expected = 0.2f / (1.0f / 60.0f) * (0.1f + 0.005f);
    EXPECT_NEAR(bias, expected, kEpsilon);
}

TEST(JacobianMatrix, AddAndGet) {
    JacobianMatrix jm;
    EXPECT_EQ(jm.count, 0u);

    Jacobian j = Jacobian::normal(kVector3Zero, kVector3Zero, kVector3UnitY);
    EXPECT_TRUE(jm.add(j, 0.5f));
    EXPECT_EQ(jm.count, 1u);
    EXPECT_FLOAT_EQ(jm.effectiveMass[0], 0.5f);
}

TEST(JacobianMatrix, MaxConstraints) {
    JacobianMatrix jm;
    Jacobian j = Jacobian::normal(kVector3Zero, kVector3Zero, kVector3UnitY);
    for (u32 i = 0; i < JacobianMatrix::kMaxConstraints; ++i) {
        EXPECT_TRUE(jm.add(j, 1.0f));
    }
    EXPECT_FALSE(jm.add(j, 1.0f)); // full
}

TEST(JacobianMatrix, VelocityError) {
    JacobianMatrix jm;
    Jacobian j = Jacobian::normal(kVector3Zero, kVector3Zero, kVector3UnitY);
    (void)jm.add(j, 1.0f);

    Vector3 vA(0, 1, 0);
    Vector3 wA = kVector3Zero;
    Vector3 vB(0, -1, 0);
    Vector3 wB = kVector3Zero;
    f32 err = jm.velocityError(0, vA, wA, vB, wB);
    // J.v = (0,1,0).(0,1,0) + 0 + (0,-1,0).(0,-1,0) + 0 = 1 + 1 = 2
    EXPECT_NEAR(err, 2.0f, kEpsilon);
}

// ═══════════════════════════════════════════════════════════════════════════
// Integration: Generate → Persist → Constrain
// ═══════════════════════════════════════════════════════════════════════════

TEST(Integration, FullPipeline) {
    // Generate contacts
    Sphere s1{Vector3(0, 0, 0), 1.0f};
    Sphere s2{Vector3(1, 0, 0), 1.0f};
    auto r = generateSphereSphere(s1, s2, 1, 2);
    ASSERT_TRUE(r.colliding);
    ASSERT_EQ(r.manifold.contactCount, 1u);

    // Simulate persistence: cached manifold with impulse history
    ContactManifold cached = r.manifold;
    cached.contacts[0].normalImpulse = 3.0f;

    // Generate fresh contacts (slightly different position)
    s2.center = Vector3(1.1f, 0, 0);
    auto r2 = generateSphereSphere(s1, s2, 1, 2);
    ASSERT_TRUE(r2.colliding);

    // Update manifold
    ContactManifold updated;
    updateManifold(updated, cached, r2.manifold);
    ASSERT_EQ(updated.contactCount, 1u);
    EXPECT_FLOAT_EQ(updated.contacts[0].normalImpulse, 3.0f); // preserved

    // Build constraint
    ContactConstraint cc;
    cc.bodyIDA = updated.bodyIDA;
    cc.bodyIDB = updated.bodyIDB;
    cc.normal = updated.normal;
    cc.pointA = updated.contacts[0].point;
    cc.pointB = updated.contacts[0].point;
    cc.penetration = updated.contacts[0].penetration;
    cc.accumulatedNormalImpulse = updated.contacts[0].normalImpulse;
    cc.massA.inverseMass = 1.0f;
    cc.massA.inverseInertiaDiag = Vector3(1, 1, 1);
    cc.massB.inverseMass = 1.0f;
    cc.massB.inverseInertiaDiag = Vector3(1, 1, 1);

    EXPECT_EQ(cc.bodyIDA, 1u);
    EXPECT_EQ(cc.bodyIDB, 2u);
    EXPECT_GT(cc.penetration, 0.0f);
    EXPECT_FLOAT_EQ(cc.accumulatedNormalImpulse, 3.0f);

    // Build Jacobian
    Vector3 tU = updated.contacts[0].tangentU;
    Vector3 tV = updated.contacts[0].tangentV;
    auto jp = ContactJacobianPair::fromConstraint(cc, tU, tV);
    EXPECT_GT(jp.normalMass, 0.0f);
}
