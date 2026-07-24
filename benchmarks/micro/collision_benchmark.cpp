#include <benchmark/benchmark.h>
#include <primeon/collision/narrowphase/sphere_sphere.hpp>
#include <primeon/collision/narrowphase/sphere_plane.hpp>
#include <primeon/collision/narrowphase/sphere_aabb.hpp>
#include <primeon/collision/narrowphase/sphere_capsule.hpp>
#include <primeon/collision/narrowphase/aabb_aabb.hpp>
#include <primeon/collision/narrowphase/obb_obb.hpp>
#include <primeon/collision/narrowphase/capsule_capsule.hpp>
#include <primeon/collision/narrowphase/raycast.hpp>
#include <primeon/collision/algorithms/sat.hpp>
#include <primeon/collision/algorithms/gjk.hpp>
#include <primeon/collision/algorithms/epa.hpp>
#include <primeon/collision/broadphase/sweep_and_prune.hpp>
#include <primeon/collision/broadphase/dynamic_aabb_tree.hpp>
#include <primeon/collision/ccd/tof.hpp>
#include <primeon/collision/queries/collision_query.hpp>
#include <primeon/collision/manifold/contact_generator.hpp>
#include <primeon/collision/manifold/contact_persistence.hpp>
#include <primeon/constraints/contact_constraint.hpp>
#include <primeon/constraints/jacobian.hpp>

using namespace primeon::math;
using namespace primeon::collision;

// ── Narrowphase: Sphere-Sphere ──────────────────────────────────────────────

static void BM_Narrowphase_SphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({1.5f, 0.0f, 0.0f}, 1.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(sphereSphere(a, b));
    }
}
BENCHMARK(BM_Narrowphase_SphereSphere);

// ── Narrowphase: Sphere-AABB ────────────────────────────────────────────────

static void BM_Narrowphase_SphereAABB(benchmark::State& state) {
    Sphere s({0.5f, 0.5f, 0.5f}, 1.0f);
    AABB b({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(sphereAABB(s, b));
    }
}
BENCHMARK(BM_Narrowphase_SphereAABB);

// ── Narrowphase: AABB-AABB ─────────────────────────────────────────────────

static void BM_Narrowphase_AABBAABB(benchmark::State& state) {
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    AABB b({1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(aabbAABB(a, b));
    }
}
BENCHMARK(BM_Narrowphase_AABBAABB);

// ── Narrowphase: OBB-OBB ───────────────────────────────────────────────────

static void BM_Narrowphase_OBBOBB(benchmark::State& state) {
    OBB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, Quaternion::identity());
    OBB b({0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f},
          Quaternion::fromAxisAngle(kVector3UnitZ, 0.3f));
    for (auto _ : state) {
        benchmark::DoNotOptimize(obbOBB(a, b));
    }
}
BENCHMARK(BM_Narrowphase_OBBOBB);

// ── Narrowphase: Capsule-Capsule ────────────────────────────────────────────

static void BM_Narrowphase_CapsuleCapsule(benchmark::State& state) {
    Capsule a({0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 0.5f);
    Capsule b({1.0f, 0.5f, 0.0f}, {3.0f, 0.5f, 0.0f}, 0.5f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(capsuleCapsule(a, b));
    }
}
BENCHMARK(BM_Narrowphase_CapsuleCapsule);

// ── Narrowphase: Sphere-Plane ───────────────────────────────────────────────

static void BM_Narrowphase_SpherePlane(benchmark::State& state) {
    Sphere s({0.0f, 0.5f, 0.0f}, 1.0f);
    Plane p({0.0f, 1.0f, 0.0f}, 0.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(spherePlane(s, p));
    }
}
BENCHMARK(BM_Narrowphase_SpherePlane);

// ── Narrowphase: Sphere-Capsule ─────────────────────────────────────────────

static void BM_Narrowphase_SphereCapsule(benchmark::State& state) {
    Sphere s({1.0f, 0.5f, 0.0f}, 0.5f);
    Capsule c({0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 0.5f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(sphereCapsule(s, c));
    }
}
BENCHMARK(BM_Narrowphase_SphereCapsule);

// ── SAT: OBB-OBB ───────────────────────────────────────────────────────────

static void BM_SAT_OBBOBB(benchmark::State& state) {
    OBB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, Quaternion::identity());
    OBB b({0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f},
          Quaternion::fromAxisAngle(kVector3UnitZ, 0.3f));
    for (auto _ : state) {
        Vector3 outNormal;
        benchmark::DoNotOptimize(satOBBFull(a, b, outNormal));
    }
}
BENCHMARK(BM_SAT_OBBOBB);

// ── GJK: Sphere-Sphere ─────────────────────────────────────────────────────

static void BM_GJK_SphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({1.5f, 0.0f, 0.0f}, 1.0f);
    auto support = [&](const Vector3& d) -> Vector3 {
        return supportSphere(a, d) - supportSphere(b, -d);
    };
    for (auto _ : state) {
        benchmark::DoNotOptimize(gjkIntersect(support));
    }
}
BENCHMARK(BM_GJK_SphereSphere);

// ── GJK: OBB-OBB ───────────────────────────────────────────────────────────

static void BM_GJK_OBBOBB(benchmark::State& state) {
    OBB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, Quaternion::identity());
    OBB b({0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f},
          Quaternion::fromAxisAngle(kVector3UnitZ, 0.3f));
    auto support = [&](const Vector3& d) -> Vector3 {
        return supportOBB(a, d) - supportOBB(b, -d);
    };
    for (auto _ : state) {
        benchmark::DoNotOptimize(gjkIntersect(support));
    }
}
BENCHMARK(BM_GJK_OBBOBB);

// ── EPA: Sphere-Sphere ──────────────────────────────────────────────────────

static void BM_EPA_SphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({1.5f, 0.0f, 0.0f}, 1.0f);
    auto support = [&](const Vector3& d) -> Vector3 {
        return supportSphere(a, d) - supportSphere(b, -d);
    };
    for (auto _ : state) {
        GJKSimplex simplex;
        Vector3 d(1.0f, 0.0f, 0.0f);
        simplex.add(support(d));
        d = -simplex.vertices[0];
        simplex.add(support(d));
        d = gjk_detail::solveLine(simplex);
        simplex.add(support(d));
        d = gjk_detail::solveTriangle(simplex);
        simplex.add(support(d));
        benchmark::DoNotOptimize(epa(simplex, support));
    }
}
BENCHMARK(BM_EPA_SphereSphere);

// ── Broadphase: SAP Insert + ComputePairs ───────────────────────────────────

static void BM_Broadphase_SAP_InsertAndQuery(benchmark::State& state) {
    const u32 N = 128;
    for (auto _ : state) {
        state.PauseTiming();
        primeon::collision::SAPBroadphase sap;
        for (u32 i = 0; i < N; ++i) {
            f32 x = static_cast<f32>(i) * 0.5f;
            sap.insert(i, AABB({x, 0.0f, 0.0f}, {x + 1.0f, 1.0f, 1.0f}));
        }
        state.ResumeTiming();

        sap.computePairs();
        benchmark::DoNotOptimize(sap.pairCount());
    }
}
BENCHMARK(BM_Broadphase_SAP_InsertAndQuery);

// ── Broadphase: Dynamic AABB Tree Insert + Query ────────────────────────────

static void BM_Broadphase_AABBTree_InsertAndQuery(benchmark::State& state) {
    const u32 N = 128;
    for (auto _ : state) {
        state.PauseTiming();
        primeon::collision::DynamicAABBTree tree(256);
        for (u32 i = 0; i < N; ++i) {
            f32 x = static_cast<f32>(i) * 0.5f;
            tree.insert(i, AABB({x, 0.0f, 0.0f}, {x + 1.0f, 1.0f, 1.0f}));
        }
        AABB queryBox({0.0f, 0.0f, 0.0f}, {10.0f, 10.0f, 10.0f});
        state.ResumeTiming();

        tree.queryAABB(queryBox);
        benchmark::DoNotOptimize(tree.queryResults.size());
    }
}
BENCHMARK(BM_Broadphase_AABBTree_InsertAndQuery);

// ── Broadphase: Dynamic AABB Tree Insert Only ───────────────────────────────

static void BM_Broadphase_AABBTree_Insert(benchmark::State& state) {
    const u32 N = 128;
    for (auto _ : state) {
        state.PauseTiming();
        primeon::collision::DynamicAABBTree tree(256);
        state.ResumeTiming();

        for (u32 i = 0; i < N; ++i) {
            f32 x = static_cast<f32>(i) * 0.5f;
            tree.insert(i, AABB({x, 0.0f, 0.0f}, {x + 1.0f, 1.0f, 1.0f}));
        }
        benchmark::DoNotOptimize(tree.nodeCount());
    }
}
BENCHMARK(BM_Broadphase_AABBTree_Insert);

// ── CCD: Sphere-Sphere TOI ─────────────────────────────────────────────────

static void BM_CCD_TOISphereSphere(benchmark::State& state) {
    Vector3 posA0(0.0f, 0.0f, 0.0f);
    Vector3 posA1(5.0f, 0.0f, 0.0f);
    Vector3 posB0(10.0f, 0.0f, 0.0f);
    Vector3 posB1(3.0f, 0.0f, 0.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(primeon::collision::toiSphereSphere(posA0, 1.0f, posB0, 1.0f, posA1, posB1));
    }
}
BENCHMARK(BM_CCD_TOISphereSphere);

// ── Raycast: Ray-Sphere ─────────────────────────────────────────────────────

static void BM_Raycast_RaySphere(benchmark::State& state) {
    Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Sphere s({5.0f, 0.0f, 0.0f}, 2.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(raySphere(r, s));
    }
}
BENCHMARK(BM_Raycast_RaySphere);

// ── Raycast: Ray-AABB ───────────────────────────────────────────────────────

static void BM_Raycast_RayAABB(benchmark::State& state) {
    Ray r({-5.0f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(rayAABB(r, a));
    }
}
BENCHMARK(BM_Raycast_RayAABB);

// ── Query: Intersects Sphere-Sphere ─────────────────────────────────────────

static void BM_Query_IntersectsSphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({1.5f, 0.0f, 0.0f}, 1.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(intersects(a, b));
    }
}
BENCHMARK(BM_Query_IntersectsSphereSphere);

// ── Query: Contact AABB-AABB ────────────────────────────────────────────────

static void BM_Query_ContactAABBAABB(benchmark::State& state) {
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    AABB b({1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(contact(a, b));
    }
}
BENCHMARK(BM_Query_ContactAABBAABB);

// ── Query: Raycast ──────────────────────────────────────────────────────────

static void BM_Query_RaycastAABB(benchmark::State& state) {
    Ray r({-5.0f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(raycast(r, a));
    }
}
BENCHMARK(BM_Query_RaycastAABB);

// ═══════════════════════════════════════════════════════════════════════════════
// Stage 7: Contact Generation
// ═══════════════════════════════════════════════════════════════════════════════

static void BM_Generator_SphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({1.5f, 0.0f, 0.0f}, 1.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateSphereSphere(a, b, 0, 1));
    }
}
BENCHMARK(BM_Generator_SphereSphere);

static void BM_Generator_SpherePlane(benchmark::State& state) {
    Sphere s({0.0f, 0.5f, 0.0f}, 1.0f);
    Plane p({0.0f, 1.0f, 0.0f}, 0.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateSpherePlane(s, p, 0, 1));
    }
}
BENCHMARK(BM_Generator_SpherePlane);

static void BM_Generator_SphereAABB(benchmark::State& state) {
    Sphere s({0.5f, 0.5f, 0.5f}, 1.0f);
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateSphereAABB(s, a, 0, 1));
    }
}
BENCHMARK(BM_Generator_SphereAABB);

static void BM_Generator_SphereCapsule(benchmark::State& state) {
    Sphere s({0.5f, 0.0f, 0.0f}, 1.0f);
    Capsule c({0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 0.5f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateSphereCapsule(s, c, 0, 1));
    }
}
BENCHMARK(BM_Generator_SphereCapsule);

static void BM_Generator_AABBAABB(benchmark::State& state) {
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    AABB b({1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateAABBAABB(a, b, 0, 1));
    }
}
BENCHMARK(BM_Generator_AABBAABB);

static void BM_Generator_CapsuleCapsule(benchmark::State& state) {
    Capsule a({0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, 0.5f);
    Capsule b({1.0f, 0.5f, 0.0f}, {3.0f, 0.5f, 0.0f}, 0.5f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateCapsuleCapsule(a, b, 0, 1));
    }
}
BENCHMARK(BM_Generator_CapsuleCapsule);

static void BM_Generator_OBBOBB(benchmark::State& state) {
    OBB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, Quaternion::identity());
    OBB b({0.5f, 0.5f, 0.0f}, {1.0f, 1.0f, 1.0f},
          Quaternion::fromAxisAngle(kVector3UnitZ, 0.3f));
    for (auto _ : state) {
        benchmark::DoNotOptimize(generateOBBOBB(a, b, 0, 1));
    }
}
BENCHMARK(BM_Generator_OBBOBB);

// ═══════════════════════════════════════════════════════════════════════════════
// Stage 7: Contact Persistence
// ═══════════════════════════════════════════════════════════════════════════════

static void BM_Persistence_UpdateManifold(benchmark::State& state) {
    ContactManifold cached;
    cached.normal = kVector3UnitY;
    for (u32 i = 0; i < 4; ++i) {
        ContactPoint cp;
        cp.point = Vector3(static_cast<f32>(i) * 0.1f, 0.0f, 0.0f);
        cp.penetration = 0.5f;
        cp.featureA = ContactFeature(FeatureType::Face, i, 0);
        cp.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
        cp.computeID();
        cp.normalImpulse = 3.0f;
        cached.addContact(cp);
    }

    ContactManifold fresh;
    fresh.normal = kVector3UnitY;
    for (u32 i = 0; i < 4; ++i) {
        ContactPoint cp;
        cp.point = Vector3(static_cast<f32>(i) * 0.12f, 0.0f, 0.0f);
        cp.penetration = 0.52f;
        cp.featureA = ContactFeature(FeatureType::Face, i, 0);
        cp.featureB = ContactFeature(FeatureType::Vertex, 0, 0);
        cp.computeID();
        fresh.addContact(cp);
    }

    for (auto _ : state) {
        ContactManifold output;
        updateManifold(output, cached, fresh);
        benchmark::DoNotOptimize(output.contactCount);
    }
}
BENCHMARK(BM_Persistence_UpdateManifold);

static void BM_Persistence_FinalizeManifold(benchmark::State& state) {
    for (auto _ : state) {
        ContactManifold m;
        m.normal = kVector3UnitY;
        for (u32 i = 0; i < 4; ++i) {
            ContactPoint cp;
            cp.point = Vector3(static_cast<f32>(i) * 0.05f, 0.0f, 0.0f);
            cp.penetration = 0.5f - static_cast<f32>(i) * 0.1f;
            m.addContact(cp);
        }
        finalizeManifold(m);
        benchmark::DoNotOptimize(m.contactCount);
    }
}
BENCHMARK(BM_Persistence_FinalizeManifold);

// ═══════════════════════════════════════════════════════════════════════════════
// Stage 7: Jacobian
// ═══════════════════════════════════════════════════════════════════════════════

static void BM_Jacobian_EffectiveMass(benchmark::State& state) {
    Jacobian j = Jacobian::normal({0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, kVector3UnitY);
    InverseMassData mA;
    mA.inverseMass = 1.0f;
    mA.inverseInertiaDiag = Vector3(1.0f, 1.0f, 1.0f);
    InverseMassData mB = mA;
    for (auto _ : state) {
        benchmark::DoNotOptimize(j.effectiveMass(mA, mB));
    }
}
BENCHMARK(BM_Jacobian_EffectiveMass);

static void BM_Jacobian_FromConstraint(benchmark::State& state) {
    ContactConstraint cc;
    cc.pointA = Vector3(0.0f, 1.0f, 0.0f);
    cc.pointB = Vector3(0.0f, -1.0f, 0.0f);
    cc.normal = kVector3UnitY;
    cc.massA.inverseMass = 1.0f;
    cc.massA.inverseInertiaDiag = Vector3(1.0f, 1.0f, 1.0f);
    cc.massB.inverseMass = 1.0f;
    cc.massB.inverseInertiaDiag = Vector3(1.0f, 1.0f, 1.0f);
    Vector3 tU(1.0f, 0.0f, 0.0f);
    Vector3 tV(0.0f, 0.0f, 1.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(ContactJacobianPair::fromConstraint(cc, tU, tV));
    }
}
BENCHMARK(BM_Jacobian_FromConstraint);
