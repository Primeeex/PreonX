#include <benchmark/benchmark.h>
#include <primeon/math/vector/vector3.hpp>
#include <primeon/math/matrix/matrix4.hpp>
#include <primeon/math/quaternion/quaternion.hpp>
#include <primeon/geometry/intersection/intersection.hpp>
#include <primeon/geometry/distance/distance.hpp>
#include <primeon/dynamics/simulation.hpp>
#include <primeon/dynamics/energy/energy.hpp>

using namespace primeon::math;

// ── Vector3 ──────────────────────────────────────────────────────────────────

static void BM_Vector3Add(benchmark::State& state) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a + b);
    }
}
BENCHMARK(BM_Vector3Add);

static void BM_Vector3Dot(benchmark::State& state) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.dot(b));
    }
}
BENCHMARK(BM_Vector3Dot);

static void BM_Vector3Cross(benchmark::State& state) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.cross(b));
    }
}
BENCHMARK(BM_Vector3Cross);

static void BM_Vector3Normalize(benchmark::State& state) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.normalized());
    }
}
BENCHMARK(BM_Vector3Normalize);

static void BM_Vector3Length(benchmark::State& state) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a.length());
    }
}
BENCHMARK(BM_Vector3Length);

// ── Matrix4 ──────────────────────────────────────────────────────────────────

static void BM_Matrix4Multiply(benchmark::State& state) {
    Matrix4 a = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::rotationXYZ(0.5f, 0.3f, 0.1f);
    Matrix4 b = Matrix4::scale(2.0f, 3.0f, 4.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a * b);
    }
}
BENCHMARK(BM_Matrix4Multiply);

static void BM_Matrix4Inverse(benchmark::State& state) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::scale(2.0f, 3.0f, 4.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(m.inverse());
    }
}
BENCHMARK(BM_Matrix4Inverse);

static void BM_Matrix4TransformPoint(benchmark::State& state) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::rotationXYZ(0.5f, 0.3f, 0.1f);
    Vector3 p(1.0f, 2.0f, 3.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(m.transformPoint(p));
    }
}
BENCHMARK(BM_Matrix4TransformPoint);

static void BM_Matrix4Determinant(benchmark::State& state) {
    Matrix4 m = Matrix4::translation(1.0f, 2.0f, 3.0f) * Matrix4::rotationXYZ(0.5f, 0.3f, 0.1f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(m.determinant());
    }
}
BENCHMARK(BM_Matrix4Determinant);

// ── Quaternion ───────────────────────────────────────────────────────────────

static void BM_QuaternionRotate(benchmark::State& state) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3(1.0f, 1.0f, 0.0f).normalized(), 0.5f);
    Vector3 v(1.0f, 2.0f, 3.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(q.rotate(v));
    }
}
BENCHMARK(BM_QuaternionRotate);

static void BM_QuaternionMultiply(benchmark::State& state) {
    Quaternion a = Quaternion::fromAxisAngle(kVector3UnitY, 0.5f);
    Quaternion b = Quaternion::fromAxisAngle(kVector3UnitX, 0.3f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(a * b);
    }
}
BENCHMARK(BM_QuaternionMultiply);

static void BM_QuaternionSlerp(benchmark::State& state) {
    Quaternion a = Quaternion::identity();
    Quaternion b = Quaternion::fromAxisAngle(kVector3UnitY, kPi);
    for (auto _ : state) {
        benchmark::DoNotOptimize(slerpTo(a, b, 0.5f));
    }
}
BENCHMARK(BM_QuaternionSlerp);

static void BM_QuaternionToMatrix(benchmark::State& state) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3(1.0f, 1.0f, 0.0f).normalized(), 0.5f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(q.toMatrix4());
    }
}
BENCHMARK(BM_QuaternionToMatrix);

// ── Intersection ─────────────────────────────────────────────────────────────

static void BM_RaySphere(benchmark::State& state) {
    Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Sphere s({5.0f, 0.0f, 0.0f}, 2.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(intersect::raySphere(r, s));
    }
}
BENCHMARK(BM_RaySphere);

static void BM_RayAABB(benchmark::State& state) {
    Ray r({-5.0f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB aabb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(intersect::rayAABB(r, aabb));
    }
}
BENCHMARK(BM_RayAABB);

static void BM_SphereSphere(benchmark::State& state) {
    Sphere a({0.0f, 0.0f, 0.0f}, 2.0f);
    Sphere b({3.0f, 0.0f, 0.0f}, 2.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(intersect::sphereSphere(a, b));
    }
}
BENCHMARK(BM_SphereSphere);

static void BM_AABB_AABB(benchmark::State& state) {
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    AABB b({1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(intersect::aabbAABB(a, b));
    }
}
BENCHMARK(BM_AABB_AABB);

// ── Distance ─────────────────────────────────────────────────────────────────

static void BM_PointSegmentDist(benchmark::State& state) {
    Vector3 point(0.5f, 1.0f, 0.0f);
    Vector3 segStart(0.0f, 0.0f, 0.0f);
    Vector3 segEnd(1.0f, 0.0f, 0.0f);
    for (auto _ : state) {
        benchmark::DoNotOptimize(dist::pointSegmentSq(point, segStart, segEnd));
    }
}
BENCHMARK(BM_PointSegmentDist);

static void BM_PointAABBDist(benchmark::State& state) {
    Vector3 point(3.0f, 3.0f, 3.0f);
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    for (auto _ : state) {
        benchmark::DoNotOptimize(dist::pointAABBSq(point, aabb));
    }
}
BENCHMARK(BM_PointAABBDist);

// ── Dynamics: Force Accumulation ─────────────────────────────────────────────

static void BM_ForceAccumulator_AddForce(benchmark::State& state) {
    ForceAccumulator acc;
    Vector3 f(1.0f, -9.8f, 0.0f);
    for (auto _ : state) {
        acc.clear();
        acc.addForce(f);
        acc.addForce(Vector3(0.0f, 5.0f, 0.0f));
        acc.addTorque(Vector3(1.0f, 0.0f, 0.0f));
        benchmark::DoNotOptimize(acc.force());
    }
}
BENCHMARK(BM_ForceAccumulator_AddForce);

// ── Dynamics: Particle Integrators ───────────────────────────────────────────

static void BM_Integrator_ExplicitEuler(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    for (auto _ : state) {
        s = integrateExplicitEuler(s, forces, mass, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_Integrator_ExplicitEuler);

static void BM_Integrator_SemiImplicitEuler(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    for (auto _ : state) {
        s = integrateSemiImplicitEuler(s, forces, mass, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_Integrator_SemiImplicitEuler);

static void BM_Integrator_Verlet(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 10.0f, 0.0f));
    for (auto _ : state) {
        s = integrateVerlet(s, forces, mass, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_Integrator_Verlet);

static void BM_Integrator_VelocityVerlet(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    for (auto _ : state) {
        s = integrateVelocityVerlet(s, forces, mass, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_Integrator_VelocityVerlet);

static void BM_Integrator_RK4(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    for (auto _ : state) {
        s = integrateRK4(s, forces, mass, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_Integrator_RK4);

// ── Dynamics: Body Integrators ───────────────────────────────────────────────

static void BM_BodySemiImplicitEuler(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 0.5f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    forces.addTorque(Vector3(0.0f, 0.0f, 1.0f));
    BodyState s;
    s.position = Vector3(0.0f, 10.0f, 0.0f);
    for (auto _ : state) {
        s = integrateBodySemiImplicitEuler(s, forces, mass, inertia, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_BodySemiImplicitEuler);

static void BM_BodyRK4(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 0.5f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    forces.addTorque(Vector3(0.0f, 0.0f, 1.0f));
    BodyState s;
    s.position = Vector3(0.0f, 10.0f, 0.0f);
    for (auto _ : state) {
        s = integrateBodyRK4(s, forces, mass, inertia, 0.016f);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_BodyRK4);

// ── Dynamics: Simulation Dispatch ────────────────────────────────────────────

static void BM_SimulateParticle(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    ParticleState s(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    for (auto _ : state) {
        s = simulateParticle(s, forces, mass, 0.016f, IntegratorType::SemiImplicitEuler);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_SimulateParticle);

static void BM_SimulateBody(benchmark::State& state) {
    MassProperties mass = MassProperties::fromMass(1.0f);
    InertiaTensor inertia = InertiaTensor::solidSphere(1.0f, 0.5f);
    ForceAccumulator forces;
    forces.addForce(Vector3(0.0f, -9.80665f, 0.0f));
    BodyState s;
    s.position = Vector3(0.0f, 10.0f, 0.0f);
    for (auto _ : state) {
        s = simulateBody(s, forces, mass, inertia, 0.016f, IntegratorType::SemiImplicitEuler);
        benchmark::DoNotOptimize(s);
    }
}
BENCHMARK(BM_SimulateBody);

// ── Dynamics: Quaternion Integration ─────────────────────────────────────────

static void BM_IntegrateQuaternion(benchmark::State& state) {
    Quaternion q = Quaternion::identity();
    Vector3 omega(0.0f, 3.14f, 0.0f);
    for (auto _ : state) {
        q = integrateQuaternion(q, omega, 0.016f);
        benchmark::DoNotOptimize(q);
    }
}
BENCHMARK(BM_IntegrateQuaternion);

// ── Dynamics: Energy Computation ─────────────────────────────────────────────

static void BM_ComputeKineticEnergy(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(kineticEnergy(1.0f, Vector3(3.0f, 4.0f, 5.0f)));
    }
}
BENCHMARK(BM_ComputeKineticEnergy);

static void BM_ComputeTotalEnergy(benchmark::State& state) {
    for (auto _ : state) {
        benchmark::DoNotOptimize(computeTotalEnergy(1.0f, Vector3(0.0f, 10.0f, 0.0f),
                                                     Vector3(3.0f, 4.0f, 5.0f)));
    }
}
BENCHMARK(BM_ComputeTotalEnergy);
