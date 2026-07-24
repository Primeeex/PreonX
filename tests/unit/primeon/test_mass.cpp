#include <gtest/gtest.h>
#include <primeon/dynamics/mass/mass.hpp>

using namespace primeon::math;

TEST(MassProperties, DefaultConstruction) {
    MassProperties mp;
    EXPECT_NEAR(mp.mass, 1.0f, kEpsilon);
    EXPECT_NEAR(mp.inverseMass, 1.0f, kEpsilon);
    EXPECT_TRUE(mp.isDynamic());
}

TEST(MassProperties, FromMass) {
    MassProperties mp = MassProperties::fromMass(5.0f);
    EXPECT_NEAR(mp.mass, 5.0f, kEpsilon);
    EXPECT_NEAR(mp.inverseMass, 0.2f, kEpsilon);
    EXPECT_TRUE(mp.isDynamic());
}

TEST(MassProperties, FromZeroMass) {
    MassProperties mp = MassProperties::fromMass(0.0f);
    EXPECT_NEAR(mp.mass, 0.0f, kEpsilon);
    EXPECT_NEAR(mp.inverseMass, 0.0f, kEpsilon);
    EXPECT_TRUE(mp.isStatic());
}

TEST(MassProperties, FromNegativeMass) {
    MassProperties mp = MassProperties::fromMass(-5.0f);
    EXPECT_TRUE(mp.isStatic());
    EXPECT_NEAR(mp.mass, 0.0f, kEpsilon);
}

TEST(MassProperties, FromInverseMass) {
    MassProperties mp = MassProperties::fromInverseMass(0.25f);
    EXPECT_NEAR(mp.inverseMass, 0.25f, kEpsilon);
    EXPECT_NEAR(mp.mass, 4.0f, kEpsilon);
}

TEST(MassProperties, FromZeroInverseMass) {
    MassProperties mp = MassProperties::fromInverseMass(0.0f);
    EXPECT_TRUE(mp.isStatic());
    EXPECT_NEAR(mp.mass, 0.0f, kEpsilon);
}

TEST(MassProperties, Equality) {
    MassProperties a = MassProperties::fromMass(5.0f);
    MassProperties b = MassProperties::fromMass(5.0f);
    MassProperties c = MassProperties::fromMass(10.0f);
    EXPECT_EQ(a, b);
    EXPECT_NE(a, c);
}

TEST(MassProperties, StaticFactory) {
    MassProperties mp = makeStaticMass();
    EXPECT_TRUE(mp.isStatic());
}

TEST(MassProperties, DynamicFactory) {
    MassProperties mp = makeDynamicMass(3.0f);
    EXPECT_TRUE(mp.isDynamic());
    EXPECT_NEAR(mp.mass, 3.0f, kEpsilon);
}

TEST(InertiaTensor, DefaultIdentity) {
    InertiaTensor it;
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 result = it.apply(v);
    EXPECT_NEAR(result.x, 1.0f, kEpsilon);
    EXPECT_NEAR(result.y, 2.0f, kEpsilon);
    EXPECT_NEAR(result.z, 3.0f, kEpsilon);
}

TEST(InertiaTensor, Diagonal) {
    InertiaTensor it = InertiaTensor::diagonal(2.0f, 3.0f, 4.0f);
    Vector3 v(1.0f, 1.0f, 1.0f);
    Vector3 result = it.apply(v);
    EXPECT_NEAR(result.x, 2.0f, kEpsilon);
    EXPECT_NEAR(result.y, 3.0f, kEpsilon);
    EXPECT_NEAR(result.z, 4.0f, kEpsilon);
}

TEST(InertiaTensor, SolidSphere) {
    InertiaTensor it = InertiaTensor::solidSphere(1.0f, 1.0f);
    // I = 2/5 * m * r^2 = 0.4
    Vector3 v(1.0f, 0.0f, 0.0f);
    Vector3 result = it.apply(v);
    EXPECT_NEAR(result.x, 0.4f, kEpsilon);
}

TEST(InertiaTensor, SolidBox) {
    InertiaTensor it = InertiaTensor::solidBox(1.0f, {1.0f, 1.0f, 1.0f});
    // halfExtents = (1,1,1), full extents = (2,2,2)
    // Ixx = 1/12 * m * (dy^2 + dz^2) = 1/12 * 1 * (4+4) = 8/12 = 2/3
    Vector3 v(1.0f, 0.0f, 0.0f);
    Vector3 result = it.apply(v);
    EXPECT_NEAR(result.x, 2.0f / 3.0f, kEpsilon);
}

TEST(InertiaTensor, Inverse) {
    InertiaTensor it = InertiaTensor::diagonal(2.0f, 3.0f, 4.0f);
    InertiaTensor inv = it.inverse();
    Vector3 v(1.0f, 1.0f, 1.0f);
    Vector3 result = inv.apply(it.apply(v));
    EXPECT_NEAR(result.x, 1.0f, kEpsilon);
    EXPECT_NEAR(result.y, 1.0f, kEpsilon);
    EXPECT_NEAR(result.z, 1.0f, kEpsilon);
}
