#include <gtest/gtest.h>
#include <primeon/geometry/primitives/aabb.hpp>
#include <primeon/geometry/primitives/sphere.hpp>
#include <primeon/geometry/primitives/capsule.hpp>
#include <primeon/geometry/primitives/triangle.hpp>
#include <primeon/geometry/primitives/obb.hpp>
#include <primeon/geometry/primitives/plane.hpp>
#include <primeon/geometry/primitives/ray.hpp>
#include <primeon/geometry/primitives/segment.hpp>

using namespace primeon::math;

TEST(AABB, Center) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 4.0f, 6.0f});
    Vector3 c = aabb.center();
    EXPECT_NEAR(c.x, 1.0f, kEpsilon);
    EXPECT_NEAR(c.y, 2.0f, kEpsilon);
    EXPECT_NEAR(c.z, 3.0f, kEpsilon);
}

TEST(AABB, HalfExtents) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 4.0f, 6.0f});
    Vector3 he = aabb.halfExtents();
    EXPECT_NEAR(he.x, 1.0f, kEpsilon);
    EXPECT_NEAR(he.y, 2.0f, kEpsilon);
    EXPECT_NEAR(he.z, 3.0f, kEpsilon);
}

TEST(AABB, ContainsPoint) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    EXPECT_TRUE(aabb.containsPoint({1.0f, 1.0f, 1.0f}));
    EXPECT_TRUE(aabb.containsPoint({0.0f, 0.0f, 0.0f}));
    EXPECT_TRUE(aabb.containsPoint({2.0f, 2.0f, 2.0f}));
    EXPECT_FALSE(aabb.containsPoint({3.0f, 1.0f, 1.0f}));
    EXPECT_FALSE(aabb.containsPoint({-1.0f, 1.0f, 1.0f}));
}

TEST(AABB, ClosestPoint) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    Vector3 cp = aabb.closestPoint({3.0f, 1.0f, 1.0f});
    EXPECT_NEAR(cp.x, 2.0f, kEpsilon);
    EXPECT_NEAR(cp.y, 1.0f, kEpsilon);
    EXPECT_NEAR(cp.z, 1.0f, kEpsilon);
}

TEST(AABB, Volume) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 3.0f, 4.0f});
    EXPECT_NEAR(aabb.volume(), 24.0f, kEpsilon);
}

TEST(AABB, SurfaceArea) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 3.0f, 4.0f});
    EXPECT_NEAR(aabb.surfaceArea(), 52.0f, kEpsilon);
}

TEST(AABB, ExpandToIncludePoint) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    aabb.expandToInclude({2.0f, 3.0f, 4.0f});
    EXPECT_NEAR(aabb.max.x, 2.0f, kEpsilon);
    EXPECT_NEAR(aabb.max.y, 3.0f, kEpsilon);
    EXPECT_NEAR(aabb.max.z, 4.0f, kEpsilon);
}

TEST(AABB, Merged) {
    AABB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    AABB b({0.5f, 0.5f, 0.5f}, {2.0f, 2.0f, 2.0f});
    AABB m = a.merged(b);
    EXPECT_NEAR(m.min.x, 0.0f, kEpsilon);
    EXPECT_NEAR(m.min.y, 0.0f, kEpsilon);
    EXPECT_NEAR(m.min.z, 0.0f, kEpsilon);
    EXPECT_NEAR(m.max.x, 2.0f, kEpsilon);
    EXPECT_NEAR(m.max.y, 2.0f, kEpsilon);
    EXPECT_NEAR(m.max.z, 2.0f, kEpsilon);
}

TEST(AABB, FromCenterHalfExtents) {
    AABB aabb = AABB::fromCenterHalfExtents({5.0f, 5.0f, 5.0f}, {1.0f, 2.0f, 3.0f});
    EXPECT_NEAR(aabb.min.x, 4.0f, kEpsilon);
    EXPECT_NEAR(aabb.max.x, 6.0f, kEpsilon);
    EXPECT_NEAR(aabb.min.y, 3.0f, kEpsilon);
    EXPECT_NEAR(aabb.max.y, 7.0f, kEpsilon);
    EXPECT_NEAR(aabb.min.z, 2.0f, kEpsilon);
    EXPECT_NEAR(aabb.max.z, 8.0f, kEpsilon);
}

TEST(Sphere, ContainsPoint) {
    Sphere s({0.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_TRUE(s.containsPoint({1.0f, 0.0f, 0.0f}));
    EXPECT_TRUE(s.containsPoint({2.0f, 0.0f, 0.0f}));
    EXPECT_FALSE(s.containsPoint({3.0f, 0.0f, 0.0f}));
}

TEST(Sphere, RadiusSq) {
    Sphere s({0.0f, 0.0f, 0.0f}, 3.0f);
    EXPECT_NEAR(s.radiusSq(), 9.0f, kEpsilon);
}

TEST(Capsule, Length) {
    Capsule c({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, 1.0f);
    EXPECT_NEAR(c.length(), 3.0f, kEpsilon);
}

TEST(Capsule, Height) {
    Capsule c({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, 1.0f);
    EXPECT_NEAR(c.height(), 5.0f, kEpsilon);
}

TEST(Capsule, Midpoint) {
    Capsule c({0.0f, 0.0f, 0.0f}, {0.0f, 4.0f, 0.0f}, 1.0f);
    Vector3 mp = c.midpoint();
    EXPECT_NEAR(mp.x, 0.0f, kEpsilon);
    EXPECT_NEAR(mp.y, 2.0f, kEpsilon);
    EXPECT_NEAR(mp.z, 0.0f, kEpsilon);
}

TEST(Triangle, Normal) {
    Triangle t({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Vector3 n = t.normal();
    EXPECT_NEAR(n.x, 0.0f, kEpsilon);
    EXPECT_NEAR(n.y, 0.0f, kEpsilon);
    EXPECT_NEAR(n.z, 1.0f, kEpsilon);
}

TEST(Triangle, Area) {
    Triangle t({0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f});
    EXPECT_NEAR(t.area(), 2.0f, kEpsilon);
}

TEST(Triangle, Centroid) {
    Triangle t({0.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f});
    Vector3 c = t.centroid();
    EXPECT_NEAR(c.x, 1.0f, kEpsilon);
    EXPECT_NEAR(c.y, 1.0f, kEpsilon);
    EXPECT_NEAR(c.z, 0.0f, kEpsilon);
}

TEST(Triangle, Barycentric) {
    Triangle t({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Vector3 p = t.pointAtBarycentric(0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(p.x, 0.0f, kEpsilon);
    EXPECT_NEAR(p.y, 1.0f, kEpsilon);
    EXPECT_NEAR(p.z, 0.0f, kEpsilon);
}

TEST(OBB, Volume) {
    OBB obb({0.0f, 0.0f, 0.0f}, {1.0f, 2.0f, 3.0f}, Quaternion::identity());
    EXPECT_NEAR(obb.volume(), 48.0f, kEpsilon);
}

TEST(OBB, GetCorners) {
    OBB obb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f}, Quaternion::identity());
    Vector3 corners[8];
    obb.getCorners(corners);
    // Each corner should be at distance sqrt(3) from center
    for (int i = 0; i < 8; ++i) {
        EXPECT_NEAR(corners[i].length(), std::sqrt(3.0f), kEpsilon);
    }
}

TEST(Plane, DistanceToPoint) {
    Plane p(kVector3UnitY, 0.0f);
    EXPECT_NEAR(p.distanceToPoint({0.0f, 5.0f, 0.0f}), 5.0f, kEpsilon);
    EXPECT_NEAR(p.distanceToPoint({0.0f, -3.0f, 0.0f}), -3.0f, kEpsilon);
}

TEST(Plane, FromPointNormal) {
    Plane p = Plane::fromPointNormal({0.0f, 5.0f, 0.0f}, kVector3UnitY);
    EXPECT_NEAR(p.distance, 5.0f, kEpsilon);
    EXPECT_NEAR(p.normal.y, 1.0f, kEpsilon);
}

TEST(Plane, FromThreePoints) {
    Plane p = Plane::fromThreePoints(
        {0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f}
    );
    EXPECT_NEAR(p.normal.y, 1.0f, kEpsilon);
    EXPECT_NEAR(p.distance, 0.0f, kEpsilon);
}

TEST(Plane, ClosestPoint) {
    Plane p(kVector3UnitY, 0.0f);
    Vector3 cp = p.closestPoint({3.0f, 5.0f, 7.0f});
    EXPECT_NEAR(cp.x, 3.0f, kEpsilon);
    EXPECT_NEAR(cp.y, 0.0f, kEpsilon);
    EXPECT_NEAR(cp.z, 7.0f, kEpsilon);
}

TEST(Ray, PointAt) {
    Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Vector3 p = r.pointAt(5.0f);
    EXPECT_NEAR(p.x, 5.0f, kEpsilon);
    EXPECT_NEAR(p.y, 0.0f, kEpsilon);
    EXPECT_NEAR(p.z, 0.0f, kEpsilon);
}

TEST(Segment, Direction) {
    Segment s({0.0f, 0.0f, 0.0f}, {3.0f, 4.0f, 0.0f});
    Vector3 d = s.direction();
    EXPECT_NEAR(d.x, 3.0f, kEpsilon);
    EXPECT_NEAR(d.y, 4.0f, kEpsilon);
}

TEST(Segment, Length) {
    Segment s({0.0f, 0.0f, 0.0f}, {3.0f, 4.0f, 0.0f});
    EXPECT_NEAR(s.length(), 5.0f, kEpsilon);
}

TEST(Segment, Midpoint) {
    Segment s({0.0f, 0.0f, 0.0f}, {4.0f, 6.0f, 0.0f});
    Vector3 mp = s.midpoint();
    EXPECT_NEAR(mp.x, 2.0f, kEpsilon);
    EXPECT_NEAR(mp.y, 3.0f, kEpsilon);
    EXPECT_NEAR(mp.z, 0.0f, kEpsilon);
}

TEST(Segment, PointAt) {
    Segment s({0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f});
    Vector3 p = s.pointAt(0.25f);
    EXPECT_NEAR(p.x, 2.5f, kEpsilon);
}
