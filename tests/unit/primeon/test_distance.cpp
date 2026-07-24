#include <gtest/gtest.h>
#include <primeon/geometry/distance/distance.hpp>

using namespace primeon::math;

TEST(Distance, PointPoint) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(3.0f, 4.0f, 0.0f);
    EXPECT_NEAR(dist::pointPoint(a, b), 5.0f, kEpsilon);
    EXPECT_NEAR(dist::pointPointSq(a, b), 25.0f, kEpsilon);
}

TEST(Distance, PointLine) {
    Vector3 point(1.0f, 1.0f, 0.0f);
    Vector3 lineOrigin(0.0f, 0.0f, 0.0f);
    Vector3 lineDir(1.0f, 0.0f, 0.0f);
    f32 d = dist::pointLineSq(point, lineOrigin, lineDir);
    EXPECT_NEAR(d, 1.0f, kEpsilon);
}

TEST(Distance, PointSegment) {
    // Point directly above the midpoint of the segment
    Vector3 point(0.5f, 1.0f, 0.0f);
    Vector3 segStart(0.0f, 0.0f, 0.0f);
    Vector3 segEnd(1.0f, 0.0f, 0.0f);
    f32 d = dist::pointSegmentSq(point, segStart, segEnd);
    EXPECT_NEAR(d, 1.0f, kEpsilon);
}

TEST(Distance, PointSegmentBeyondEnd) {
    // Point beyond the end of the segment
    Vector3 point(3.0f, 0.0f, 0.0f);
    Vector3 segStart(0.0f, 0.0f, 0.0f);
    Vector3 segEnd(1.0f, 0.0f, 0.0f);
    f32 d = dist::pointSegmentSq(point, segStart, segEnd);
    EXPECT_NEAR(d, 4.0f, kEpsilon);
}

TEST(Distance, PointSegmentBeyondStart) {
    // Point beyond the start of the segment
    Vector3 point(-2.0f, 0.0f, 0.0f);
    Vector3 segStart(0.0f, 0.0f, 0.0f);
    Vector3 segEnd(1.0f, 0.0f, 0.0f);
    f32 d = dist::pointSegmentSq(point, segStart, segEnd);
    EXPECT_NEAR(d, 4.0f, kEpsilon);
}

TEST(Distance, PointPlane) {
    Plane p(kVector3UnitY, 0.0f);
    f32 d = dist::pointPlane({0.0f, 5.0f, 0.0f}, p);
    EXPECT_NEAR(d, 5.0f, kEpsilon);
    d = dist::pointPlane({0.0f, -3.0f, 0.0f}, p);
    EXPECT_NEAR(d, -3.0f, kEpsilon);
}

TEST(Distance, PointSphere) {
    Sphere s({0.0f, 0.0f, 0.0f}, 2.0f);
    // Point outside
    f32 d = dist::pointSphereSq({5.0f, 0.0f, 0.0f}, s);
    EXPECT_NEAR(d, 9.0f, kEpsilon);
    // Point inside (distance should be 0)
    d = dist::pointSphereSq({1.0f, 0.0f, 0.0f}, s);
    EXPECT_NEAR(d, 0.0f, kEpsilon);
    // Point on surface
    d = dist::pointSphereSq({2.0f, 0.0f, 0.0f}, s);
    EXPECT_NEAR(d, 0.0f, kEpsilon);
}

TEST(Distance, PointAABB) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    // Point inside
    f32 d = dist::pointAABBSq({1.0f, 1.0f, 1.0f}, aabb);
    EXPECT_NEAR(d, 0.0f, kEpsilon);
    // Point outside
    d = dist::pointAABBSq({3.0f, 1.0f, 1.0f}, aabb);
    EXPECT_NEAR(d, 1.0f, kEpsilon);
    // Point at corner outside
    d = dist::pointAABBSq({3.0f, 3.0f, 3.0f}, aabb);
    EXPECT_NEAR(d, 3.0f, kEpsilon);
}

TEST(Distance, PointCapsule) {
    Capsule c({0.0f, 0.0f, 0.0f}, {0.0f, 3.0f, 0.0f}, 1.0f);
    // Point inside (within radius of the segment)
    f32 d = dist::pointCapsuleSq({0.5f, 1.5f, 0.0f}, c);
    EXPECT_NEAR(d, 0.0f, kEpsilon);
    // Point outside
    d = dist::pointCapsuleSq({5.0f, 1.5f, 0.0f}, c);
    EXPECT_NEAR(d, 16.0f, kEpsilon);
}
