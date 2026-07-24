#include <gtest/gtest.h>
#include <primeon/geometry/intersection/intersection.hpp>

using namespace primeon::math;

TEST(Intersect, RaySphere_Hit) {
    Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Sphere s({5.0f, 0.0f, 0.0f}, 2.0f);
    auto hit = intersect::raySphere(r, s);
    EXPECT_TRUE(hit.hit);
    EXPECT_NEAR(hit.distance, 3.0f, kEpsilon);
    EXPECT_NEAR(hit.point.x, 3.0f, kEpsilon);
}

TEST(Intersect, RaySphere_Miss) {
    Ray r({0.0f, 10.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Sphere s({5.0f, 0.0f, 0.0f}, 2.0f);
    auto hit = intersect::raySphere(r, s);
    EXPECT_FALSE(hit.hit);
}

TEST(Intersect, RaySphere_OriginInside) {
    Ray r({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Sphere s({0.0f, 0.0f, 0.0f}, 5.0f);
    auto hit = intersect::raySphere(r, s);
    EXPECT_TRUE(hit.hit);
    EXPECT_NEAR(hit.distance, 5.0f, kEpsilon);
}

TEST(Intersect, RayPlane_Hit) {
    Ray r({0.0f, 5.0f, 0.0f}, {0.0f, -1.0f, 0.0f});
    Plane p(kVector3UnitY, 0.0f);
    auto hit = intersect::rayPlane(r, p);
    EXPECT_TRUE(hit.hit);
    EXPECT_NEAR(hit.distance, 5.0f, kEpsilon);
}

TEST(Intersect, RayPlane_Parallel) {
    Ray r({0.0f, 5.0f, 0.0f}, {1.0f, 0.0f, 0.0f});
    Plane p(kVector3UnitY, 0.0f);
    auto hit = intersect::rayPlane(r, p);
    EXPECT_FALSE(hit.hit);
}

TEST(Intersect, RayPlane_AwayFromPlane) {
    Ray r({0.0f, 5.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    Plane p(kVector3UnitY, 0.0f);
    auto hit = intersect::rayPlane(r, p);
    EXPECT_FALSE(hit.hit);
}

TEST(Intersect, RayAABB_Hit) {
    Ray r({-5.0f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB aabb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    auto hit = intersect::rayAABB(r, aabb);
    EXPECT_TRUE(hit.hit);
    EXPECT_NEAR(hit.distance, 5.0f, kEpsilon);
    EXPECT_NEAR(hit.point.x, 0.0f, kEpsilon);
}

TEST(Intersect, RayAABB_Miss) {
    Ray r({-5.0f, 2.0f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB aabb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    auto hit = intersect::rayAABB(r, aabb);
    EXPECT_FALSE(hit.hit);
}

TEST(Intersect, RayAABB_OriginInside) {
    Ray r({0.5f, 0.5f, 0.5f}, {1.0f, 0.0f, 0.0f});
    AABB aabb({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    auto hit = intersect::rayAABB(r, aabb);
    EXPECT_TRUE(hit.hit);
}

TEST(Intersect, SphereSphere_Hit) {
    Sphere a({0.0f, 0.0f, 0.0f}, 2.0f);
    Sphere b({3.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_TRUE(intersect::sphereSphere(a, b));
}

TEST(Intersect, SphereSphere_Miss) {
    Sphere a({0.0f, 0.0f, 0.0f}, 1.0f);
    Sphere b({5.0f, 0.0f, 0.0f}, 1.0f);
    EXPECT_FALSE(intersect::sphereSphere(a, b));
}

TEST(Intersect, SphereSphere_Tangent) {
    Sphere a({0.0f, 0.0f, 0.0f}, 2.0f);
    Sphere b({4.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_TRUE(intersect::sphereSphere(a, b));
}

TEST(Intersect, AABB_AABB_Hit) {
    AABB a({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    AABB b({1.0f, 1.0f, 1.0f}, {3.0f, 3.0f, 3.0f});
    EXPECT_TRUE(intersect::aabbAABB(a, b));
}

TEST(Intersect, AABB_AABB_Miss) {
    AABB a({0.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 1.0f});
    AABB b({2.0f, 2.0f, 2.0f}, {3.0f, 3.0f, 3.0f});
    EXPECT_FALSE(intersect::aabbAABB(a, b));
}

TEST(Intersect, PointInsideAABB) {
    AABB aabb({0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 2.0f});
    EXPECT_TRUE(intersect::pointInsideAABB({1.0f, 1.0f, 1.0f}, aabb));
    EXPECT_FALSE(intersect::pointInsideAABB({3.0f, 1.0f, 1.0f}, aabb));
}

TEST(Intersect, PointInsideSphere) {
    Sphere s({0.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_TRUE(intersect::pointInsideSphere({1.0f, 0.0f, 0.0f}, s));
    EXPECT_FALSE(intersect::pointInsideSphere({3.0f, 0.0f, 0.0f}, s));
}

TEST(Intersect, SegmentSphere_Hit) {
    Segment s({0.0f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f});
    Sphere sphere({5.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_TRUE(intersect::segmentSphere(s, sphere));
}

TEST(Intersect, SegmentSphere_Miss) {
    Segment s({0.0f, 5.0f, 0.0f}, {10.0f, 5.0f, 0.0f});
    Sphere sphere({5.0f, 0.0f, 0.0f}, 2.0f);
    EXPECT_FALSE(intersect::segmentSphere(s, sphere));
}

TEST(Intersect, PointInsideOBB) {
    Quaternion id = Quaternion::identity();
    EXPECT_TRUE(intersect::pointInsideOBB({0.5f, 0.5f, 0.5f}, kVector3Zero, {1.0f, 1.0f, 1.0f}, id));
    EXPECT_FALSE(intersect::pointInsideOBB({2.0f, 2.0f, 2.0f}, kVector3Zero, {1.0f, 1.0f, 1.0f}, id));
}
