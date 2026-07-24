#pragma once

#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"
#include "primeon/math/matrix/matrix4.hpp"

namespace primeon::math {

struct Transform {
    Vector3 position = kVector3Zero;
    Quaternion rotation = Quaternion::identity();
    Vector3 scale = kVector3One;

    constexpr Transform() noexcept = default;
    constexpr Transform(const Vector3& pos, const Quaternion& rot, const Vector3& scl) noexcept
        : position(pos), rotation(rot), scale(scl) {}
    constexpr Transform(const Vector3& pos, const Quaternion& rot) noexcept
        : position(pos), rotation(rot) {}

    /// Returns the identity transform.
    [[nodiscard]] static constexpr Transform identity() noexcept { return {}; }

    /// Returns the local-to-world matrix (TRS).
    [[nodiscard]] Matrix4 toMatrix() const noexcept {
        Matrix3 r = rotation.toMatrix3();
        Matrix3 s = Matrix3::scale(scale.x, scale.y, scale.z);
        Matrix3 rs = r * s;
        Matrix4 m{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                m.m[row][col] = rs.m[row][col];
            }
            m.m[row][3] = position[row];
        }
        return m;
    }

    /// Returns the inverse of the local-to-world matrix.
    [[nodiscard]] Transform inverse() const noexcept {
        Quaternion invRot = rotation.inverse();
        Vector3 invScale = {1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z};
        Vector3 invPos = invRot.rotate(-position);
        invPos = {invPos.x * invScale.x, invPos.y * invScale.y, invPos.z * invScale.z};
        return {invPos, invRot, invScale};
    }

    /// Transforms a point from local to world space.
    [[nodiscard]] Vector3 transformPoint(const Vector3& point) const noexcept {
        return position + rotation.rotate(point.cmul(scale));
    }

    /// Transforms a direction from local to world space (ignores translation, applies rotation and scale).
    [[nodiscard]] Vector3 transformDirection(const Vector3& dir) const noexcept {
        return rotation.rotate(dir.cmul(scale));
    }

    /// Transforms a point from world to local space.
    [[nodiscard]] Vector3 inverseTransformPoint(const Vector3& point) const noexcept {
        Vector3 d = point - position;
        Vector3 r = rotation.inverse().rotate(d);
        return {r.x / scale.x, r.y / scale.y, r.z / scale.z};
    }

    /// Transforms a direction from world to local space.
    [[nodiscard]] Vector3 inverseTransformDirection(const Vector3& dir) const noexcept {
        Vector3 r = rotation.inverse().rotate(dir);
        return {r.x / scale.x, r.y / scale.y, r.z / scale.z};
    }

    /// Composes two transforms: this * other (first apply other, then this).
    [[nodiscard]] Transform operator*(const Transform& other) const noexcept {
        return {transformPoint(other.position), rotation * other.rotation, scale.cmul(other.scale)};
    }

    constexpr bool operator==(const Transform& o) const noexcept {
        return position == o.position && rotation == o.rotation && scale == o.scale;
    }

    constexpr bool operator!=(const Transform& o) const noexcept { return !(*this == o); }
};

} // namespace primeon::math
