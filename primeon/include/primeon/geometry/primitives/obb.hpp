#pragma once

#include "primeon/math/vector/vector3.hpp"
#include "primeon/math/quaternion/quaternion.hpp"

namespace primeon::math {

struct OBB {
    Vector3 center = kVector3Zero;
    Vector3 halfExtents = {0.5f, 0.5f, 0.5f};
    Quaternion orientation = Quaternion::identity();

    constexpr OBB() noexcept = default;
    constexpr OBB(const Vector3& center, const Vector3& halfExtents, const Quaternion& orientation) noexcept
        : center(center), halfExtents(halfExtents), orientation(orientation) {}

    /// Returns the 8 corner points of the OBB.
    void getCorners(Vector3 corners[8]) const noexcept {
        Vector3 axes[3] = {orientation.right(), orientation.up(), orientation.forward()};
        for (int i = 0; i < 8; ++i) {
            corners[i] = center;
            corners[i] += axes[0] * ((i & 1) ? halfExtents.x : -halfExtents.x);
            corners[i] += axes[1] * ((i & 2) ? halfExtents.y : -halfExtents.y);
            corners[i] += axes[2] * ((i & 4) ? halfExtents.z : -halfExtents.z);
        }
    }

    /// Returns the size (full extents).
    [[nodiscard]] constexpr Vector3 size() const noexcept { return halfExtents * 2.0f; }

    /// Returns the volume.
    [[nodiscard]] constexpr f32 volume() const noexcept {
        return size().x * size().y * size().z;
    }

    /// Transforms the OBB by a matrix.
    [[nodiscard]] OBB transformed(const Matrix4& m) const noexcept {
        OBB result;
        result.center = m.transformPoint(center);
        Vector3 axes[3] = {orientation.right(), orientation.up(), orientation.forward()};
        Vector3 newHalfExtents = kVector3Zero;
        for (int i = 0; i < 3; ++i) {
            Vector3 transformedAxis = m.transformDirection(axes[i] * halfExtents[i]);
            newHalfExtents.x += abs(transformedAxis.x);
            newHalfExtents.y += abs(transformedAxis.y);
            newHalfExtents.z += abs(transformedAxis.z);
        }
        result.halfExtents = newHalfExtents;
        result.orientation = orientation;
        return result;
    }
};

} // namespace primeon::math
