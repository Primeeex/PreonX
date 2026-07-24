#pragma once

#include "primeon/math/scalar/scalar.hpp"

namespace primeon::math {

// ── Physics Material ───────────────────────────────────────────────────────
//
// Reusable material properties for collision response.
// Materials are value types with no ownership semantics — multiple bodies
// can share the same material. Future material combination (averaging,
// max, user-defined) is supported via the combine modes.
//
// Friction: [0, ∞). 0 = frictionless, 1 = high friction, >1 = rubber grip.
// Restitution: [0, 1]. 0 = perfectly inelastic, 1 = perfectly elastic.
// Density: (0, ∞). Used to compute mass from collision shape volume.

/// Material combination mode for resolving contact properties.
enum class MaterialCombine : u32 {
    Average = 0,   ///< (A + B) / 2
    Min     = 1,   ///< min(A, B)
    Max     = 2,   ///< max(A, B)
    Multiply = 3,  ///< A * B
};

/// Reusable physics material.
struct PhysicsMaterial {
    f32 friction    = 0.5f;
    f32 restitution = 0.0f;
    f32 density     = 1.0f;

    MaterialCombine frictionCombine    = MaterialCombine::Average;
    MaterialCombine restitutionCombine = MaterialCombine::Average;

    constexpr PhysicsMaterial() noexcept = default;

    constexpr PhysicsMaterial(f32 friction, f32 restitution, f32 density) noexcept
        : friction(friction), restitution(restitution), density(density) {}

    /// Computes effective friction between two materials.
    [[nodiscard]] static constexpr f32 combineFriction(
        f32 a, f32 b, MaterialCombine mode) noexcept
    {
        switch (mode) {
            case MaterialCombine::Average:  return (a + b) * 0.5f;
            case MaterialCombine::Min:      return (a < b) ? a : b;
            case MaterialCombine::Max:      return (a > b) ? a : b;
            case MaterialCombine::Multiply: return a * b;
        }
        return (a + b) * 0.5f;
    }

    /// Computes effective restitution between two materials.
    [[nodiscard]] static constexpr f32 combineRestitution(
        f32 a, f32 b, MaterialCombine mode) noexcept
    {
        switch (mode) {
            case MaterialCombine::Average:  return (a + b) * 0.5f;
            case MaterialCombine::Min:      return (a < b) ? a : b;
            case MaterialCombine::Max:      return (a > b) ? a : b;
            case MaterialCombine::Multiply: return a * b;
        }
        return (a + b) * 0.5f;
    }

    /// Combines two materials using their respective combine modes.
    [[nodiscard]] constexpr f32 combinedFriction(const PhysicsMaterial& other) const noexcept {
        return combineFriction(friction, other.friction, frictionCombine);
    }

    [[nodiscard]] constexpr f32 combinedRestitution(const PhysicsMaterial& other) const noexcept {
        return combineRestitution(restitution, other.restitution, restitutionCombine);
    }

    /// Default materials
    [[nodiscard]] static constexpr PhysicsMaterial defaultMaterial() noexcept { return {}; }

    [[nodiscard]] static constexpr PhysicsMaterial frictionless() noexcept {
        PhysicsMaterial m;
        m.friction = 0.0f;
        return m;
    }

    [[nodiscard]] static constexpr PhysicsMaterial bouncy() noexcept {
        PhysicsMaterial m;
        m.restitution = 0.8f;
        return m;
    }

    [[nodiscard]] static constexpr PhysicsMaterial rubber() noexcept {
        PhysicsMaterial m;
        m.friction = 0.9f;
        m.restitution = 0.7f;
        return m;
    }

    [[nodiscard]] static constexpr PhysicsMaterial ice() noexcept {
        PhysicsMaterial m;
        m.friction = 0.05f;
        m.restitution = 0.1f;
        return m;
    }

    constexpr bool operator==(const PhysicsMaterial& o) const noexcept {
        return nearEqual(friction, o.friction) && nearEqual(restitution, o.restitution)
            && nearEqual(density, o.density);
    }
    constexpr bool operator!=(const PhysicsMaterial& o) const noexcept { return !(*this == o); }
};

} // namespace primeon::math
