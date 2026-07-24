#pragma once

#include <primeon/math/vector/vector3.hpp>

class Camera {
public:
    void init(float viewportWidth, float viewportHeight);

    void pan(float dx, float dy);
    void zoom(float factor);
    void reset();

    void setViewport(float width, float height);

    [[nodiscard]] float getZoom() const { return zoom_; }
    [[nodiscard]] const primeon::math::Vector3& getCenter() const { return center_; }

    [[nodiscard]] float getViewLeft() const { return center_.x - (viewportWidth_ * 0.5f) / zoom_; }
    [[nodiscard]] float getViewRight() const { return center_.x + (viewportWidth_ * 0.5f) / zoom_; }
    [[nodiscard]] float getViewBottom() const { return center_.y - (viewportHeight_ * 0.5f) / zoom_; }
    [[nodiscard]] float getViewTop() const { return center_.y + (viewportHeight_ * 0.5f) / zoom_; }

    void getViewMatrix(float matrix[16]) const;
    void getProjectionMatrix(float matrix[16]) const;

    primeon::math::Vector3 screenToWorld(float sx, float sy) const;

private:
    primeon::math::Vector3 center_ = {0.0f, 0.0f, 0.0f};
    float zoom_ = 50.0f;
    float viewportWidth_ = 1280.0f;
    float viewportHeight_ = 720.0f;
};
