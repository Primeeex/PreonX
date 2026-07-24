#include "rendering/camera.hpp"
#include <cstring>
#include <cmath>

void Camera::init(float viewportWidth, float viewportHeight) {
    viewportWidth_ = viewportWidth;
    viewportHeight_ = viewportHeight;
    center_ = {0.0f, 5.0f, 0.0f};
    zoom_ = 50.0f;
}

void Camera::pan(float dx, float dy) {
    center_.x -= dx / zoom_;
    center_.y += dy / zoom_;
}

void Camera::zoom(float factor) {
    zoom_ *= factor;
    if (zoom_ < 1.0f) zoom_ = 1.0f;
    if (zoom_ > 1000.0f) zoom_ = 1000.0f;
}

void Camera::reset() {
    center_ = {0.0f, 5.0f, 0.0f};
    zoom_ = 50.0f;
}

void Camera::setViewport(float width, float height) {
    viewportWidth_ = width;
    viewportHeight_ = height;
}

void Camera::getViewMatrix(float matrix[16]) const {
    std::memset(matrix, 0, 16 * sizeof(float));
    matrix[0] = 1.0f;
    matrix[5] = 1.0f;
    matrix[10] = 1.0f;
    matrix[15] = 1.0f;
    matrix[12] = -center_.x;
    matrix[13] = -center_.y;
}

void Camera::getProjectionMatrix(float matrix[16]) const {
    std::memset(matrix, 0, 16 * sizeof(float));
    float halfW = (viewportWidth_ * 0.5f) / zoom_;
    float halfH = (viewportHeight_ * 0.5f) / zoom_;
    matrix[0] = 1.0f / halfW;
    matrix[5] = 1.0f / halfH;
    matrix[10] = -1.0f;
    matrix[15] = 1.0f;
}

primeon::math::Vector3 Camera::screenToWorld(float sx, float sy) const {
    float ndcX = (2.0f * sx / viewportWidth_) - 1.0f;
    float ndcY = 1.0f - (2.0f * sy / viewportHeight_);
    float halfW = (viewportWidth_ * 0.5f) / zoom_;
    float halfH = (viewportHeight_ * 0.5f) / zoom_;
    return {center_.x + ndcX * halfW, center_.y + ndcY * halfH, 0.0f};
}
