#pragma once

#include "rendering/opengl.hpp"
#include <primeon/math/vector/vector3.hpp>
#include <vector>
#include <string>

struct DebugVertex {
    float x, y, z;
    float r, g, b, a;
};

class DebugRenderer {
public:
    void init();
    void shutdown();

    void beginFrame();
    void endFrame(const float viewMatrix[16], const float projMatrix[16]);

    void drawLine(const primeon::math::Vector3& a, const primeon::math::Vector3& b,
                  const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});
    void drawLine(const primeon::math::Vector3& a, const primeon::math::Vector3& b,
                  float r, float g, float b2, float alpha = 1.0f);
    void drawPoint(const primeon::math::Vector3& p, float size,
                   const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});
    void drawCircle(const primeon::math::Vector3& center, float radius,
                    const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f}, int segments = 32);
    void drawFilledCircle(const primeon::math::Vector3& center, float radius,
                          const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f}, int segments = 32);
    void drawRectangle(const primeon::math::Vector3& min, const primeon::math::Vector3& max,
                       const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});
    void drawFilledRectangle(const primeon::math::Vector3& min, const primeon::math::Vector3& max,
                             const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});
    void drawBox(const primeon::math::Vector3& center, const primeon::math::Vector3& halfExtents,
                 const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});
    void drawPolygon(const primeon::math::Vector3* points, int count,
                     const primeon::math::Vector3& color = {1.0f, 1.0f, 1.0f});

    void drawArrow(const primeon::math::Vector3& from, const primeon::math::Vector3& dir, float length,
                   const primeon::math::Vector3& color = {1.0f, 0.0f, 0.0f});

    void drawRotatedFilledRect(float cx, float cy, float hx, float hy, float angleRad,
                               const primeon::math::Vector3& color);
    void drawRotatedRect(float cx, float cy, float hx, float hy, float angleRad,
                         const primeon::math::Vector3& color);
    void drawGrid(float y, float halfSize, float spacing, const primeon::math::Vector3& color);

    [[nodiscard]] unsigned getLineCount() const { return static_cast<unsigned>(lineVertices_.size() / 2); }
    [[nodiscard]] unsigned getTriangleCount() const { return static_cast<unsigned>(triVertices_.size() / 3); }

private:
    void initShaders();
    void initBuffers();

    GLuint lineVAO_ = 0, lineVBO_ = 0;
    GLuint triVAO_ = 0, triVBO_ = 0;
    GLuint lineProgram_ = 0;
    GLuint triProgram_ = 0;
    GLint lineMVP_ = -1;
    GLint triMVP_ = -1;
    GLint triColorUniform_ = -1;

    std::vector<DebugVertex> lineVertices_;
    std::vector<DebugVertex> triVertices_;
    static constexpr unsigned kMaxVertices = 65536;
};
