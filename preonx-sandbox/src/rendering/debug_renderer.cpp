#include "rendering/debug_renderer.hpp"
#include <cstdio>
#include <cmath>

static const char* kLineVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMVP;
out vec4 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

static const char* kLineFragSrc = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)";

static const char* kTriVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMVP;
out vec4 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
)";

static const char* kTriFragSrc = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)";

static GLuint compileShader(GLenum type, const char* src) {
    GLuint shader = gl::CreateShader(type);
    gl::ShaderSource(shader, 1, &src, nullptr);
    gl::CompileShader(shader);
    GLint ok = 0;
    gl::GetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        gl::GetShaderInfoLog(shader, 512, nullptr, log);
        std::printf("Shader compile error: %s\n", log);
    }
    return shader;
}

static GLuint linkProgram(GLuint vert, GLuint frag) {
    GLuint prog = gl::CreateProgram();
    gl::AttachShader(prog, vert);
    gl::AttachShader(prog, frag);
    gl::LinkProgram(prog);
    GLint ok = 0;
    gl::GetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        gl::GetProgramInfoLog(prog, 512, nullptr, log);
        std::printf("Program link error: %s\n", log);
    }
    gl::DeleteShader(vert);
    gl::DeleteShader(frag);
    return prog;
}

void DebugRenderer::init() {
    initShaders();
    initBuffers();
    lineVertices_.reserve(kMaxVertices);
    triVertices_.reserve(kMaxVertices);
}

void DebugRenderer::shutdown() {
    if (lineVAO_) gl::DeleteVertexArrays(1, &lineVAO_);
    if (lineVBO_) gl::DeleteBuffers(1, &lineVBO_);
    if (triVAO_) gl::DeleteVertexArrays(1, &triVAO_);
    if (triVBO_) gl::DeleteBuffers(1, &triVBO_);
    if (lineProgram_) gl::DeleteProgram(lineProgram_);
    if (triProgram_) gl::DeleteProgram(triProgram_);
}

void DebugRenderer::initShaders() {
    GLuint lv = compileShader(GL_VERTEX_SHADER, kLineVertSrc);
    GLuint lf = compileShader(GL_FRAGMENT_SHADER, kLineFragSrc);
    lineProgram_ = linkProgram(lv, lf);
    lineMVP_ = gl::GetUniformLocation(lineProgram_, "uMVP");

    GLuint tv = compileShader(GL_VERTEX_SHADER, kTriVertSrc);
    GLuint tf = compileShader(GL_FRAGMENT_SHADER, kTriFragSrc);
    triProgram_ = linkProgram(tv, tf);
    triMVP_ = gl::GetUniformLocation(triProgram_, "uMVP");
}

void DebugRenderer::initBuffers() {
    gl::GenVertexArrays(1, &lineVAO_);
    gl::GenBuffers(1, &lineVBO_);
    gl::BindVertexArray(lineVAO_);
    gl::BindBuffer(GL_ARRAY_BUFFER, lineVBO_);
    gl::BufferData(GL_ARRAY_BUFFER, kMaxVertices * sizeof(DebugVertex), nullptr, GL_DYNAMIC_DRAW);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), nullptr);
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), reinterpret_cast<const void*>(3 * sizeof(float)));
    gl::BindVertexArray(0);

    gl::GenVertexArrays(1, &triVAO_);
    gl::GenBuffers(1, &triVBO_);
    gl::BindVertexArray(triVAO_);
    gl::BindBuffer(GL_ARRAY_BUFFER, triVBO_);
    gl::BufferData(GL_ARRAY_BUFFER, kMaxVertices * sizeof(DebugVertex), nullptr, GL_DYNAMIC_DRAW);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), nullptr);
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), reinterpret_cast<const void*>(3 * sizeof(float)));
    gl::BindVertexArray(0);
}

void DebugRenderer::beginFrame() {
    lineVertices_.clear();
    triVertices_.clear();
}

void DebugRenderer::endFrame(const float viewMatrix[16], const float projMatrix[16]) {
    float mvp[16];
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            mvp[c * 4 + r] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                mvp[c * 4 + r] += projMatrix[k * 4 + r] * viewMatrix[c * 4 + k];
            }
        }
    }

    gl::Enable(GL_BLEND);
    gl::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (!lineVertices_.empty()) {
        gl::UseProgram(lineProgram_);
        gl::UniformMatrix4fv(lineMVP_, 1, GL_FALSE, mvp);
        gl::BindVertexArray(lineVAO_);
        gl::BindBuffer(GL_ARRAY_BUFFER, lineVBO_);
        gl::BufferSubData(GL_ARRAY_BUFFER, 0,
                          static_cast<GLsizeiptr>(lineVertices_.size() * sizeof(DebugVertex)),
                          lineVertices_.data());
        gl::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVertices_.size()));
        gl::BindVertexArray(0);
    }

    if (!triVertices_.empty()) {
        gl::UseProgram(triProgram_);
        gl::UniformMatrix4fv(triMVP_, 1, GL_FALSE, mvp);
        gl::BindVertexArray(triVAO_);
        gl::BindBuffer(GL_ARRAY_BUFFER, triVBO_);
        gl::BufferSubData(GL_ARRAY_BUFFER, 0,
                          static_cast<GLsizeiptr>(triVertices_.size() * sizeof(DebugVertex)),
                          triVertices_.data());
        gl::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(triVertices_.size()));
        gl::BindVertexArray(0);
    }

    gl::UseProgram(0);
    gl::Disable(GL_BLEND);
}

void DebugRenderer::drawLine(const primeon::math::Vector3& a, const primeon::math::Vector3& b,
                              const primeon::math::Vector3& color) {
    drawLine(a, b, color.x, color.y, color.z);
}

void DebugRenderer::drawLine(const primeon::math::Vector3& a, const primeon::math::Vector3& b,
                              float r, float g, float bv, float alpha) {
    if (lineVertices_.size() + 2 > kMaxVertices) return;
    lineVertices_.emplace_back(DebugVertex{a.x, a.y, a.z, r, g, bv, alpha});
    lineVertices_.emplace_back(DebugVertex{b.x, b.y, b.z, r, g, bv, alpha});
}

void DebugRenderer::drawPoint(const primeon::math::Vector3& p, float size,
                               const primeon::math::Vector3& color) {
    float hs = size * 0.5f;
    drawLine({p.x - hs, p.y, p.z}, {p.x + hs, p.y, p.z}, color);
    drawLine({p.x, p.y - hs, p.z}, {p.x, p.y + hs, p.z}, color);
}

void DebugRenderer::drawCircle(const primeon::math::Vector3& center, float radius,
                                const primeon::math::Vector3& color, int segments) {
    constexpr float kPi2 = 6.283185307f;
    for (int i = 0; i < segments; ++i) {
        float a0 = static_cast<float>(i) / static_cast<float>(segments) * kPi2;
        float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * kPi2;
        float x0 = center.x + std::cos(a0) * radius;
        float y0 = center.y + std::sin(a0) * radius;
        float x1 = center.x + std::cos(a1) * radius;
        float y1 = center.y + std::sin(a1) * radius;
        drawLine({x0, y0, center.z}, {x1, y1, center.z}, color);
    }
}

void DebugRenderer::drawFilledCircle(const primeon::math::Vector3& center, float radius,
                                      const primeon::math::Vector3& color, int segments) {
    constexpr float kPi2 = 6.283185307f;
    for (int i = 0; i < segments; ++i) {
        float a0 = static_cast<float>(i) / static_cast<float>(segments) * kPi2;
        float a1 = static_cast<float>(i + 1) / static_cast<float>(segments) * kPi2;
        float x0 = center.x + std::cos(a0) * radius;
        float y0 = center.y + std::sin(a0) * radius;
        float x1 = center.x + std::cos(a1) * radius;
        float y1 = center.y + std::sin(a1) * radius;
        if (triVertices_.size() + 3 <= kMaxVertices) {
            triVertices_.emplace_back(DebugVertex{center.x, center.y, center.z, color.x, color.y, color.z, 0.5f});
            triVertices_.emplace_back(DebugVertex{x0, y0, center.z, color.x, color.y, color.z, 0.5f});
            triVertices_.emplace_back(DebugVertex{x1, y1, center.z, color.x, color.y, color.z, 0.5f});
        }
    }
}

void DebugRenderer::drawRectangle(const primeon::math::Vector3& min, const primeon::math::Vector3& max,
                                   const primeon::math::Vector3& color) {
    drawLine({min.x, min.y, min.z}, {max.x, min.y, min.z}, color);
    drawLine({max.x, min.y, min.z}, {max.x, max.y, min.z}, color);
    drawLine({max.x, max.y, min.z}, {min.x, max.y, min.z}, color);
    drawLine({min.x, max.y, min.z}, {min.x, min.y, min.z}, color);
}

void DebugRenderer::drawFilledRectangle(const primeon::math::Vector3& min, const primeon::math::Vector3& max,
                                         const primeon::math::Vector3& color) {
    if (triVertices_.size() + 6 > kMaxVertices) return;
    float z = min.z;
    float a = 0.4f;
    triVertices_.emplace_back(DebugVertex{min.x, min.y, z, color.x, color.y, color.z, a});
    triVertices_.emplace_back(DebugVertex{max.x, min.y, z, color.x, color.y, color.z, a});
    triVertices_.emplace_back(DebugVertex{max.x, max.y, z, color.x, color.y, color.z, a});

    triVertices_.emplace_back(DebugVertex{min.x, min.y, z, color.x, color.y, color.z, a});
    triVertices_.emplace_back(DebugVertex{max.x, max.y, z, color.x, color.y, color.z, a});
    triVertices_.emplace_back(DebugVertex{min.x, max.y, z, color.x, color.y, color.z, a});
}

void DebugRenderer::drawBox(const primeon::math::Vector3& center, const primeon::math::Vector3& halfExtents,
                             const primeon::math::Vector3& color) {
    primeon::math::Vector3 min = {center.x - halfExtents.x, center.y - halfExtents.y, center.z};
    primeon::math::Vector3 max = {center.x + halfExtents.x, center.y + halfExtents.y, center.z};
    drawRectangle(min, max, color);
}

void DebugRenderer::drawPolygon(const primeon::math::Vector3* points, int count,
                                 const primeon::math::Vector3& color) {
    for (int i = 0; i < count; ++i) {
        int next = (i + 1) % count;
        drawLine(points[i], points[next], color);
    }
}

void DebugRenderer::drawArrow(const primeon::math::Vector3& from, const primeon::math::Vector3& dir,
                               float length, const primeon::math::Vector3& color) {
    primeon::math::Vector3 end = {from.x + dir.x * length, from.y + dir.y * length, from.z + dir.z * length};
    drawLine(from, end, color);

    float hs = length * 0.15f;
    float nx = -dir.y;
    float ny = dir.x;
    primeon::math::Vector3 p1 = {end.x - dir.x * hs + nx * hs * 0.5f, end.y - dir.y * hs + ny * hs * 0.5f, end.z};
    primeon::math::Vector3 p2 = {end.x - dir.x * hs - nx * hs * 0.5f, end.y - dir.y * hs - ny * hs * 0.5f, end.z};
    drawLine(end, p1, color);
    drawLine(end, p2, color);
}

void DebugRenderer::drawRotatedFilledRect(float cx, float cy, float hx, float hy, float angleRad,
                                           const primeon::math::Vector3& color) {
    float c = std::cos(angleRad);
    float s = std::sin(angleRad);
    float ax = hx * c, ay = hx * s;
    float bx = -hy * s, by = hy * c;

    float x0 = cx + ax + bx, y0 = cy + ay + by;
    float x1 = cx - ax + bx, y1 = cy - ay + by;
    float x2 = cx - ax - bx, y2 = cy - ay - by;
    float x3 = cx + ax - bx, y3 = cy + ay - by;

    if (triVertices_.size() + 6 <= kMaxVertices) {
        triVertices_.emplace_back(DebugVertex{x0, y0, 0.0f, color.x, color.y, color.z, 0.85f});
        triVertices_.emplace_back(DebugVertex{x1, y1, 0.0f, color.x, color.y, color.z, 0.85f});
        triVertices_.emplace_back(DebugVertex{x2, y2, 0.0f, color.x, color.y, color.z, 0.85f});
        triVertices_.emplace_back(DebugVertex{x0, y0, 0.0f, color.x, color.y, color.z, 0.85f});
        triVertices_.emplace_back(DebugVertex{x2, y2, 0.0f, color.x, color.y, color.z, 0.85f});
        triVertices_.emplace_back(DebugVertex{x3, y3, 0.0f, color.x, color.y, color.z, 0.85f});
    }
}

void DebugRenderer::drawRotatedRect(float cx, float cy, float hx, float hy, float angleRad,
                                     const primeon::math::Vector3& color) {
    float cs = std::cos(angleRad);
    float sn = std::sin(angleRad);
    float ax = hx * cs, ay = hx * sn;
    float bx = -hy * sn, by = hy * cs;

    primeon::math::Vector3 p0 = {cx + ax + bx, cy + ay + by, 0.0f};
    primeon::math::Vector3 p1 = {cx - ax + bx, cy - ay + by, 0.0f};
    primeon::math::Vector3 p2 = {cx - ax - bx, cy - ay - by, 0.0f};
    primeon::math::Vector3 p3 = {cx + ax - bx, cy + ay - by, 0.0f};

    drawLine(p0, p1, color);
    drawLine(p1, p2, color);
    drawLine(p2, p3, color);
    drawLine(p3, p0, color);
}

void DebugRenderer::drawGrid(float y, float halfSize, float spacing, const primeon::math::Vector3& color) {
    for (float x = -halfSize; x <= halfSize; x += spacing) {
        drawLine({x, y, 0.0f}, {x, y, 0.0f}, color);
    }
    for (float z = -halfSize; z <= halfSize; z += spacing) {
        drawLine({-halfSize, y, z}, {halfSize, y, z}, color);
    }
}
