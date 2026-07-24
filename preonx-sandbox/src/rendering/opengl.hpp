#pragma once

#include <GL/gl.h>
#include <GL/glext.h>

namespace gl {

// ── Buffer Objects ───────────────────────────────────────────────────────────
inline void GenVertexArrays(GLsizei n, GLuint* arrays) { glGenVertexArrays(n, arrays); }
inline void BindVertexArray(GLuint array) { glBindVertexArray(array); }
inline void DeleteVertexArrays(GLsizei n, const GLuint* arrays) { glDeleteVertexArrays(n, arrays); }
inline void GenBuffers(GLsizei n, GLuint* buffers) { glGenBuffers(n, buffers); }
inline void BindBuffer(GLenum target, GLuint buffer) { glBindBuffer(target, buffer); }
inline void BufferData(GLenum target, GLsizeiptr size, const void* data, GLenum usage) { glBufferData(target, size, data, usage); }
inline void BufferSubData(GLenum target, GLintptr offset, GLsizeiptr size, const void* data) { glBufferSubData(target, offset, size, data); }
inline void DeleteBuffers(GLsizei n, const GLuint* buffers) { glDeleteBuffers(n, buffers); }

// ── Shaders ──────────────────────────────────────────────────────────────────
inline GLuint CreateShader(GLenum type) { return glCreateShader(type); }
inline void ShaderSource(GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length) { glShaderSource(shader, count, string, length); }
inline void CompileShader(GLuint shader) { glCompileShader(shader); }
inline void GetShaderiv(GLuint shader, GLenum pname, GLint* params) { glGetShaderiv(shader, pname, params); }
inline void GetShaderInfoLog(GLuint shader, GLsizei bufSize, GLsizei* length, GLchar* infoLog) { glGetShaderInfoLog(shader, bufSize, length, infoLog); }
inline GLuint CreateProgram() { return glCreateProgram(); }
inline void AttachShader(GLuint program, GLuint shader) { glAttachShader(program, shader); }
inline void LinkProgram(GLuint program) { glLinkProgram(program); }
inline void GetProgramiv(GLuint program, GLenum pname, GLint* params) { glGetProgramiv(program, pname, params); }
inline void GetProgramInfoLog(GLuint program, GLsizei bufSize, GLsizei* length, GLchar* infoLog) { glGetProgramInfoLog(program, bufSize, length, infoLog); }
inline void UseProgram(GLuint program) { glUseProgram(program); }
inline void DeleteShader(GLuint shader) { glDeleteShader(shader); }
inline void DeleteProgram(GLuint program) { glDeleteProgram(program); }

// ── Uniforms ─────────────────────────────────────────────────────────────────
inline GLint GetUniformLocation(GLuint program, const GLchar* name) { return glGetUniformLocation(program, name); }
inline void UniformMatrix4fv(GLint location, GLsizei count, GLboolean transpose, const GLfloat* value) { glUniformMatrix4fv(location, count, transpose, value); }
inline void Uniform4f(GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3) { glUniform4f(location, v0, v1, v2, v3); }
inline void Uniform1i(GLint location, GLint v0) { glUniform1i(location, v0); }

// ── Attributes ───────────────────────────────────────────────────────────────
inline void EnableVertexAttribArray(GLuint index) { glEnableVertexAttribArray(index); }
inline void DisableVertexAttribArray(GLuint index) { glDisableVertexAttribArray(index); }
inline void VertexAttribPointer(GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer) { glVertexAttribPointer(index, size, type, normalized, stride, pointer); }

// ── Drawing ──────────────────────────────────────────────────────────────────
inline void DrawArrays(GLenum mode, GLint first, GLsizei count) { glDrawArrays(mode, first, count); }
inline void DrawElements(GLenum mode, GLsizei count, GLenum type, const void* indices) { glDrawElements(mode, count, type, indices); }

// ── Framebuffer ──────────────────────────────────────────────────────────────
inline void Viewport(GLint x, GLint y, GLsizei width, GLsizei height) { glViewport(x, y, width, height); }
inline void ClearColor(GLfloat r, GLfloat g, GLfloat b, GLfloat a) { glClearColor(r, g, b, a); }
inline void Clear(GLbitfield mask) { glClear(mask); }

// ── State ────────────────────────────────────────────────────────────────────
inline void LineWidth(GLfloat width) { glLineWidth(width); }
inline void PointSize(GLfloat size) { glPointSize(size); }
inline void Enable(GLenum cap) { glEnable(cap); }
inline void Disable(GLenum cap) { glDisable(cap); }
inline void BlendFunc(GLenum sfactor, GLenum dfactor) { glBlendFunc(sfactor, dfactor); }
inline const GLubyte* GetString(GLenum name) { return glGetString(name); }

} // namespace gl
