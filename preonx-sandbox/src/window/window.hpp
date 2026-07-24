#pragma once

#include <SDL2/SDL.h>

struct SDL_Window;

class Window {
public:
    Window() = default;
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&&) = delete;
    Window& operator=(Window&&) = delete;

    [[nodiscard]] bool create(const char* title, unsigned width, unsigned height);
    void destroy();
    void pollEvents();

    [[nodiscard]] SDL_Window* getHandle() const { return window_; }
    [[nodiscard]] SDL_GLContext getGLContext() const { return glContext_; }
    [[nodiscard]] unsigned getWidth() const { return width_; }
    [[nodiscard]] unsigned getHeight() const { return height_; }
    [[nodiscard]] bool isOpen() const { return open_; }
    [[nodiscard]] bool wasResized() const { return resized_; }
    void clearResized() { resized_ = false; }

private:
    SDL_Window* window_ = nullptr;
    SDL_GLContext glContext_ = nullptr;
    unsigned width_ = 0;
    unsigned height_ = 0;
    bool open_ = false;
    bool resized_ = false;
};
