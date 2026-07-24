#pragma once

#include <SDL2/SDL.h>

class Input {
public:
    void beginFrame();
    void processEvent(const SDL_Event& event);

    [[nodiscard]] bool isKeyDown(SDL_Scancode key) const { return keys_[key]; }
    [[nodiscard]] bool isKeyPressed(SDL_Scancode key) const { return keys_[key] && !prevKeys_[key]; }
    [[nodiscard]] bool isKeyReleased(SDL_Scancode key) const { return !keys_[key] && prevKeys_[key]; }

    [[nodiscard]] bool isMouseButtonDown(Uint8 button) const { return mouseButtons_[button]; }
    [[nodiscard]] bool isMouseButtonPressed(Uint8 button) const { return mouseButtons_[button] && !prevMouseButtons_[button]; }

    [[nodiscard]] int getMouseX() const { return mouseX_; }
    [[nodiscard]] int getMouseY() const { return mouseY_; }
    [[nodiscard]] int getMouseDeltaX() const { return mouseX_ - prevMouseX_; }
    [[nodiscard]] int getMouseDeltaY() const { return mouseY_ - prevMouseY_; }
    [[nodiscard]] int getMouseWheel() const { return mouseWheel_; }

private:
    bool keys_[SDL_NUM_SCANCODES] = {};
    bool prevKeys_[SDL_NUM_SCANCODES] = {};
    bool mouseButtons_[8] = {};
    bool prevMouseButtons_[8] = {};
    int mouseX_ = 0;
    int mouseY_ = 0;
    int prevMouseX_ = 0;
    int prevMouseY_ = 0;
    int mouseWheel_ = 0;
};
