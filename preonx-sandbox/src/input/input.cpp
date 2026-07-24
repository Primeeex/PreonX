#include "input/input.hpp"
#include <cstring>

void Input::beginFrame() {
    std::memcpy(prevKeys_, keys_, sizeof(keys_));
    std::memcpy(prevMouseButtons_, mouseButtons_, sizeof(mouseButtons_));
    prevMouseX_ = mouseX_;
    prevMouseY_ = mouseY_;
    mouseWheel_ = 0;
}

void Input::processEvent(const SDL_Event& event) {
    switch (event.type) {
        case SDL_KEYDOWN:
            if (!event.key.repeat) {
                keys_[event.key.keysym.scancode] = true;
            }
            break;
        case SDL_KEYUP:
            keys_[event.key.keysym.scancode] = false;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button < 8) {
                mouseButtons_[event.button.button] = true;
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (event.button.button < 8) {
                mouseButtons_[event.button.button] = false;
            }
            break;
        case SDL_MOUSEMOTION:
            mouseX_ = event.motion.x;
            mouseY_ = event.motion.y;
            break;
        case SDL_MOUSEWHEEL:
            mouseWheel_ = event.wheel.y;
            break;
    }
}
