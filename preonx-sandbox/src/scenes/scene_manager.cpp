#include "scenes/scene_manager.hpp"
#include <cstdio>

void SceneManager::addScene(std::unique_ptr<Scene> scene) {
    scenes_.push_back(std::move(scene));
}

void SceneManager::selectScene(unsigned index) {
    if (index >= scenes_.size()) return;
    if (auto* cur = currentScene()) {
        cur->cleanup();
    }
    currentIndex_ = index;
    if (auto* next = currentScene()) {
        next->init();
    }
}

void SceneManager::resetScene() {
    if (auto* cur = currentScene()) {
        cur->cleanup();
        cur->init();
    }
}

void SceneManager::nextScene() {
    if (scenes_.empty()) return;
    selectScene((currentIndex_ + 1) % static_cast<unsigned>(scenes_.size()));
}

void SceneManager::prevScene() {
    if (scenes_.empty()) return;
    selectScene(currentIndex_ == 0 ? static_cast<unsigned>(scenes_.size()) - 1 : currentIndex_ - 1);
}

Scene* SceneManager::currentScene() const {
    if (currentIndex_ < scenes_.size()) {
        return scenes_[currentIndex_].get();
    }
    return nullptr;
}

const std::string& SceneManager::currentSceneName() const {
    static const std::string empty;
    if (auto* s = currentScene()) {
        static std::string cached;
        cached = s->name();
        return cached;
    }
    return empty;
}

const char* SceneManager::sceneName(unsigned index) const {
    if (index < scenes_.size()) {
        return scenes_[index]->name();
    }
    return "";
}

void SceneManager::listAll() const {
    std::printf("Available scenes:\n");
    for (unsigned i = 0; i < scenes_.size(); ++i) {
        std::printf("  [%u] %-24s %s\n", i, scenes_[i]->name(), scenes_[i]->description());
    }
}
