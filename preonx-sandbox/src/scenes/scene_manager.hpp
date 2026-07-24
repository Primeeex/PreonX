#pragma once

#include "scenes/scene.hpp"
#include <memory>
#include <vector>
#include <string>

class SceneManager {
public:
    void addScene(std::unique_ptr<Scene> scene);

    void selectScene(unsigned index);
    void resetScene();
    void nextScene();
    void prevScene();

    [[nodiscard]] Scene* currentScene() const;
    [[nodiscard]] unsigned currentSceneIndex() const { return currentIndex_; }
    [[nodiscard]] unsigned sceneCount() const { return static_cast<unsigned>(scenes_.size()); }
    [[nodiscard]] const std::string& currentSceneName() const;
    [[nodiscard]] const char* sceneName(unsigned index) const;

    void listAll() const;

private:
    std::vector<std::unique_ptr<Scene>> scenes_;
    unsigned currentIndex_ = 0;
};
