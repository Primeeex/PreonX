#include "ui/debug_ui.hpp"
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>
#include <cstdio>

void DebugUI::init(void* window, void* glContext) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    io_ = &ImGui::GetIO();
    io_->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;

    ImGui_ImplSDL2_InitForOpenGL(static_cast<SDL_Window*>(window), glContext);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void DebugUI::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    io_ = nullptr;
}

void DebugUI::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
}

void DebugUI::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void DebugUI::renderPanel(DebugUIData& data, SceneManager& scenes) {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f, 400.0f), ImGuiCond_FirstUseEver);

    ImGui::Begin("PreonX Sandbox", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::CollapsingHeader("Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Paused", &data.paused);
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            data.stepOnce = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            scenes.resetScene();
        }

        ImGui::Checkbox("Gravity", &data.gravityEnabled);
        ImGui::SliderFloat("Fixed Dt", &data.fixedDt, 1.0f / 240.0f, 1.0f / 10.0f, "%.4f s");
        ImGui::SliderFloat("Speed", &data.simulationSpeed, 0.0f, 3.0f, "%.1fx");
    }

    if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen)) {
        unsigned cur = scenes.currentSceneIndex();
        unsigned count = scenes.sceneCount();
        for (unsigned i = 0; i < count; ++i) {
            bool selected = (i == cur);
            if (ImGui::Selectable(scenes.sceneName(i), selected)) {
                scenes.selectScene(i);
            }
        }
    }

    if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
        Scene* scene = scenes.currentScene();
        if (scene) {
            const auto& stats = scene->getWorld().getStats();
            ImGui::Text("Bodies:    %u", stats.totalBodies);
            ImGui::Text("Dynamic:   %u", stats.dynamicBodies);
            ImGui::Text("Static:    %u", stats.staticBodies);
            ImGui::Text("Contacts:  %u", stats.contactCount);
            ImGui::Text("Islands:   %u", stats.islandCount);
            ImGui::Text("Frame:     %u", scene->getWorld().getFrameCount());
        }
    }

    if (ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("FPS:       %.0f", static_cast<double>(data.fps));
        ImGui::Text("Frame:     %.2f ms", static_cast<double>(data.frameTime));
        ImGui::Text("Physics:   %.2f ms", static_cast<double>(data.physicsTime));
        ImGui::Text("Render:    %.2f ms", static_cast<double>(data.renderTime));
    }

    ImGui::End();
}
