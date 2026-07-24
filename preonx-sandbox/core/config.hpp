#pragma once

#include <cstdio>
#include <cstring>
#include <string>

struct SandboxConfig {
    bool gravityEnabled = true;
    float gravityStrength = 9.81f;

    int boxStackCount = 12;
    float boxStackMass = 1.0f;
    float boxStackRestitution = 0.0f;
    float boxStackSpacing = 1.05f;

    float collisionSphereMass = 1.0f;
    float collisionBoxMass = 1.5f;

    int stressGridX = 12;
    int stressGridY = 8;
    int stressGridZ = 8;
    float stressMass = 1.0f;

    float crashProjectileMass = 20.0f;
    float crashProjectileSpeed = 25.0f;
    int crashBoxCount = 8;
    float crashBoxMass = 1.0f;

    [[nodiscard]] static SandboxConfig loadFrom(const char* path) {
        SandboxConfig cfg;
        std::FILE* f = std::fopen(path, "r");
        if (!f) return cfg;

        std::string section;
        char line[256];
        while (std::fgets(line, sizeof(line), f)) {
            if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;

            char* p = line;
            while (*p == ' ' || *p == '\t') ++p;
            char* end = p + std::strlen(p);
            while (end > p && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == ' ')) --end;
            *end = '\0';

            if (p[0] == '[') {
                section = std::string(p + 1, end - 1);
                continue;
            }

            char* eq = std::strchr(p, '=');
            if (!eq) continue;

            std::string key(p, eq);
            std::string val(eq + 1);
            while (!key.empty() && key.back() == ' ') key.pop_back();
            while (!val.empty() && val.front() == ' ') val.erase(val.begin());

            const char* k = key.c_str();
            const char* v = val.c_str();

            if (section == "gravity") {
                if (std::strcmp(k, "enabled") == 0)
                    cfg.gravityEnabled = (std::strcmp(v, "true") == 0 || std::strcmp(v, "1") == 0);
                else if (std::strcmp(k, "strength") == 0)
                    cfg.gravityStrength = static_cast<float>(std::atof(v));
            } else if (section == "box_stack") {
                if (std::strcmp(k, "count") == 0)
                    cfg.boxStackCount = std::atoi(v);
                else if (std::strcmp(k, "mass") == 0)
                    cfg.boxStackMass = static_cast<float>(std::atof(v));
                else if (std::strcmp(k, "restitution") == 0)
                    cfg.boxStackRestitution = static_cast<float>(std::atof(v));
                else if (std::strcmp(k, "spacing") == 0)
                    cfg.boxStackSpacing = static_cast<float>(std::atof(v));
            } else if (section == "collision") {
                if (std::strcmp(k, "sphere_mass") == 0)
                    cfg.collisionSphereMass = static_cast<float>(std::atof(v));
                else if (std::strcmp(k, "box_mass") == 0)
                    cfg.collisionBoxMass = static_cast<float>(std::atof(v));
            } else if (section == "stress") {
                if (std::strcmp(k, "grid_x") == 0)
                    cfg.stressGridX = std::atoi(v);
                else if (std::strcmp(k, "grid_y") == 0)
                    cfg.stressGridY = std::atoi(v);
                else if (std::strcmp(k, "grid_z") == 0)
                    cfg.stressGridZ = std::atoi(v);
                else if (std::strcmp(k, "mass") == 0)
                    cfg.stressMass = static_cast<float>(std::atof(v));
            } else if (section == "crash") {
                if (std::strcmp(k, "projectile_mass") == 0)
                    cfg.crashProjectileMass = static_cast<float>(std::atof(v));
                else if (std::strcmp(k, "projectile_speed") == 0)
                    cfg.crashProjectileSpeed = static_cast<float>(std::atof(v));
                else if (std::strcmp(k, "box_count") == 0)
                    cfg.crashBoxCount = std::atoi(v);
                else if (std::strcmp(k, "box_mass") == 0)
                    cfg.crashBoxMass = static_cast<float>(std::atof(v));
            }
        }
        std::fclose(f);
        return cfg;
    }
};
