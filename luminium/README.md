# Luminium — Rendering Engine

**Status**: Placeholder — Implementation begins in Stage 5.

## Purpose

`luminium` is the rendering engine subsystem. It manages the graphics pipeline, material system, lighting, and post-processing.

## Planned Contents

- **core/** — Renderer core, initialization, resource management
- **renderer/** — Render pipeline, draw call management
- **materials/** — Material system and shader integration
- **lights/** — Light types, shadow mapping
- **cameras/** — Camera types and projection
- **postprocess/** — Post-processing effects
- **textures/** — Texture loading and management
- **meshes/** — Mesh and geometry handling
- **shaders/** — Shader compilation and management
- **backend/** — Graphics API abstraction (Vulkan initial target)
- **framegraph/** — Render pass management and resource aliasing

## Dependencies

- `foundation` — Core utilities, math, memory
- `cambyses` — ECS integration for rendering components
