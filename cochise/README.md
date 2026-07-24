# Cochise — Asset Pipeline

**Status**: Placeholder — Implementation begins in Stage 6.

## Purpose

`cochise` manages the asset pipeline — loading, caching, importing, and managing all game assets (meshes, textures, audio, animations, etc.).

## Planned Contents

- **core/** — Asset system core, handles, registries
- **loaders/** — Asset loading implementations
- **formats/** — File format support
- **pipeline/** — Import pipeline and conversion
- **cache/** — Asset caching and memory management
- **importers/** — Asset importers for various formats

## Dependencies

- `foundation` — Core utilities, memory, filesystem
- Rust components for format-specific parsing (via workspace)
