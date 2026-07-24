# Jubal — Audio System

**Status**: Placeholder — Implementation begins in Stage 7.

## Purpose

`jubal` provides the audio subsystem — playback, mixing, spatial audio, effects, and streaming.

## Planned Contents

- **core/** — Audio system core and initialization
- **backend/** — Audio API abstraction (OpenAL, WASAPI, etc.)
- **mixer/** — Audio mixing and channel management
- **spatial/** — 3D positional audio
- **effects/** — Audio effects and filters
- **streams/** — Streaming audio from disk
- **resources/** — Audio resource management

## Dependencies

- `foundation` — Core utilities, memory, threading
- Rust components for audio format decoding (via workspace)
