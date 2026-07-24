# Contributing to PreonX

Thank you for your interest in contributing to PreonX. This document provides guidelines and instructions for contributing.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Coding Standards](#coding-standards)
- [Commit Guidelines](#commit-guidelines)
- [Pull Request Process](#pull-request-process)
- [Reporting Issues](#reporting-issues)
- [License](#license)

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code.

## Getting Started

### Prerequisites

- CMake 3.24+
- Clang 17+ or GCC 14+
- Rust 1.75+
- Python 3.11+ with `uv`
- Git

### Setting Up the Development Environment

1. Fork and clone the repository:
   ```bash
   git clone https://github.com/<your-username>/PreonX.git
   cd PreonX
   ```

2. Configure the project:
   ```bash
   cmake -B build -DCMAKE_BUILD_TYPE=Debug -DPREONX_WARNINGS_AS_ERRORS=ON
   ```

3. Build:
   ```bash
   cmake --build build --parallel
   ```

4. Set up Python tooling:
   ```bash
   pip install uv
   uv sync
   ```

5. Verify the setup:
   ```bash
   uv run pre-commit run --all-files
   ```

## Development Workflow

1. Create a feature branch from `main`:
   ```bash
   git checkout -b feature/my-feature main
   ```

2. Make your changes following the coding standards.

3. Run formatting and linting:
   ```bash
   uv run clang-format -i <files>
   uv run ruff format .
   cargo fmt --all
   cargo clippy --all-targets -- -D warnings
   ```

4. Build and test:
   ```bash
   cmake --build build --parallel
   ctest --test-dir build --output-on-failure
   ```

5. Commit with a descriptive message following the [commit guidelines](#commit-guidelines).

6. Push and create a pull request.

## Coding Standards

### C++

- Standard: C++20
- Formatting: `clang-format` with project configuration
- Linting: `clang-tidy` with project configuration
- See [Coding Standards](docs/development/coding-standards.md) for full details.

### Rust

- Edition: 2021
- Formatting: `rustfmt` with project configuration
- Linting: `clippy` with project configuration
- See [Coding Standards](docs/development/coding-standards.md) for full details.

### Python

- Formatting: `ruff format`
- Linting: `ruff check`
- Type checking: `mypy`
- See [Coding Standards](docs/development/coding-standards.md) for full details.

## Commit Guidelines

### Format

```
<type>(<scope>): <description>

[optional body]

[optional footer(s)]
```

### Types

- **feat**: A new feature
- **fix**: A bug fix
- **docs**: Documentation changes
- **style**: Code style changes (formatting, missing semicolons, etc.)
- **refactor**: Code refactoring without functionality changes
- **perf**: Performance improvements
- **test**: Adding or correcting tests
- **chore**: Build process, CI, or tooling changes
- **ci**: CI configuration changes

### Examples

```
feat(physics): add rigid body dynamics simulation

fix(rendering): correct shadow map offset calculation

docs(architecture): update subsystem interaction diagram

chore(build): add CMake preset for sanitizers
```

### Rules

- Use the imperative mood ("add feature", not "added feature")
- Do not capitalize the first letter of the description
- No period at the end of the description
- Keep the subject line to 72 characters or fewer
- Use the body to explain _what_ and _why_, not _how_

## Pull Request Process

1. **Fill out the PR template** completely.
2. **Ensure CI passes** — all checks must be green.
3. **Add tests** for new functionality (where applicable).
4. **Update documentation** if the change affects the public API or developer guides.
5. **Request review** from at least one maintainer.
6. **Address review feedback** — push additional commits as needed.
7. **Squash and merge** once approved (maintainer will handle the merge).

### PR Size Guidelines

- Keep PRs focused and reasonably sized.
- Large changes should be broken into smaller, sequential PRs.
- If a PR exceeds ~500 lines of changes, consider splitting it.

## Reporting Issues

- Use the [GitHub Issue Tracker](https://github.com/Primeeex/PreonX/issues).
- Use the appropriate issue template (bug report or feature request).
- Provide as much detail as possible.
- For security issues, follow the process in [SECURITY.md](SECURITY.md).

## License

By contributing to PreonX, you agree that your contributions will be licensed under the [MIT License](LICENSE).
