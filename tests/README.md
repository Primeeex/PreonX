# Tests

Test suites for PreonX.

## Structure

- **unit/** — Unit tests for individual components
- **integration/** — Integration tests for subsystem interactions
- **e2e/** — End-to-end tests for full engine workflows

## Running Tests

```bash
# Using CMake
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DPREONX_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Current Status

No tests have been implemented yet. Test infrastructure will be populated as subsystems are built.
