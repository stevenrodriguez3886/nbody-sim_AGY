To compile for code coverage use these commands

```bash
cmake -S . -B build-coverage \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"
``` 

```bash
cmake --build build-coverage -j"$(nproc)"
```

run the tests
```bash
ctest --test-dir build-coverage --output-on-failure
```

capture lcov output

```bash
lcov --capture \
  --directory build-coverage \
  --output-file coverage.info \
  --include '/home/stephent/SWEN6936/nbody-sim_AGY/physics/*' \
  --include '/home/stephent/SWEN6936/nbody-sim_AGY/app/*'
```

generate the html report
```bash
genhtml coverage.info \
  --output-directory coverage-html \
  --title "nbody-sim_AGY Test Coverage"
```