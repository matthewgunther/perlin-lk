#!/usr/bin/env bash

clang-format -i $(find ./project/perlin_lk -name "*.cpp" -o -name "*.h")
