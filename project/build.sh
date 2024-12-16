#!/usr/bin/env bash

cmake -S "${HOME_DIR}/project/perlin_lk" -B "${HOME_DIR}/build"
cmake --build "${HOME_DIR}/build"
