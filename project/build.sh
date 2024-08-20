#!/usr/bin/env bash

cmake -S "${APP_DIR}/project/perlin_lk" -B "${APP_DIR}/build"
cmake --build "${APP_DIR}/build"
