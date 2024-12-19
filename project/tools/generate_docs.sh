#!/usr/bin/env bash

readonly doxyfile="$(dirname "$0")/../Doxyfile"
readonly docs_dir="$(dirname "$0")/../../build/docs"

mkdir -p "${docs_dir}"
doxygen "${doxyfile}"
