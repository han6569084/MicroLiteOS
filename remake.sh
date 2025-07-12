#!/bin/bash

rm -rf build/out
mkdir -p build/out
cmake -B build/out -DCMAKE_TOOLCHAIN_FILE=build/build_rules/toolchain.cmake
cmake --build build/out
