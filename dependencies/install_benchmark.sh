#!/bin/bash

# Exit when an error happens instead of continue.
set -e

# Default values for flags.
NUM_JOBS=24
GENERATOR="Unix Makefiles"

# Set the working directory.
WORKING_DIR="$(pwd)/benchmark"
if [ -d "$WORKING_DIR" ]; then
    sudo rm -r $WORKING_DIR
fi
mkdir -p $WORKING_DIR

cd $WORKING_DIR
git clone https://github.com/google/benchmark.git
cd benchmark
mkdir build
cmake -G"$GENERATOR" -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$WORKING_DIR/benchmark_install -S . -B "build"
cmake --build "build" --config Release -j$NUM_JOBS
cmake --build "build" --config Release --target install
