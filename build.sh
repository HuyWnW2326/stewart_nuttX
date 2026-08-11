#!/bin/bash

TARGET=${1:-stewart}
BOARD=stm32f411e-disco
REPO_ROOT=${PWD}

echo ">>> Building payload: $TARGET"
echo ">>> Using board:     $BOARD"

mkdir -p apps/external
cat > apps/external/CMakeLists.txt << CMAKE
add_subdirectory(${REPO_ROOT}/common common_build)
add_subdirectory(${REPO_ROOT}/payloads/${TARGET} payload_build)
CMAKE

BUILD_DIR=build/${BOARD}/${TARGET}

if [ ! -f ${BUILD_DIR}/build.ninja ]; then
  echo ">>> Configuring..."
  cmake -S nuttx \
    -B ${BUILD_DIR} \
    -DBOARD_CONFIG=${REPO_ROOT}/boards/${BOARD}/configs/$TARGET \
    -DNUTTX_APPS_DIR=${REPO_ROOT}/apps \
    -DREPO_ROOT=${REPO_ROOT} \
    -GNinja
fi

# Build
cmake --build ${BUILD_DIR}