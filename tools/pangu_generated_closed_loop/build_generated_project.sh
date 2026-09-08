#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <newaccpro4|lks3>" >&2
  exit 2
fi

PROJECT_NAME="$1"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FRAMEWORK_ROOT="${PANGU_FRAMEWORK_ROOT:-${HOME}/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd}"
IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:latest}"
BUILD_JOBS="${PANGU_BUILD_JOBS:-4}"
PROJECT_SOURCE="${PANGU_PROJECT_SOURCE:-${ROOT_DIR}/project/${PROJECT_NAME}/icvos/src}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

case "${PROJECT_NAME}" in
  newaccpro4)
    MODULE_NAME="module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43"
    INPUT_MODULE="AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
    OUTPUT_MODULE="AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43"
    DEFAULT_BUILD_ROOT="${HOME}/.cache/gaasd-pangu/newaccpro4_codegen_build"
    ;;
  lks3)
    MODULE_NAME="module_empty_a807407f_21e6_442d_a6e4_13a788ec666d"
    INPUT_MODULE="LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d"
    OUTPUT_MODULE="LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d"
    DEFAULT_BUILD_ROOT="${HOME}/.cache/gaasd-pangu/lks3_codegen_build"
    ;;
  *)
    echo "Unsupported project: ${PROJECT_NAME}" >&2
    exit 2
    ;;
esac

BUILD_ROOT="${PANGU_BUILD_ROOT:-${DEFAULT_BUILD_ROOT}}"
MODULE_SOURCE="${PROJECT_SOURCE}/modules"
RUNTIME_ROOT="${BUILD_ROOT}/install"

require_file() {
  if [[ ! -f "$1" ]]; then
    echo "[Generated build] missing file: $1" >&2
    exit 1
  fi
}

require_file "${FRAMEWORK_ROOT}/CMakeLists.txt"
require_file "${FRAMEWORK_ROOT}/modules/CMakeLists.txt"
require_file "${PROJECT_SOURCE}/configs/app_module/app_empty.pt"
require_file "${MODULE_SOURCE}/module_empty/${MODULE_NAME}/CMakeLists.txt"

mkdir -p "${BUILD_ROOT}/modules"
cp "${FRAMEWORK_ROOT}/CMakeLists.txt" "${BUILD_ROOT}/CMakeLists.txt"
cp "${FRAMEWORK_ROOT}/version.hpp" "${BUILD_ROOT}/version.hpp"
rm -rf "${BUILD_ROOT}/script"
cp -a "${FRAMEWORK_ROOT}/script" "${BUILD_ROOT}/"
cp "${FRAMEWORK_ROOT}/modules/CMakeLists.txt" "${BUILD_ROOT}/modules/CMakeLists.txt"
ln -sfn "${FRAMEWORK_ROOT}/dependencies" "${BUILD_ROOT}/dependencies"
rm -rf "${BUILD_ROOT}/configs"
cp -a "${PROJECT_SOURCE}/configs" "${BUILD_ROOT}/configs"
find "${BUILD_ROOT}/modules" -mindepth 1 -maxdepth 1 \
  ! -name CMakeLists.txt -exec rm -rf {} +
cp -a "${MODULE_SOURCE}/." "${BUILD_ROOT}/modules/"
rm -rf "${BUILD_ROOT}/build_${PROJECT_NAME}"

echo "[Generated build] project=${PROJECT_NAME} source=${PROJECT_SOURCE} image=${IMAGE}"
docker run --rm --net=host \
  -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" /bin/bash -lc "
    set -e
    cd '${BUILD_ROOT}'
    source dependencies/thirdparty/X86/setup.bash >/dev/null 2>&1 || true
    cmake -S . -B 'build_${PROJECT_NAME}' \
      -DOPEN_WHITE_LIST=OFF \
      -DRUNTIME_OS=ICVOS \
      -DBUILD_TARGET=PC
    cmake --build 'build_${PROJECT_NAME}' -j'${BUILD_JOBS}'
    rm -rf '${RUNTIME_ROOT}'
    cmake --install 'build_${PROJECT_NAME}'
  "

docker run --rm -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" chown -R "${HOST_UID}:${HOST_GID}" "${RUNTIME_ROOT}"
chmod -R u+rwX "${RUNTIME_ROOT}"

mkdir -p "${RUNTIME_ROOT}/image/lib" \
         "${RUNTIME_ROOT}/image/conf/app_module" \
         "${RUNTIME_ROOT}/image/conf/node_module/${MODULE_NAME}" \
         "${RUNTIME_ROOT}/image/conf/global_conf"

for library in \
  "lib${MODULE_NAME}.so" \
  "lib${MODULE_NAME}_core.so" \
  "lib${MODULE_NAME}_conf_pb.so" \
  "lib${INPUT_MODULE}.so" \
  "lib${OUTPUT_MODULE}.so"; do
  require_file "${RUNTIME_ROOT}/lib/${library}"
  cp "${RUNTIME_ROOT}/lib/${library}" "${RUNTIME_ROOT}/image/lib/"
done

cp "${PROJECT_SOURCE}/configs/app_module/app_empty.pt" \
   "${RUNTIME_ROOT}/image/conf/app_module/"
cp -a "${RUNTIME_ROOT}/conf/node_module/${MODULE_NAME}/." \
      "${RUNTIME_ROOT}/image/conf/node_module/${MODULE_NAME}/"
cp "${PROJECT_SOURCE}/configs/global_conf/global_module.pt" \
   "${PROJECT_SOURCE}/configs/global_conf/global_com.pt" \
   "${PROJECT_SOURCE}/configs/global_conf/icvos_machine.pt" \
   "${PROJECT_SOURCE}/configs/global_conf/tf.pt" \
   "${RUNTIME_ROOT}/image/conf/global_conf/"
cp "${PROJECT_SOURCE}/configs/space_name" \
   "${RUNTIME_ROOT}/image/conf/space_name"

rm -rf "${RUNTIME_ROOT}/conf"
ln -s "${RUNTIME_ROOT}/image/conf" "${RUNTIME_ROOT}/conf"

require_file "${RUNTIME_ROOT}/setup.bash"
require_file "${RUNTIME_ROOT}/run.sh"
require_file "${RUNTIME_ROOT}/image/conf/app_module/app_empty.pt"

docker run --rm -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" chown -R "${HOST_UID}:${HOST_GID}" "${BUILD_ROOT}"
chmod -R u+rwX "${BUILD_ROOT}"

echo "[Generated build] ready: ${RUNTIME_ROOT}"
