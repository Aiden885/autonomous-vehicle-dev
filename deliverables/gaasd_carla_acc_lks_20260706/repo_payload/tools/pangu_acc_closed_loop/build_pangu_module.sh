#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FRAMEWORK_ROOT="${PANGU_FRAMEWORK_ROOT:-/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd}"
BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/newaccpro3_codegen_build}"
IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5}"
BUILD_JOBS="${PANGU_BUILD_JOBS:-4}"
PROJECT_SOURCE="${ROOT_DIR}/project/newaccpro3/icvos/src"
MODULE_SOURCE="${PROJECT_SOURCE}/modules"
GENERATED_SOURCE="${PROJECT_SOURCE}/temp_codegen_output"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

mkdir -p "${BUILD_ROOT}/modules"
cp "${FRAMEWORK_ROOT}/CMakeLists.txt" "${BUILD_ROOT}/CMakeLists.txt"
cp "${FRAMEWORK_ROOT}/version.hpp" "${BUILD_ROOT}/version.hpp"
rm -rf "${BUILD_ROOT}/script"
cp -a "${FRAMEWORK_ROOT}/script" "${BUILD_ROOT}/"
cp "${FRAMEWORK_ROOT}/modules/CMakeLists.txt" "${BUILD_ROOT}/modules/CMakeLists.txt"
ln -sfn "${FRAMEWORK_ROOT}/dependencies" "${BUILD_ROOT}/dependencies"
rm -rf "${BUILD_ROOT}/configs"
cp -a "${PROJECT_SOURCE}/configs" "${BUILD_ROOT}/configs"
find "${BUILD_ROOT}/modules" -mindepth 1 -maxdepth 1 ! -name CMakeLists.txt -exec rm -rf {} +
cp -a "${MODULE_SOURCE}/." "${BUILD_ROOT}/modules/"
ln -sfn "${GENERATED_SOURCE}" "${BUILD_ROOT}/temp_codegen_output"

docker run --rm --net=host \
  -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" /bin/bash -lc "
    set -e
    cd '${BUILD_ROOT}'
    source dependencies/thirdparty/X86/setup.bash >/dev/null 2>&1 || true
    cmake -S . -B build_acc \
      -DOPEN_WHITE_LIST=OFF \
      -DRUNTIME_OS=ICVOS \
      -DBUILD_TARGET=PC
    cmake --build build_acc -j'${BUILD_JOBS}'
    rm -rf '${BUILD_ROOT}/install'
    cmake --install build_acc
  "

docker run --rm -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" chown -R "${HOST_UID}:${HOST_GID}" "${BUILD_ROOT}/install"
chmod -R u+rwX "${BUILD_ROOT}/install"

runtime_root="${BUILD_ROOT}/install"
mkdir -p "${runtime_root}/image/lib" \
         "${runtime_root}/image/conf/app_module" \
         "${runtime_root}/image/conf/node_module/module_empty" \
         "${runtime_root}/image/conf/global_conf"

cp "${runtime_root}/lib/libmodule_empty.so" \
   "${runtime_root}/lib/libmodule_empty_core.so" \
   "${runtime_root}/lib/libmodule_empty_conf_pb.so" \
   "${runtime_root}/lib/libZmqBridgeModule.so" \
   "${runtime_root}/image/lib/"
cp "${PROJECT_SOURCE}/configs/app_module/app_empty.pt" \
   "${runtime_root}/image/conf/app_module/"
cp -a "${runtime_root}/conf/node_module/module_empty/." \
   "${runtime_root}/image/conf/node_module/module_empty/"
cp "${PROJECT_SOURCE}/configs/global_conf/global_module.pt" \
   "${PROJECT_SOURCE}/configs/global_conf/global_com.pt" \
   "${PROJECT_SOURCE}/configs/global_conf/tf.pt" \
   "${runtime_root}/image/conf/global_conf/"
if grep -q 'local_machine_name:' "${PROJECT_SOURCE}/configs/global_conf/icvos_machine.pt"; then
  machine_config="${PROJECT_SOURCE}/configs/global_conf/icvos_machine.pt"
else
  machine_config="${FRAMEWORK_ROOT}/configs/global_conf/icvos_machine.pt"
fi
cp "${machine_config}" "${runtime_root}/image/conf/global_conf/icvos_machine.pt"

rm -rf "${runtime_root}/conf"
ln -s "${runtime_root}/image/conf" "${runtime_root}/conf"

for required in \
  "${runtime_root}/setup.bash" \
  "${runtime_root}/run.sh" \
  "${runtime_root}/lib/libZmqBridgeModule.so" \
  "${runtime_root}/image/conf/app_module/app_empty.pt"; do
  if [[ ! -f "${required}" ]]; then
    echo "[ACC build] missing runtime file: ${required}" >&2
    exit 1
  fi
done

echo "[ACC build] persistent install ready: ${runtime_root}"
