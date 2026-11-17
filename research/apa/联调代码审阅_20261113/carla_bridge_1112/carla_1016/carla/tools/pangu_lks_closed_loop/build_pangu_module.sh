#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
FRAMEWORK_ROOT="${PANGU_FRAMEWORK_ROOT:-/home/aiden/gaasd_versions/gaasd-2.7.0.5/home/gaasd_server/panguFramework/pangu_gaasd}"
BUILD_ROOT="${PANGU_BUILD_ROOT:-${HOME}/.cache/gaasd-pangu/lks2_codegen_build}"
IMAGE="${PANGU_DOCKER_IMAGE:-docker.cbdes.cn:8080/cbdes/x86:pangu-2.0.5}"
BUILD_JOBS="${PANGU_BUILD_JOBS:-4}"
MODULE_SOURCE="${ROOT_DIR}/tools/pangu_lks_closed_loop/LKSClosedLoopModule"
GENERATED_SOURCE="${ROOT_DIR}/project/lks2/icvos/src/temp_codegen_output"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

mkdir -p "${BUILD_ROOT}/modules"
cp "${FRAMEWORK_ROOT}/CMakeLists.txt" "${BUILD_ROOT}/CMakeLists.txt"
cp "${FRAMEWORK_ROOT}/version.hpp" "${BUILD_ROOT}/version.hpp"
cp -a "${FRAMEWORK_ROOT}/script" "${BUILD_ROOT}/"
cp "${FRAMEWORK_ROOT}/modules/CMakeLists.txt" "${BUILD_ROOT}/modules/CMakeLists.txt"
ln -sfn "${FRAMEWORK_ROOT}/dependencies" "${BUILD_ROOT}/dependencies"
rm -rf "${BUILD_ROOT}/configs"
cp -a "${FRAMEWORK_ROOT}/configs" "${BUILD_ROOT}/configs"
rm -rf "${BUILD_ROOT}/modules/LKSModule"
cp -a "${MODULE_SOURCE}" "${BUILD_ROOT}/modules/LKSModule"

if [[ "${SKIP_PANGU_BUILD:-0}" != "1" ]]; then
  docker run --rm --net=host \
    -v /home:/home -v /data:/data -v /tmp:/tmp \
    "${IMAGE}" /bin/bash -lc "
      set -e
      cd '${BUILD_ROOT}'
      source dependencies/thirdparty/X86/setup.bash >/dev/null 2>&1 || true
      cmake -S . -B build_lks \
        -DOPEN_WHITE_LIST=ON \
        -DWHITE_LIST=';LKSModule:' \
        -DRUNTIME_OS=ICVOS \
        -DBUILD_TARGET=PC \
        -DLKS_GENERATED_DIR='${GENERATED_SOURCE}'
      cmake --build build_lks -j'${BUILD_JOBS}'
      mkdir -p '${BUILD_ROOT}/install'
      ln -sfn '${BUILD_ROOT}/configs' '${BUILD_ROOT}/install/conf'
      cmake --install build_lks
      # The installed conf symlink points back to the framework source tree.
      # Runtime scenarios must use this build's packaged configuration instead.
      ln -sfn '${BUILD_ROOT}/install/image/conf' '${BUILD_ROOT}/install/conf'
    "
fi

# CMake install runs as root in the build container. Return ownership to the
# invoking user before completing the scenario-specific runtime packaging.
docker run --rm -v /home:/home -v /data:/data -v /tmp:/tmp \
  "${IMAGE}" chown -R "${HOST_UID}:${HOST_GID}" "${BUILD_ROOT}/install"
chmod -R u+rwX "${BUILD_ROOT}/install"

mkdir -p "${BUILD_ROOT}/install/conf/app_module"
mkdir -p "${BUILD_ROOT}/install/conf/node_module/LKSModule"
cp "${MODULE_SOURCE}/conf/"*.pt \
   "${BUILD_ROOT}/install/conf/node_module/LKSModule/"
cat >"${BUILD_ROOT}/install/conf/app_module/app_empty.pt" <<'EOF'
soc {
  process {
    node {
      node_name: "LKSModule"
      module_name: "LKSModule"
      path: "conf/node_module/LKSModule/LKSModule.pt"
      bind_cpu: "-1"
      node_priority: 0
    }
    process_name: "LKSModule"
    log { level: INFO to_file: OFF to_remote: OFF to_console: ON sys_log: OFF }
  }
  process {
    node {
      node_name: "ZmqBridgeModule"
      module_name: "ZmqBridgeModule"
      path: "conf/node_module/LKSModule/ZmqBridgeModule.pt"
      bind_cpu: "-1"
      node_priority: 0
    }
    process_name: "ZmqBridgeModule"
    log { level: INFO to_file: OFF to_remote: OFF to_console: ON sys_log: OFF }
  }
  soc_name: "soc1"
}
EOF

runtime_root="${BUILD_ROOT}/install"
mkdir -p "${runtime_root}/image/lib" \
         "${runtime_root}/conf/app_module" \
         "${runtime_root}/conf/node_module/LKSModule" \
         "${runtime_root}/conf/global_conf"
cp "${runtime_root}/lib/libLKSModule.so" \
   "${runtime_root}/lib/libLKSModule_core.so" \
   "${runtime_root}/lib/libLKSModule_conf_pb.so" \
   "${runtime_root}/lib/libZmqBridgeModule.so" \
   "${runtime_root}/image/lib/"

global_module="${runtime_root}/conf/global_conf/global_module.pt"
global_com="${runtime_root}/conf/global_conf/global_com.pt"
if ! grep -q 'name: "LKSModule"' "${global_module}"; then
    cat >>"${global_module}" <<'EOF'
modules {
  name: "LKSModule"
  config_type: "pangu.modules.LKSModuleConfig"
}
EOF
fi
if ! grep -q 'name: "ZmqBridgeModule"' "${global_module}"; then
    cat >>"${global_module}" <<'EOF'
modules {
  name: "ZmqBridgeModule"
  config_type: "pangu.modules.ZmqBridgeModuleConfig"
}
EOF
fi
if ! grep -q 'channel_name: "lks_input"' "${global_com}"; then
    cat >>"${global_com}" <<'EOF'
channels {
  channel_name: "lks_input"
  channel_type: "pangu.modules.LksInput"
  mdw_type: INNER_SHM_DDS
  max_thread_num: 1
  queue_buffer_size: 1
  keep_latest: true
  shm_config { type: FastDDS block_size: SHM_10K }
}
channels {
  channel_name: "lks_output"
  channel_type: "pangu.modules.LksOutput"
  mdw_type: INNER_SHM_DDS
  max_thread_num: 1
  queue_buffer_size: 1
  keep_latest: true
  shm_config { type: FastDDS block_size: SHM_10K }
}
EOF
fi

echo "[LKS build] install ready: ${BUILD_ROOT}/install"
