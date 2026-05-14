#!/bin/bash

function main(){
    local -r PACKAGE_DIR=$(cd "$(dirname "$0")" && pwd)
    local -r SRC_DIR=$(dirname "${PACKAGE_DIR}")
    local -r WS_ROOT=$(dirname "${SRC_DIR}")
    
    local BUILD_ALL=0
    local REBUILD=0
    local CLEAN=0
    local DEBUG=0
    
    while getopts arcd opt
    do
        case ${opt} in
            "a" ) BUILD_ALL=1 ;;
            "r" ) REBUILD=1 ;;
            "c" ) CLEAN=1 ;;
            "d" ) DEBUG=1 ;;
        esac
    done
    
    local -r NODE_NAME=$(basename "${PACKAGE_DIR}")
    local B_TYPE="Release"
    if [ ${DEBUG} -eq 1 ]; then
        B_TYPE="Debug"
    fi
    
    local BUILD_OPT="--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${B_TYPE}"
    local BUILD_DIR="${WS_ROOT}/build"
    local INSTALL_DIR="${WS_ROOT}/install"
    
    if [ ${BUILD_ALL} -eq 0 ]; then
        BUILD_OPT="${BUILD_OPT} --packages-select ${NODE_NAME}"
        BUILD_DIR="${BUILD_DIR}/${NODE_NAME}"
        INSTALL_DIR="${INSTALL_DIR}/${NODE_NAME}"
    fi
    
    if [ ${REBUILD} -eq 1 ] || [ ${CLEAN} -eq 1 ]; then
        echo "Cleaning ROS 2 build/install for: ${NODE_NAME}"
        rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
    fi
    
    if [ ${CLEAN} -eq 0 ]; then
        cd "${WS_ROOT}" && colcon build ${BUILD_OPT}
    fi
}

main "$@"
