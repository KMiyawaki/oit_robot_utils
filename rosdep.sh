#!/bin/bash

function main(){
    local -r SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
    cd "$SCRIPT_DIR"
    
    # パッケージルートから2つ上がワークスペースのルート (~/ros2_ws)
    local -r WS_ROOT=$(cd "$SCRIPT_DIR/../.." && pwd)
    local -r SRC_DIR="$WS_ROOT/src"
    
    echo "==> Updating rosdep database..."
    rosdep update
    
    echo "==> Scanning and installing workspace dependencies package by package..."
    
    # src ディレクトリ直下にある各パッケージを走査
    for pkg_path in "$SRC_DIR"/*/; do
        if [ -f "${pkg_path}package.xml" ]; then
            pkg_name=$(basename "$pkg_path")
            echo "--------------------------------------------------"
            echo "Processing package: $pkg_name"
            echo "--------------------------------------------------"
            
            # 各パッケージごとに rosdep install を実行（エラーが出ても停止せず続行）
            rosdep install --from-paths "$pkg_path" --ignore-src -y --rosdistro ${ROS_DISTRO:-jazzy} || {
                echo "[WARNING] Failed to resolve some dependencies for $pkg_name. Skipping..." >&2
            }
        fi
    done
    
    echo "=================================================="
    echo "==> Finished dependency installation process!"
}

main "$@"
