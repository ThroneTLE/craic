#!/bin/bash
# =============================================================================
# 工作空间路径迁移脚本
# 用法: bash migrate_ws.sh <新工作空间名>
# 示例: bash migrate_ws.sh W00ZTK
#       将旧工作空间路径 craic / demo 统一替换为 W00ZTK
# =============================================================================

set -e

if [ -z "$1" ]; then
    echo "错误: 请提供新工作空间名称"
    echo "用法: bash $0 <新工作空间名>"
    echo "示例: bash $0 W00ZTK"
    exit 1
fi

NEW_WS="$1"
WS_ROOT="/home/abot/${NEW_WS}"

if [ ! -d "$WS_ROOT" ]; then
    echo "警告: 工作空间目录 ${WS_ROOT} 不存在，继续执行但路径可能无效"
fi

# 获取脚本所在目录作为工作空间根目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "  工作空间路径迁移脚本"
echo "  旧 -> 新: craic/demo -> ${NEW_WS}"
echo "  工作空间根目录: ${WS_ROOT}"
echo "=========================================="

# 旧工作空间名列表
OLD_NAMES=("craic" "demo")

# 需要修改的文件列表（相对于工作空间根目录）
FILES=(
    # === 启动脚本 ===
    "demo_carto.sh"

    # === VSCode 配置 ===
    ".vscode/settings.json"

    # === CLAUDE.md ===
    "CLAUDE.md"

    # === robot_slam 包 ===
    "src/robot_slam/scripts/start.py"
    "src/robot_slam/scripts/detect.py"
    "src/robot_slam/scripts/decoder.py"
    "src/robot_slam/scripts/paraformer_wake.py"
    "src/robot_slam/scripts/main.py"
    "src/robot_slam/scripts/main copy.py"
    "src/robot_slam/scripts/main copy 2.py"
    "src/robot_slam/maps/my_lab.yaml"

    # === ocr_detect 包 ===
    "src/ocr_detect/scripts/extract.py"
    "src/ocr_detect/scripts/ocr_detect.py"
    "src/ocr_detect/scripts/ocr.py"
    "src/ocr_detect/launch/ocr.launch"

    # === abot_vlm 包 ===
    "src/abot_vlm/scripts/identify_service.py"
    "src/abot_vlm/scripts/calculate.py"

    # === abot_find 包 ===
    "src/abot_find/launch/find_object_2d.launch"
)

modified=0
skipped=0

for file in "${FILES[@]}"; do
    if [ ! -f "$file" ]; then
        echo "  [跳过] 文件不存在: $file"
        ((skipped++)) || true
        continue
    fi

    changed=false
    for old_name in "${OLD_NAMES[@]}"; do
        # 替换 /home/abot/<old_name>/ -> /home/abot/<NEW_WS>/
        if grep -q "/home/abot/${old_name}/" "$file" 2>/dev/null; then
            sed -i "s|/home/abot/${old_name}/|/home/abot/${NEW_WS}/|g" "$file"
            changed=true
        fi
        # 替换 ~/<old_name>/ -> ~/<NEW_WS>/
        if grep -q "~/${old_name}/" "$file" 2>/dev/null; then
            sed -i "s|~/${old_name}/|~/${NEW_WS}/|g" "$file"
            changed=true
        fi
    done

    if $changed; then
        echo "  [已修改] $file"
        ((modified++)) || true
    else
        echo "  [无需改] $file"
        ((skipped++)) || true
    fi
done

echo "=========================================="
echo "  完成: 修改 ${modified} 个文件, 跳过 ${skipped} 个文件"
echo "=========================================="
