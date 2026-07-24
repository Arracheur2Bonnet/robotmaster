#!/usr/bin/env bash
# Sauvegarde des fichiers sources actifs dans saves/YYYY-MM-DD-HH-MM/
# Usage : bash shortcuts/save_session.sh [message optionnel]
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAMP=$(date +"%Y-%m-%d-%H-%M")
DEST="$ROOT/saves/$STAMP"
mkdir -p "$DEST"

# Fichiers à sauvegarder (ajouter ici si d'autres fichiers critiques sont créés)
FILES=(
    "shortcuts/carolus_launcher.py"
    "shortcuts/cam_view_helper.py"
    "shortcuts/map_editor.py"
    "carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py"
    "carolus_ws/src/carolus_node/launch/testcarolus.launch"
    "carolus_ws/src/CMakeLists.txt"
    "carolus_ws/src/libuvgs_astrobee/CMakeLists.txt"
    "carolus_ws/src/ff_msgs/CMakeLists.txt"
    "carolus_ws/src/robomaster_cam/CMakeLists.txt"
    "carolus_ws/src/carolus_node/CMakeLists.txt"
)

for f in "${FILES[@]}"; do
    src="$ROOT/$f"
    if [ -f "$src" ]; then
        dst="$DEST/$(echo "$f" | tr '/' '__')"
        cp "$src" "$dst"
        echo "  saved: $f"
    fi
done

# Note optionnelle
if [ -n "$1" ]; then
    echo "$1" > "$DEST/NOTE.txt"
    echo "  note: $1"
fi

echo ""
echo "Sauvegarde -> $DEST"
echo "Pour restaurer un fichier: cp saves/YYYY-MM-DD-HH-MM/shortcuts__carolus_launcher.py shortcuts/carolus_launcher.py"
