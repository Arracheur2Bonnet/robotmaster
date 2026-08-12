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
    # 2026-07-28 : ajoutes apres avoir du les sauvegarder A LA MAIN deux fois
    # dans la meme session (docking). Tout fichier source activement modifie
    # doit etre ici, sinon la sauvegarde donne une fausse impression de filet.
    "carolus_ws/src/robomaster_cam/scripts/beacon_docking.py"
    "carolus_ws/src/carolus_node/scripts/beacon_absolute_pose.py"
    # 2026-08-11 : ajoute apres avoir constate qu'il en etait absent au moment
    # meme ou il devenait le fichier central de la question quaternion (mail
    # Hector 2026-08-11). Meme regle qu'en 2026-07-28 : un fichier source
    # activement modifie doit etre dans cette liste, sans exception.
    "carolus_ws/src/carolus_node/scripts/carolus_tf_broadcaster.py"
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
