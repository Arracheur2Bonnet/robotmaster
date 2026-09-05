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
    # 2026-08-14 : ajoute apres avoir constate qu'il en etait absent au moment
    # meme ou il devenait le fichier touche (tuning min_area pour la portee
    # 8m, demande d'Hector) -- meme regle que ci-dessus, sans exception.
    "carolus_ws/src/carolus_node/config/robomaster_s1.yaml"
    # 2026-09-04 : meme trou, meme regle -- robomaster_s1.yaml etait suivi
    # mais ses deux voisins dans le meme dossier ne l'etaient pas, constate en
    # patchant BUG-138 (fov/plumb_bob mismatch) dans logitech_1080p.yaml.
    "carolus_ws/src/carolus_node/config/logitech_1080p.yaml"
    "carolus_ws/src/carolus_node/config/robomaster_s1_longrange.yaml"
    "carolus_ws/src/CMakeLists.txt"
    "carolus_ws/src/libuvgs_astrobee/CMakeLists.txt"
    "carolus_ws/src/ff_msgs/CMakeLists.txt"
    "carolus_ws/src/robomaster_cam/CMakeLists.txt"
    "carolus_ws/src/carolus_node/CMakeLists.txt"
    # 2026-08-18 : ajoute apres avoir constate qu'il en etait absent au moment
    # meme ou il devenait le fichier central de l'extraction du coeur Carolus
    # hors ROS (demande Hector, "plus important que le reste du plan") --
    # meme regle que ci-dessus, sans exception. Les nouveaux fichiers
    # beacon_detector.hpp/.cpp seront ajoutes ici des leur creation.
    "carolus_ws/src/libuvgs_astrobee/src/carolus_astrobee.cpp"
    "carolus_ws/src/libuvgs_astrobee/src/ceresP4P.cpp"
    "carolus_ws/src/libuvgs_astrobee/src/pose_est.cpp"
    "carolus_ws/src/libuvgs_astrobee/src/beacon_detector.cpp"
    "carolus_ws/src/libuvgs_astrobee/include/carolus_node/beacon_detector.hpp"
    # 2026-08-18 (2): the new ROS2 wrapper package, added the moment it was
    # created rather than after the fact -- same standing rule as above.
    # 2026-08-19 : deplace carolus_ws/src/ -> racine du depot (un paquet
    # ament dans un espace source catkin casse catkin_make pour tout le
    # workspace, "non-homogeneous workspace").
    # 2026-08-20 (2) : deplace une DEUXIEME fois, racine -> raspberry5-carolus-ros2/,
    # et rendu autonome (le coeur C++ est copie ici, plus de reference
    # relative vers carolus_ws/). Chaque deplacement casse silencieusement
    # cette liste si elle n'est pas mise a jour le jour meme -- une entree
    # pointant vers un fichier absent est copiee en rien du tout, la
    # sauvegarde parait reussir. Troisieme occurrence du meme pattern.
    "raspberry5-carolus-ros2/carolus_ros2/src/carolus_node_ros2.cpp"
    "raspberry5-carolus-ros2/carolus_ros2/CMakeLists.txt"
    "raspberry5-carolus-ros2/carolus_ros2/config/logitech_1080p.yaml"
    "raspberry5-carolus-ros2/carolus_ros2/package.xml"
    # 2026-08-20 (2) : le coeur copie dans raspberry5-carolus-ros2/ est un
    # SNAPSHOT independant de carolus_ws/src/libuvgs_astrobee/ -- il peut
    # diverger et doit etre sauvegarde separement, pas suppose synchronise.
    "raspberry5-carolus-ros2/carolus_ros2/src/ceresP4P.cpp"
    "raspberry5-carolus-ros2/carolus_ros2/src/pose_est.cpp"
    "raspberry5-carolus-ros2/carolus_ros2/src/beacon_detector.cpp"
    "raspberry5-carolus-ros2/carolus_ros2/src/pose_filter.cpp"
    "raspberry5-carolus-ros2/carolus_ros2/include/carolus_node/beacon_detector.hpp"
    "raspberry5-carolus-ros2/carolus_ros2/include/carolus_node/pose_filter.hpp"
    "raspberry5-carolus-ros2/technical-ros2.tex"
    "raspberry5-carolus-ros2/README.md"
    "shortcuts/ros2_sync_check.sh"
    # 2026-08-19 (2) : filtre FIFO extrait du noeud ROS1 vers carolus_core.
    "carolus_ws/src/libuvgs_astrobee/src/pose_filter.cpp"
    "carolus_ws/src/libuvgs_astrobee/include/carolus_node/pose_filter.hpp"
    # 2026-08-31 : ajoute AVANT de le modifier, pas apres -- il devient
    # l'instrument principal du protocole 25 (rejouer les trois conditions de
    # BUG-093 avec le canal optique, apres que la telemetrie sub_attitude ait
    # ete disqualifiee). Quatrieme occurrence du meme pattern que les commentaires
    # ci-dessus decrivent : un fichier qui devient central doit entrer dans cette
    # liste le jour meme, sans exception.
    "shortcuts/optical_drift_observer.py"
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
