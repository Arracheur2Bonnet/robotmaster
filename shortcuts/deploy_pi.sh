#!/usr/bin/env bash
# deploy_pi.sh — deploie les fichiers cote Pi et verifie l'integrite par checksum.
#
# Seul rm_cam_beacon.py tourne SUR le Pi : carolus_launcher.py et cam_view_helper.py
# tournent sur le PC labo (pris en compte au prochain lancement du launcher, pas de SCP).
# carolus_tf_broadcaster.py (T4) et map_collision.py ne sont pas modifies cette session.
#
# Usage : bash shortcuts/deploy_pi.sh
# Prerequis : Pi joignable (RNDIS up), cle SSH ~/.ssh/carolus_nopass.

set -u

PI="ubuntu@192.168.0.103"
KEY="$HOME/.ssh/carolus_nopass"
SSH_OPTS=(-i "$KEY" -o IdentitiesOnly=yes -o StrictHostKeyChecking=no -o ConnectTimeout=5)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL="$ROOT/carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py"
REMOTE="/home/ubuntu/carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py"

echo "== deploy_pi.sh =="

# 0. Verif syntaxe locale avant d'envoyer quoi que ce soit
if ! python3 -m py_compile "$LOCAL"; then
    echo "ABORT: rm_cam_beacon.py ne compile pas localement." >&2
    exit 1
fi
echo "  py_compile local OK"

# 1. Joignabilite Pi
if ! SSH_AUTH_SOCK='' ssh "${SSH_OPTS[@]}" "$PI" "echo ok" >/dev/null 2>&1; then
    echo "ABORT: Pi injoignable ($PI). Verifier RNDIS / alimentation." >&2
    exit 1
fi
echo "  Pi joignable"

# 2. SCP
if ! SSH_AUTH_SOCK='' scp "${SSH_OPTS[@]}" "$LOCAL" "$PI:$REMOTE" >/dev/null 2>&1; then
    echo "ABORT: scp echoue." >&2
    exit 1
fi
echo "  scp rm_cam_beacon.py -> Pi OK"

# 3. Verif checksum local vs distant
LOC_MD5="$(md5sum "$LOCAL" | awk '{print $1}')"
REM_MD5="$(SSH_AUTH_SOCK='' ssh "${SSH_OPTS[@]}" "$PI" "md5sum $REMOTE" 2>/dev/null | awk '{print $1}')"
echo "  md5 local  : $LOC_MD5"
echo "  md5 distant: $REM_MD5"
if [ "$LOC_MD5" != "$REM_MD5" ]; then
    echo "ABORT: checksum different apres scp !" >&2
    exit 1
fi
echo "  checksum identique -> deploiement verifie"

# 4. Verif syntaxe cote Pi (interpreteur du Pi)
if SSH_AUTH_SOCK='' ssh "${SSH_OPTS[@]}" "$PI" "python3 -c 'import ast; ast.parse(open(\"$REMOTE\").read())'" 2>/dev/null; then
    echo "  py ast.parse cote Pi OK"
else
    echo "  WARN: ast.parse cote Pi a echoue (a verifier manuellement)." >&2
fi

echo
echo "OK. Rappel : cote launcher (PC labo), relancer T2 (Kill T2 puis > 2 Camera+Beacon)"
echo "pour charger le nouveau rm_cam_beacon.py + le nouveau cam_view_helper.py."
