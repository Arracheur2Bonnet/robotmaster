#!/usr/bin/env bash
# leak_scan.sh — scan par motifs pour identifiants/secrets avant tout envoi externe
# (GitHub, Overleaf). Complementaire d'un outil dedie (gitleaks/trufflehog) si
# disponible sur la machine -- ce script est le filet de secours grep-based.
#
# Usage : bash shortcuts/leak_scan.sh [dossier...]
# Sans argument : scanne carolus_ws/, shortcuts/, github/, research-log/ (les
# dossiers destines a un envoi externe ou cites dans le rapport Hector).

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TARGETS=("$@")
if [ ${#TARGETS[@]} -eq 0 ]; then
    TARGETS=(carolus_ws shortcuts github research-log)
fi

# Motifs : identifiants/mots de passe en dur, cles API generiques, en-tetes de
# cle privee, tokens connus (GitHub/AWS/OpenAI-style), chaines user:pass@host.
PATTERN='password[[:space:]]*=|passwd[[:space:]]*=|api[_-]?key|secret[[:space:]]*=|token[[:space:]]*=|ssh-rsa AAAA|BEGIN (RSA|OPENSSH|EC|DSA) PRIVATE KEY|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|AKIA[0-9A-Z]{16}|sk-[A-Za-z0-9]{20,}|[a-zA-Z0-9_.+-]+:[^[:space:]@/]{6,}@[a-zA-Z0-9.-]+'

echo "== leak_scan.sh =="
echo "Cibles : ${TARGETS[*]}"
echo

FOUND=0
for dir in "${TARGETS[@]}"; do
    [ -d "$dir" ] || continue
    HITS="$(grep -rInE "$PATTERN" "$dir" \
        --exclude-dir=__pycache__ --exclude-dir=build --exclude-dir=devel \
        --exclude="*.pyc" 2>/dev/null)"
    if [ -n "$HITS" ]; then
        echo "--- $dir ---"
        echo "$HITS"
        echo
        FOUND=1
    fi
done

if [ "$FOUND" -eq 0 ]; then
    echo "Rien trouve sur les motifs connus."
else
    echo "ATTENTION : verifier chaque ligne ci-dessus manuellement (faux positifs possibles"
    echo "-- ex. le mot 'token' dans un commentaire, une IP privee dans un user:pass@host mal detecte)."
fi

echo
echo "Rappel : ce script ne remplace pas un outil dedie (gitleaks/trufflehog, non installes"
echo "sur cette machine au 2026-07-24) -- detection par mots-cles seulement, pas par entropie."
