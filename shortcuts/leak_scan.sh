#!/usr/bin/env bash
# leak_scan.sh — scan par motifs pour identifiants/secrets avant tout envoi externe
# (GitHub, Overleaf, transfert a un collegue). Complementaire d'un outil dedie
# (gitleaks/trufflehog) si disponible -- ce script est le filet grep-based.
#
# Usage : bash shortcuts/leak_scan.sh [dossier...]
# Sans argument : scanne TOUT ce qui peut partir a l'exterieur.
#
# ============================================================================
# 2026-08-03 — DEUX FAILLES CORRIGEES, decouvertes en lisant la doc heritee
# ============================================================================
# Un Personal Access Token GitHub ACTIF (prefixe github_pat_11BXSQ7DQ0..., compte
# d'un tiers) dort en clair dans
#   old/Carolus Node Installation, OpenCV Compatibility, and Operation on LIMO.docx
# depuis des mois, dans une procedure de clone git recopiee telle quelle.
#
# Ce script ne l'a jamais vu, pour DEUX raisons independantes :
#
#   1. PERIMETRE. old/ et documentations/ n'etaient pas dans les cibles par
#      defaut. L'audit du 2026-07-24 s'est declare "clean" sans jamais les
#      regarder. -> corrige ci-dessous.
#
#   2. FORMAT. Le motif github_pat_ EXISTAIT deja et etait correct : il n'a
#      simplement jamais eu l'occasion de matcher, parce qu'un .docx est un
#      conteneur ZIP et qu'un grep texte est structurellement aveugle dedans.
#      Idem .odt et .pdf. C'est-a-dire aveugle a EXACTEMENT les 38 documents
#      du registre research-log/23-inventaire-doc-sources.md.
#      -> corrige : extraction du texte avant grep.
#
# La lecon generale, qui vaut au-dela de ce script : un scanner qui ne peut pas
# lire un format donne ne rapporte pas "je ne sais pas", il rapporte "rien
# trouve". C'est le meme mode de defaillance que BUG-087 (solveur non converge
# publie comme valide) -- une absence de signal prise pour un signal negatif.
# ============================================================================

set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TARGETS=("$@")
if [ ${#TARGETS[@]} -eq 0 ]; then
    # old/ et documentations/ ajoutes 2026-08-03. Ils contiennent la doc heritee
    # que l'on s'apprete a absorber, archiver et potentiellement publier.
    TARGETS=(carolus_ws shortcuts github research-log documentations old)
fi

# Motifs : identifiants/mots de passe en dur, cles API generiques, en-tetes de
# cle privee, tokens connus (GitHub/AWS/OpenAI-style), chaines user:pass@host.
PATTERN='password[[:space:]]*=|passwd[[:space:]]*=|api[_-]?key|secret[[:space:]]*=|token[[:space:]]*=|ssh-rsa AAAA|BEGIN (RSA|OPENSSH|EC|DSA) PRIVATE KEY|ghp_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,}|AKIA[0-9A-Z]{16}|sk-[A-Za-z0-9]{20,}|[a-zA-Z0-9_.+-]+:[^[:space:]@/]{6,}@[a-zA-Z0-9.-]+'

# Arbres de code tiers : bruit garanti (chaines "token" dans des parsers, mots de
# passe d'exemple dans des tests unitaires). Exclus du scan TEXTE seulement --
# le scan DOCUMENTS ci-dessous ne les traverse pas de toute facon.
EXCL_DIR=(--exclude-dir=__pycache__ --exclude-dir=build --exclude-dir=devel
          --exclude-dir=.git --exclude-dir=node_modules --exclude-dir=site-packages
          --exclude-dir=librealsense --exclude-dir=third-party --exclude-dir=unit-tests)

echo "== leak_scan.sh =="
echo "Cibles : ${TARGETS[*]}"
echo

FOUND=0

# ---------------------------------------------------------------- scan TEXTE
for dir in "${TARGETS[@]}"; do
    [ -d "$dir" ] || continue
    HITS="$(grep -rInE "$PATTERN" "$dir" "${EXCL_DIR[@]}" --exclude="*.pyc" \
            --binary-files=without-match 2>/dev/null)"
    if [ -n "$HITS" ]; then
        echo "--- $dir (texte) ---"
        echo "$HITS"
        echo
        FOUND=1
    fi
done

# ----------------------------------------------------------- scan DOCUMENTS
# .pdf via pdftotext ; .docx/.odt/.pptx via leur conteneur ZIP (XML interne).
# Chaque document est converti en texte dans un fichier temporaire, grepe, puis
# le temporaire est supprime immediatement -- on ne laisse pas trainer une copie
# en clair d'un document qui contient justement un secret.
echo "--- documents (.pdf / .docx / .odt) ---"
DOC_HITS=0
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

while IFS= read -r -d '' f; do
    case "${f,,}" in
        *.pdf)
            command -v pdftotext >/dev/null || continue
            pdftotext -q "$f" "$TMP" 2>/dev/null || continue
            ;;
        *.docx|*.odt|*.pptx)
            # Pas d'odt2txt sur cette machine (verifie 2026-08-03) : on lit le
            # XML interne du conteneur, ce qui suffit largement pour un grep.
            unzip -p "$f" '*.xml' > "$TMP" 2>/dev/null || continue
            ;;
        *) continue ;;
    esac
    H="$(grep -aoIE "$PATTERN" "$TMP" 2>/dev/null | sort -u | head -5)"
    if [ -n "$H" ]; then
        echo "  [$f]"
        # On affiche le motif trouve TRONQUE : le but est de signaler, pas de
        # recopier un secret dans un log de terminal qui finira colle quelque part.
        echo "$H" | cut -c1-40 | sed 's/^/     /'
        DOC_HITS=1
        FOUND=1
    fi
done < <(find "${TARGETS[@]}" -type f \
            \( -iname '*.pdf' -o -iname '*.docx' -o -iname '*.odt' -o -iname '*.pptx' \) \
            -print0 2>/dev/null)

[ "$DOC_HITS" -eq 0 ] && echo "  (rien sur les motifs connus)"
echo

if [ "$FOUND" -eq 0 ]; then
    echo "Rien trouve sur les motifs connus."
else
    echo "ATTENTION : verifier chaque ligne manuellement (faux positifs possibles --"
    echo "ex. le mot 'token' dans un commentaire, une IP privee mal detectee en user:pass@host)."
    echo "Les correspondances documents sont TRONQUEES a 40 caracteres exprès."
fi

echo
echo "Rappel : detection par mots-cles, pas par entropie -- ne remplace pas"
echo "gitleaks/trufflehog (non installes sur cette machine au 2026-07-24)."
echo "Un format non lisible par ce script rapporterait 'rien trouve', pas 'je ne sais"
echo "pas' : verifier le periMETRE autant que le resultat."
