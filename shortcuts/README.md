# Shortcuts — Carolus / RoboMaster

Scripts de raccourci pour les opérations fréquentes. Prérequis communs : robot allumé (double carillon), Pi sur `192.168.0.103`.

---

## `carolus_launcher.py`

**Quoi :** GUI tkinter (thème sombre) — séquence T1/T2/T3/T4, dashboard live, live map intégrée, pilotage manuel châssis (ZQSD) et nacelle (numpad), blocs de touches interactifs, mode LOCATE (localisation balise sans avance), bouton LOCK (centrage périodique de la balise, période configurable en secondes, 2026-07-23), voyant + minimap balise (2026-07-23, cf. section dédiée), bouton RECENTRER CAM (position de base nacelle, 2026-07-23), bouton APERCU CAM (OFF par défaut — active/désactive l'abonnement caméra, gagne en fluidité + bande passante réseau), plein écran (**F11** bascule, **Échap** quitte, 2026-07-23).

**Pourquoi :** lance la pile sans taper de commandes ; dashboard live (état SEARCH/APPROACH/STOP/LOCATE/MANUEL, profondeur, batterie robot, caméra) ; live map temps réel (position robot + balise sur fond JSON) ; pilotage immédiat sans quitter la fenêtre ; retour visuel des touches actives.

**Usage :**
```bash
python3 shortcuts/carolus_launcher.py
```

| Bouton | Ce qui tourne | Déverrouillé quand |
|---|---|---|
| 1 · roscore + Pi | gnome-terminal → SSH → `eth1 up` + `roscore` | port 11311 ouvert (timeout 60s) |
| 2 · Caméra + Beacon | SSH intégré → `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` publié (timeout 60s) |
| 3 · Carolus Astrobee | gnome-terminal → `roslaunch carolus_node testcarolus.launch` | — (manuel) |
| 4 · TF Broadcaster (quat fix) | SSH intégré → `carolus_tf_broadcaster.py` sur le Pi | — (manuel, aucune attente — nœud léger, démarrage quasi instantané) |

**T4 — ajouté le 2026-07-20, suite au fix BUG-048** (remapping quaternion Carolus→ROS, permutation naïve remplacée par composition `q_ros=r⊗q`). Republie `/pose` de Carolus en TF (`camera_link`→`beacon_observed`) via `carolus_tf_broadcaster.py`, exécuté sur le Pi. Sans effet sur le pipeline SEARCH/ALIGN/APPROACH actuel (qui consomme `/pose` directement, pas la TF) — pertinent pour valider l'orientation (`rosrun tf tf_echo camera_link beacon_observed`) et prépare le terrain pour l'adoption tf2_ROS/EKF de la Phase F. Peut être lancé indépendamment de T3, mais n'aura rien à republier tant que T3 (source de `/pose`) ne tourne pas.

**Logs — refonte du 2026-07-20 : un onglet par terminal.** Les 4 terminaux (T1-T4) sont désormais **tous intégrés** (plus de fenêtre gnome-terminal externe pour T1/T3 — leur sortie est capturée et affichée dans l'appli, comme T2/T4 l'étaient déjà). La zone Logs est un `ttk.Notebook` à 4 onglets (`T1 roscore+Pi`, `T2 Camera+Beacon`, `T3 Carolus Astrobee`, `T4 TF Broadcaster`), chaque terminal écrit uniquement dans son propre onglet — plus de mélange dans une boîte unique. Les messages d'événement globaux (mode AUTO/MANUEL, kill, etc.) restent diffusés dans les 4 onglets à la fois. Le bouton "Copier les logs" copie désormais le contenu de l'onglet actif uniquement. Changement nécessaire pour lancer T1 en mode intégré : vérifié au préalable que `sudo` sur le Pi ne demande pas de mot de passe (`sudo -n true`), sinon la commande `sudo ip link set eth1 up` de T1 bloquerait silencieusement en pipe.

**Kill** : annule les attentes en cours (`wait_for_roscore` / `wait_for_camera`) puis tue les processus SSH et locaux. Libère les zombies OS (`proc.wait()`). Un Kill partiel (bouton Kill d'une ligne) ne tue que cette cible et les processus en aval.

---

### Live map (panneau droit)

Panneau `_LiveMapCanvas` affiché à droite de tous les contrôles. Canvas 520×420 px (26 cases × 20 px), fond grille, blocs obstacles du JSON.

| Élément | Description |
|---|---|
| Carré bleu (■▲) | Position + cap robot en temps réel (mis à jour via `[POS]` + `[ATTI]`) |
| Point jaune | Position balise détectée la plus récente (mis à jour via `[BEACONPOS]`). **Disparaît automatiquement** si aucune détection depuis 1.5s (`BEACON_FRESH_S`) — sert d'indicateur visuel que la détection est active sans avoir à lire les logs T3. |
| Blocs gris | Obstacles chargés depuis `mapv1.json` au démarrage |
| Bouton **Charger map** | Ouvrir un autre fichier JSON depuis le disque |

**Auto-chargement :** `mapv1.json` (racine projet) chargé automatiquement 500ms après le lancement du launcher.

**Convention axes (même que map_editor) :**
- x EP (avant/nord) → haut sur canvas
- y EP (droite/est) → droite sur canvas

---

### Mode LOCALISER (LOCATE)

Bouton **LOCALISER** dans la rangée de contrôle (jaune-or quand actif). Publie `"LOCATE"` sur `/carolus/mode`.

**Comportement en mode LOCATE :**
- Le sweep gimbal continue (identique à AUTO/SEARCH).
- Dès que la balise est visible : robot s'immobilise (`stop_gimbal` + `stop_chassis`), position publiée sur la live map.
- Pas de transition ALIGN/APPROACH — le robot reste en place.
- Si la balise disparaît : le sweep reprend.

**Auto-activation :** LOCATE s'active automatiquement 500ms après que T2 confirme que la caméra est prête. Pas besoin d'appuyer manuellement sur LOCALISER au démarrage.

Pour repasser en AUTO (suivi complet) : cliquer **MODE : AUTO**.

---

### LOCK (centrage périodique de la balise, 2026-07-23)

Bouton **LOCK** dans la rangée de contrôle, avec un champ de saisie à côté (période en secondes, défaut **1**). Publie `"LOCK ON"`/`"LOCK OFF"` sur `/carolus/gimbal_lock`, et la période sur `/carolus/gimbal_lock_period` (via `cam_view_helper.py`).

**Fonctionnement :** toutes les *N* secondes (N = valeur du champ), s'il y a une pose fraîche, une **seule** commande de mouvement relatif (`gimbal.move()`) recentre la balise dans le champ, indépendamment du mouvement du châssis.

- **Période configurable en direct** : taper une valeur (ex. 5, 10) dans le champ et appuyer sur Entrée. **Secondes uniquement.** Une valeur non comprise (texte, négatif, vide) est ignorée sans planter — repli silencieux sur 1s côté `rm_cam_beacon.py`, comme un champ de formulaire web classique. Testé en changeant la valeur sur un node déjà lancé, sans relancer la stack.
- **Yaw seulement** : le pitch reste désactivé (`GIM_PITCH_TRACK_ENABLED=False`, depuis l'incident BUG-058 — nacelle en butée → câble accroché).
- Actif uniquement en mode MANUEL, reset à OFF à l'entrée/sortie MANUEL et au Kill.
- **Pilotage numpad nacelle IGNORÉ tant que LOCK est ON** (2026-07-23 nuit) : quand le LOCK est actif, il a la main exclusive sur la nacelle ; les touches numpad 8/4/5/6 sont sans effet. Le châssis (ZQSD) reste pilotable normalement. Hors LOCK, le numpad reprend la main.
- Ignore un tick si l'erreur d'angle dépasse `GIM_LOCK_MAX_ERR_DEG` (45°) — probable pose aberrante.
- **Deadband `GIM_LOCK_DEADBAND_DEG=5°`** : sous ce seuil, pas de re-correction (pas la peine de recentrer trop finement).
- **Vitesse `GIM_LOCK_YAW_SPEED=540°/s`** (plafond SDK, demande explicite utilisateur) — jamais testée sur ce robot au-delà de 80°/s avant ce choix, **confirmée fonctionnelle sur matériel** le 2026-07-23 (soir).
- **Historique :** un précédent LOCK BALISE (servo continu à 20Hz avec gating/rampe/rejet d'aberration, distinct du mécanisme ci-dessus, parfois appelé « v1 ») a existé du 2026-07-22 au 2026-07-23 puis a été **retiré intégralement** le 2026-07-23 (soir), jugé redondant. Ce bouton s'appelait alors « LOCK V2 » ; il a été renommé simplement « LOCK » une fois le v1 supprimé.
- **Confirmé fonctionnel sur matériel le 2026-07-23 (soir)** par l'utilisateur ("tout marche"), y compris à 540°/s.

---

### Voyant + minimap balise (2026-07-23)

Sous le panneau caméra du dashboard.

- **Voyant** (rond + texte, anglais) : `BEACON: DETECTED` (vert) / `BEACON: LOST` (rouge). Alimenté par le log `[BEACON] status=...` que `rm_cam_beacon.py` publie à 5Hz.
- **MINIMAP BALISE** (petit canvas 100×100) : un point représente la position de la balise *dans le champ caméra* (vert si centrée à ±3°, orange sinon) — distinct de la live map robot/grille existante, qui montre la position dans le labo.
- Reset à l'entrée/sortie MANUEL et au Kill (même hygiène que le bouton LOCK).
- **Boutons REMEMBER BEACON/SEARCH BEACON retirés le 2026-07-23 (nuit)** : avaient été confirmés fonctionnels sur matériel plus tôt dans la soirée, puis jugés insatisfaisants par l'utilisateur sans détail précis — retrait complet demandé, aucune trace de code restante. Remplacés par le bouton **RECENTRER CAM** (voir section dédiée ci-dessous).

---

### RECENTRER CAM (position de base nacelle, 2026-07-23)

Bouton **RECENTRER CAM**, sous le panneau caméra. Publie `"RECENTER"` sur `/carolus/gimbal_recenter`.

- Ramène la nacelle à sa position de base (pitch=0, yaw=0, repère power-on du gimbal) via `gimbal.recenter()` du SDK — orientation de la **caméra**, indépendante de l'orientation du châssis robot.
- Vitesse de recentrage : 360°/s sur les deux axes (plafond SDK pour `recenter()`, différent de `move()` qui plafonne à 540°/s).
- Actif uniquement en mode MANUEL (même scope que LOCK).
- **Bug corrigé le 2026-07-23 (nuit)** : ne fonctionnait pas car la boucle MANUEL réémettait `drive_speed(0,0)` à 20Hz, annulant l'action de recentrage ~50ms après son lancement (un grand angle de recentrage prend ~0.7s). Fix : fenêtre « gimbal occupé » de 2.5s pendant laquelle la boucle MANUEL et le LOCK suspendent leurs commandes. **Corrigé au niveau code, pas encore déployé/testé** (Pi injoignable au moment du fix).

---

### Blocs de pilotage visuel (MODE MANUEL uniquement)

Deux blocs apparaissent sous les boutons de lancement. Les touches s'allument en or quand elles sont actives (clavier ou clic souris).

**CHASSIS (ZQSD)**

```
      [Z]
  [Q] [S] [D]
```
- `Z` = avant · `S` = arrière · `Q` = rotation gauche · `D` = rotation droite
- vx = 0.20 m/s · wz = 20 deg/s
- Stop auto à la release de toutes les touches

**NACELLE (NUM 8/4/5/6/2)**

```
      [8]
  [4] [5] [6]
      [2]
```
- `8` = pitch haut · `2` = pitch bas · `4` = yaw gauche · `6` = yaw droite · `5` = stop gimbal
- pitch = 30 deg/s · yaw = 40 deg/s
- Fonctionne avec NumLock ON (`KP_8`…) et NumLock OFF (`KP_Up`…)

**Activation :** cliquer `MODE : AUTO` → passe en `MODE : MANUEL` (orange). ZQSD/numpad actifs depuis le launcher **et** depuis la fenêtre map editor (bindings propagés aux deux fenêtres). Guard : les touches ne déclenchent pas de commande si le focus est sur un widget de saisie texte. Retour AUTO : re-cliquer, chassis + gimbal s'arrêtent immédiatement.

**Focus clavier :** T1 et T3 ouvrent des fenêtres `gnome-terminal` externes qui volent le focus clavier du système. Si ZQSD ne répond plus après avoir lancé T1/T3, il suffit de **survoler la fenêtre du launcher ou de l'éditeur de map avec la souris** — le focus est repris automatiquement (`<Enter>` → `focus_set()`, corrigé 2026-07-01).

---

### Dashboard

| Indicateur | Détail |
|---|---|
| Point état robot | gris=SEARCH · orange=APPROACH · vert=STOP · jaune-or=LOCATE · bleu=MANUEL |
| `depth = X.XXm` | distance balise en mode APPROACH |
| Batterie robot | barre verte>40% · orange 15-40% · rouge <15% · `N/A` si non exposée |
| Caméra 320×180 | vignette PNG mise à jour ~4 Hz via `cam_view_helper.py` |
| Connexion Pi | ping toutes les 5s → point vert/rouge + IP |

**Logs :** zone sélectionnable, `Ctrl+A` pour tout sélectionner, `Ctrl+C` pour copier, bouton **"Copier les logs"**. Pas de freeze grâce à la queue asynchrone (batch 50ms / 50 lignes max, débit 1000 lignes/s). Télémétrie haute fréquence (`[ESC]`, `[ATTI]`, `[POS]`, `[BAT]`, `[VEL]`, `[TOF]`) filtrée de la zone Logs — affichée uniquement dans le dashboard. `[BEACONPOS]` reste visible dans les logs (utile pour diagnostiquer la position balise).

---

## `save_session.sh`

**Quoi :** snapshot des fichiers sources actifs dans `saves/YYYY-MM-DD-HH-MM/`.

**Pourquoi :** permet de revenir à un état stable si une modification casse quelque chose.

**Usage :**
```bash
bash shortcuts/save_session.sh "avant test gimbal"
# Restaurer un fichier :
cp saves/2026-06-24-20-10/carolus_ws__src__robomaster_cam__scripts__rm_cam_beacon.py \
   carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py
```

**Fichiers sauvegardés :** `carolus_launcher.py`, `cam_view_helper.py`, `map_editor.py`, `rm_cam_beacon.py`, `testcarolus.launch`, plus les 5 `CMakeLists.txt` du workspace (`src/`, `libuvgs_astrobee/`, `ff_msgs/`, `robomaster_cam/`, `carolus_node/`) — ajoutés le 2026-07-13 pour couvrir la règle CLAUDE.md qui les cite comme fichiers critiques.

**Attendu :** dossier `saves/YYYY-MM-DD-HH-MM/` créé avec 10 fichiers + `NOTE.txt`.

---

## `deploy_pi.sh`

**Quoi :** déploie `rm_cam_beacon.py` sur le Pi (SCP) et vérifie l'intégrité par checksum md5 local vs distant + `ast.parse` côté Pi.

**Pourquoi :** rendre le déploiement fiable en une commande (au lieu d'un SCP manuel + vérif à l'œil), avec garde-fous : refuse d'envoyer si le fichier ne compile pas localement, si le Pi est injoignable, ou si le checksum diffère après copie.

**Usage :**
```bash
bash shortcuts/deploy_pi.sh
```

| Étape | Contrôle |
|---|---|
| 0 | `py_compile` local — abort si erreur syntaxe |
| 1 | Pi joignable (SSH ConnectTimeout 5s) — abort sinon |
| 2 | `scp rm_cam_beacon.py` → `/home/ubuntu/carolus_ws/.../rm_cam_beacon.py` |
| 3 | md5 local == md5 distant — abort si différent |
| 4 | `ast.parse` côté Pi (warn seulement) |

**Note :** seul `rm_cam_beacon.py` tourne sur le Pi. `carolus_launcher.py` et `cam_view_helper.py` tournent sur le PC labo → pris en compte au prochain lancement du launcher (pas de SCP). Après déploiement : relancer T2 (Kill T2 → `> 2 Camera+Beacon`) pour charger le nouveau code.

**Attendu :** `checksum identique -> deploiement verifie`, puis rappel de relancer T2.

---

## `leak_scan.sh`

**Quoi :** scan par motifs (mots de passe/clés API/tokens/en-têtes de clé privée en dur) sur `carolus_ws/`, `shortcuts/`, `github/`, `research-log/`.

**Pourquoi :** filet de sécurité avant tout envoi externe (premier `git push`, upload Overleaf) — créé le 2026-07-24 dans le cadre de l'audit fuite (`research-log/15-audit-fuites.md`), qui a trouvé un vrai mot de passe en clair dans `journal.md` (depuis corrigé). Détection par mots-clés seulement (pas d'outil dédié type gitleaks/trufflehog installé sur cette machine) — ne remplace pas une relecture manuelle des sorties terminales collées dans le journal.

**Usage :**
```bash
bash shortcuts/leak_scan.sh                      # scan les 4 dossiers par défaut
bash shortcuts/leak_scan.sh chemin/specifique     # scan un dossier précis
```

**Attendu :** `Rien trouve sur les motifs connus.` si clean, sinon liste des lignes suspectes à vérifier manuellement (faux positifs possibles).

---

## `cam_view_helper.py`

**Quoi :** process séparé à trois rôles — (1) vignette caméra PNG ~4 Hz avec HUD incrusté (réticule au centre géométrique de l'image + anneau de tolérance + marqueur balise reprojeté via les vraies intrinsèques, 2026-07-23), (2) passerelle clavier GUI → topics ROS châssis, (3) passerelle numpad GUI → topic ROS nacelle.

**Pourquoi :** process isolé car `rospy.init_node` + SIGINT entrent en conflit avec tkinter. Centralise toutes les publications ROS depuis le GUI sans 2e connexion SDK. Lancé/arrêté automatiquement par `carolus_launcher.py`.

**Commandes stdin (envoyées par le launcher via PIPE) :**

| Commande | Effet |
|---|---|
| `MODE AUTO` | Publie `"AUTO"` sur `/carolus/mode` (latch) |
| `MODE MANUAL` | Publie `"MANUAL"` sur `/carolus/mode` (latch) |
| `MODE LOCATE` | Publie `"LOCATE"` sur `/carolus/mode` (latch) — sweep sans avance |
| `VX 0.20 WZ 20.0` | Publie `Twist(linear.x, angular.z)` sur `/carolus/cmd_vel` |
| `STOP` | Publie `Twist()` zéros sur `/carolus/cmd_vel` |
| `GIMBAL 30.0 0.0` | Publie `Twist(angular.y=pitch, angular.z=yaw)` sur `/carolus/gimbal_vel` |
| `LOCK ON` / `LOCK OFF` | Publie `"ON"`/`"OFF"` sur `/carolus/gimbal_lock` (centrage périodique) |
| `LOCKPERIOD 5.0` | Publie `"5.0"` sur `/carolus/gimbal_lock_period` (période en secondes, repli sur défaut si invalide) |
| `RECENTER` | Publie `"RECENTER"` sur `/carolus/gimbal_recenter` (position de base nacelle, 2026-07-23) |

**Usage :** (auto, via le launcher) — manuel pour debug :
```bash
source /opt/ros/noetic/setup.bash && source carolus_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.103:11311 ROS_IP=192.168.0.100
python3 shortcuts/cam_view_helper.py /tmp/carolus_cam.png
```

**Attendu :** `/tmp/carolus_cam.png` se met à jour ~4×/s. En mode MANUEL, `rm_cam_beacon.py` répond aux commandes VX/WZ/GIMBAL dans les ~50ms.

---

## `map_editor.py`

**Quoi :** éditeur de map 2D (fenêtre Toplevel séparée) — grille 26×21 cases (10.4m×8.4m, 1 case = 40 cm ≈ footprint S1), blocs plein/demi/quart, outil zone (drag fill), balises orientées demi-bloc, overlay robot snapé sur grille.

**Pourquoi :** permet de cartographier les obstacles réels du labo (chaises, bureaux) avant une session, de positionner la balise et de visualiser la position du robot en live. La map JSON exportée est chargée par `map_collision.py` sur le Pi pour l'évitement d'obstacles.

**Usage :** ouvert depuis le bouton "ÉDITEUR MAP" dans `carolus_launcher.py` (pas de lancement direct).

| Outil | Clic gauche | Clic droit |
|---|---|---|
| ▓ Plein | Poser bloc plein | Pas d'effet |
| ▬ Demi | Poser demi-bloc (rotation auto selon position) | Changer rotation |
| ▪ Quart | Poser quart-bloc | Changer rotation |
| ▦ Zone | Drag → remplir rectangle de blocs pleins | — |
| ◉ Balise | Placer balise (mode MANUEL) | Tourner balise 90° |
| ✕ Effacer | Supprimer bloc ou balise | — |

**Robot :** verrouillé par défaut (🔒). Déverrouiller via la palette → drag → snap automatique au centre de case. La position définit l'origine (0,0) pour toutes les mises à jour live SDK.

**Balises :**
- **Mode MANUEL** (défaut) : drag pour placer, clic droit pour tourner. 1 seule balise orange.
- **Mode AUTO** : `add_auto_beacon(wx_m, wy_m, facing_deg)` appelé par le launcher à chaque `[BEACONPOS]`. Multi-balises, déduplication < 0.5m, couleur jaune-or.

**Save/Load :** JSON v3 — `blocks`, `beacon_man` (wx, wy, rot), `beacons_auto` (liste wx/wy/facing). Copier la map sauvegardée vers `/home/ubuntu/carolus_map.json` sur le Pi pour activer la collision avoidance.

**Navigation dans la map :**
- **Molette souris** → zoom centré sur le curseur (facteur 1.15 par cran, Linux Button-4/5 supporté)
- **Clic droit + drag** → pan (déplacement de la vue), tous les items se déplacent ensemble
- **Clic droit sans drag** → changer la rotation du bloc sous le curseur (comportement inchangé)
- Zoom/pan n'affecte pas les coords stockées (world, mètres) — save/load reste correct à tout niveau de zoom

**Convention axes (EP SDK → map) :**
- EP `x+` = avant → nord sur canvas (py diminue)
- EP `y+` = droite (est) → px augmente — convention inverse de ROS REP-103
- Robot positionné au centre géométrique de la grille par défaut (colonne 13, ligne ~10.5 — 21 lignes est impair, le centre exact n'est pas une case entière — sur une grille 26×21, depuis l'agrandissement du 2026-06-30 — anciennement en bas de la grille 20×15).

**Attendu :** overlay robot bleu (■▲) se déplace en live via `update_robot()`, point orange temporaire via `update_beacon()`, balises persistantes via `add_auto_beacon()`. Zoom avec molette, pan avec clic droit drag. Hover ghost optimisé (cache par case/rotation — aucun redraw si la souris ne change pas de case).
