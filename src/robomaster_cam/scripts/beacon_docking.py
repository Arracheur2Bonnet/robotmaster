#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Docking balise — position finale FIXE et repetable par rapport a la balise.

Objectif (demande Hector, cf. research-log/07-perplexity/17-docking-position-
fixe-balise.md) : quel que soit l'angle d'arrivee, le robot doit finir TOUJOURS
au meme endroit par rapport a la balise — a DOCK_DISTANCE_M devant elle, sur
l'axe frontal de la balise, face a elle. Analogie utilisateur : une voiture qui
rentre au garage, alignee, pas de biais.

Le pipeline existant (SEARCH -> ALIGN -> APPROACH -> STOP dans
`rm_cam_beacon.py`) s'arrete a "pointe la balise et avance jusqu'a 0.70 m" : il
n'utilise QUE la position (x,y,z) de la balise, jamais son ORIENTATION. Il
converge donc vers un cercle de rayon 0.70 m autour de la balise, pas vers un
point. Ce module ajoute l'etage manquant : l'angle hors-axe.


=====================================================================
ARCHITECTURE — pourquoi un nœud separe et pas un etat de plus
=====================================================================
mini-ADR. *Contexte* : le docking a besoin de piloter le chassis, or
`rm_cam_beacon.py` detient la connexion SDK UNIQUE au robot (tout son entete
documente que deux connexions simultanees cassent `drive_speed`). *Options* :
(a) ajouter un etat DOCK dans `rm_cam_beacon.py` — acces direct au SDK, donc
au deplacement lateral (chassis holonome Mecanum), mais modifie un fichier
teste et valide sur materiel ; (b) nœud separe qui commande via les topics ROS
deja exposes (`/carolus/cmd_vel`, `/carolus/gimbal_vel`), sans toucher a
l'existant. *Choix* : (b) — aucune regression possible sur la chaine validee
(F3, ALIGN/APPROACH testes materiel), le docking peut etre lance, teste et
abandonne sans redeployer `rm_cam_beacon.py`.
*Consequence acceptee* : `/carolus/cmd_vel` n'est cable en mode MANUEL que sur
vx et wz (`drive_speed(x=vx, y=0.0, z=wz)`) — la translation laterale du
chassis holonome n'est PAS accessible par ce canal. Le docking est donc traite
comme un probleme NON-HOLONOME (tourner / avancer / tourner), pas comme un
recalage lateral direct. Condition qui reviserait ce choix : si un jour
`rm_cam_beacon.py` relaie `msg.linear.y` vers `drive_speed(y=...)`, la manœuvre
en 3 segments pourrait etre remplacee par un simple deport lateral.


=====================================================================
STRATEGIE DE COMMANDE — "look-and-move" iteratif, pas de servo continu
=====================================================================
mini-ADR. *Contexte* : Carolus publie `/pose` a ~2.5 Hz (goulot = transport
reseau des images, documente dans `rm_cam_beacon.py`). *Options* : (a) loi de
commande continue en coordonnees polaires (controle de parking classique
rho/alpha/beta pour robot unicycle) ; (b) boucle "mesurer a l'arret -> planifier
-> executer en aveugle -> re-mesurer" repetee jusqu'a tolerance. *Choix* : (b).
Une loi continue asservie a 2.5 Hz sur un angle dont le signe n'est pas confirme
(cf. section CONVENTIONS) oscille au lieu de converger ; la boucle iterative
rend chaque segment verifiable, borne l'erreur par la re-mesure, et degrade
proprement (si une iteration empire les choses, on le VOIT a l'iteration
suivante et on s'arrete). C'est le meme paradigme "look-and-move" que celui
deja retenu pour ALIGN (cf. commentaire ALIGN dans `rm_cam_beacon.py`).
Condition qui reviserait ce choix : Carolus deplace sur le Pi (F0.C du roadmap)
faisant monter `/pose` a >10 Hz.


=====================================================================
CONVENTIONS DE SIGNE — le vrai risque de ce module
=====================================================================
Ce projet a un historique de signes non confirmes (GIM_YAW_SIGN confirme
seulement le 2026-06-26 par test ; GIM_PITCH_SIGN toujours non confirme apres
l'incident BUG-058 ; "signe EP non confirme" note pour `/odom` ; meme reserve
dans `gimbal_bearing.py`). Le docking depend de PLUSIEURS de ces signes, donc :

  * Signe de rotation du chassis (`cmd_vel.angular.z`) -> MESURE AU DEMARRAGE
    par `_probe_turn_sign()` (petite rotation, on regarde dans quel sens le yaw
    `/odom` bouge). Aucune constante a deviner.
  * Signe de rotation du gimbal (`gimbal_vel.angular.z`) -> MESURE de la meme
    facon par `_probe_gimbal_sign()` sur `/carolus/gimbal_yaw_rel`.
  * Orientation de la balise (`p.yaw`) -> NE PEUT PAS s'auto-calibrer : il faut
    savoir ou pointe physiquement la balise. D'ou le mode CALIBRATE (voir
    ci-dessous), a passer UNE FOIS avant le premier docking reel.

⚠️ Tant que BEACON_YAW_SIGN / BEACON_YAW_OFFSET_DEG n'ont pas ete etablis par
le mode CALIBRATE, ce module refuse de docker (garde-fou `_yaw_convention_ok`)
et se rabat sur un simple maintien de distance, comportement equivalent a
l'APPROACH existant. C'est volontaire : un signe faux ferait tourner le robot
DANS LE MAUVAIS SENS autour de la balise, en s'eloignant de la solution.


=====================================================================
UTILISATION
=====================================================================
Prerequis : `rm_cam_beacon.py` tourne (il fournit `/odom`,
`/carolus/gimbal_yaw_rel` et consomme `/carolus/cmd_vel`), Carolus tourne (il
fournit `/pose`), balise visible.

    python3 beacon_docking.py

**GUI-integre (2026-07-27)** : lance depuis `carolus_launcher.py` (5e terminal
T5), panneau "DOCKING BALISE" avec boutons CALIBRATE / CAL STEP 2 / START /
ABORT et un statut live -- voir `shortcuts/README.md`.

Ou manuellement, depuis un autre terminal :

    # calibration de la convention d'orientation balise (a faire une fois,
    # en 2 clics independants -- pas de minuteur, chaque etape attend l'ordre
    # explicite suivant, a ton rythme) :
    #   1. positionner le robot EN FACE de la balise (~1m), puis :
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'CALIBRATE'"
    #   2. deplacer le robot d'~30 deg a droite de la balise, puis :
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'CALSTEP2'"

    # docking complet (phases 1+2+3 enchainees)
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'START'"

    # test isole (2026-07-28) : alignement chassis SEUL, n'avance jamais
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'ALIGN_ONLY'"

    # test isole (2026-07-28) : avance SEULE, verifie l'alignement avant de
    # bouger et refuse (NOT_ALIGNED) si le chassis n'est pas deja aligne --
    # lancer ALIGN_ONLY avant si besoin
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'APPROACH_ONLY'"

    # arret d'urgence
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'ABORT'"

Etat publie en continu sur `/carolus/dock_status` (String), et logue en
`[DOCKSTATUS] status=... yaw_validated=...` (parsable par `carolus_launcher.py`,
meme mecanisme que `[BEACON]`) en plus des lignes `[DOCK] ...` classiques.

Ce fichier ne modifie AUCUN fichier existant et n'ouvre AUCUNE connexion SDK.
"""

import math
import threading
import time

# Imports ROS tolerants : la geometrie de ce module (`plan_maneuver`) est une
# fonction pure, testable sur une machine sans ROS installe/source
# (`python3 beacon_docking.py --selftest`). On n'echoue donc qu'au moment de
# demarrer reellement le nœud, pas a l'import.
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32, String
    _ROS_AVAILABLE = True
except ImportError as _e:          # pragma: no cover — depend de l'environnement
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = _e


# =========================================================
# CONFIG
# =========================================================

# ── Cible de docking ─────────────────────────────────────────────────────────
DOCK_DISTANCE_M = 0.20    # distance finale robot<->balise (2026-07-27, demande
                          # explicite utilisateur -- avant : 0.70, aligne sur
                          # STOP_DISTANCE_M de rm_cam_beacon.py)

# Mode simplifie (2026-07-27, demande explicite utilisateur pour le premier
# test materiel) : tourner pour faire face a la balise (bearing -> 0), PUIS
# avancer tout droit jusqu'a DOCK_DISTANCE_M. Pas d'alignement sur l'axe
# frontal de la balise (ignore offaxis), pas de boucle iterative -- une seule
# mesure, un seul tour, une seule avance. La manœuvre complete (plan_maneuver,
# point de ligne-up, iterations jusqu'a convergence) reste dans le code
# ci-dessous, desactivee par ce flag -- remettre a False pour la reactiver.
SIMPLE_APPROACH_ONLY = True

# ── Tolerances de fin (le docking s'arrete quand les 3 sont satisfaites) ─────
TOL_RANGE_M      = 0.06   # |range - DOCK_DISTANCE_M| accepte (m)
TOL_OFFAXIS_DEG  = 8.0    # |angle hors-axe| accepte (deg) — "de face"
TOL_BEARING_DEG  = 6.0    # |balise pas centree devant le robot| accepte (deg)

# ── Boucle iterative ─────────────────────────────────────────────────────────
MAX_ITERATIONS   = 5      # au-dela : on considere que ca ne converge pas
MIN_PROGRESS_DEG = 2.0    # si une iteration ne gagne pas au moins ca sur
                          # l'angle hors-axe, on arrete (evite de tourner en
                          # rond si un signe est faux malgre la calibration)

# ── Mesure (a l'arret, robot immobile) ───────────────────────────────────────
MEAS_SAMPLES     = 7      # nb de poses agregees par mesure (mediane)
MEAS_TIMEOUT_S   = 8.0    # abandon si on n'a pas MEAS_SAMPLES a temps
MEAS_MAX_SPREAD_DEG = 25.0  # dispersion max toleree sur l'angle hors-axe,
                            # au-dela la mesure est jugee non fiable

# ── Vitesses (volontairement basses : manœuvre de precision) ────────────────
TURN_WZ_DEG_S    = 25.0   # vitesse de rotation chassis pendant la manœuvre
DRIVE_VX_M_S     = 0.12   # vitesse d'avance pendant la manœuvre
CMD_RATE_HZ      = 10.0   # republication cmd_vel (recepteur coupe a 0.5s,
                          # cf. MANUAL_CMDVEL_TIMEOUT dans rm_cam_beacon.py)

# ── Primitives de mouvement ──────────────────────────────────────────────────
TURN_TOL_DEG     = 2.0    # precision d'arret d'une rotation
TURN_TIMEOUT_MAX_S = 15.0 # securite : jamais tourner plus longtemps que ca
SEQUENCE_TIMEOUT_S = 45.0 # 2026-07-28 : budget global pour phases 1+2+3 du
                          # pipeline simple, cumule (chaque phase a deja son
                          # propre timeout, ceci est une securite en plus, pas
                          # un remplacement) -- valeur de depart a calibrer,
                          # pas issue d'une mesure existante
DRIVE_TOL_M      = 0.03   # precision d'arret d'une avance
DRIVE_TIMEOUT_MAX_S = 20.0
MAX_SEGMENT_M    = 2.5    # avance max executee d'un seul tenant. Un plan plus
                          # long n'est PAS une erreur (un docking tres hors-axe
                          # a longue portee demande un vrai detour) : on tronque
                          # et on laisse l'iteration suivante re-planifier depuis
                          # une mesure fraiche — plus sur que d'avancer 3 m en
                          # aveugle sur une mesure a 2.5 Hz.
ABSURD_SEGMENT_M = 10.0   # au-dela, la mesure est forcement fausse -> abandon

# ── Sondage des signes (auto-calibration au demarrage de la manœuvre) ───────
PROBE_TURN_DEG_S = 20.0   # vitesse de la rotation de sondage
PROBE_DURATION_S = 0.8    # duree de la rotation de sondage
PROBE_MIN_DELTA_DEG = 1.5 # en dessous, on considere que rien n'a bouge

# ── Gimbal ───────────────────────────────────────────────────────────────────
# La mesure se fait gimbal aligne sur le chassis (yaw_rel ~ 0) : ainsi le repere
# camera == le repere chassis, et toute la geometrie ci-dessous se passe du
# signe de yaw_rel (qui n'est pas confirme). C'est la simplification centrale
# de ce module.
GIMBAL_NULL_TOL_DEG  = 3.0
GIMBAL_NULL_SPEED    = 25.0
GIMBAL_NULL_TIMEOUT_S = 12.0
# 2026-07-30 : la phase 1 declarait la nacelle alignee sur UNE seule lecture
# sous tolerance, alors que la phase 2 exigeait deja 3 lectures consecutives
# (ALIGN_CONSECUTIVE_OK). C'etait l'inverse de la priorite reelle : la sortie
# de la phase 1 est LA reference de tout le reste du pipeline, donc celle qui
# merite le plus d'etre confirmee. Bruit P4P de l'ordre de 1 deg (journal
# 2026-07-23) pour une tolerance de 3 deg -> une lecture isolee peut passer
# sous tolerance par bruit seul. Les lectures comptees sont des poses
# DISTINCTES (stamp different) : a 10 Hz de boucle pour 2.5 Hz de vision, le
# meme message aurait ete compte 3 fois sinon.
GIMBAL_CONFIRM_OK    = 3
# Duree sans pose fraiche au-dela de laquelle la phase 1 conclut "balise pas
# en vue" plutot que "n'a pas converge" -- deux causes distinctes qui
# produisaient jusqu'ici le meme message de timeout. Cascade du 2026-07-29 :
# 8 runs sur 11 morts en phase 1, sans que les logs permettent de dire
# lequel des deux cas s'etait produit.
GIMBAL_NO_POSE_S     = 4.0

# ── Verification terminale de l'alignement (2026-07-30) ─────────────────────
# La phase 2 asservit `yaw_rel` -> 0, ce qui signifie "chassis aligne sur la
# NACELLE". Que le chassis soit aligne sur la BALISE n'en decoule QUE si la
# nacelle pointe encore la balise a cet instant -- ce qui n'etait jamais
# verifie. Preuve materielle (2026-07-29, run 1) : SUCCES annonce avec
# yaw_rel=+0.6 deg alors que l'ecart image valait -26.3 deg au meme instant.
#
# Le correctif n'est PAS de re-bloquer sur l'ecart image (essaye le matin du
# 2026-07-29, retire l'apres-midi a juste titre : la valeur etait lue pendant
# la rotation, donc non pertinente au moment de la decision). C'est de
# MESURER proprement -- robot arrete, stabilise, mesure agregee via _measure()
# comme partout ailleurs dans ce module -- puis de CORRIGER le residu par une
# passe supplementaire au lieu de se contenter de le signaler.
ALIGN_SETTLE_S       = 1.2   # arret complet + renouvellement vision (/pose a
                             # 2.5 Hz -> ~3 trames) avant de mesurer
ALIGN_VERIFY_PASSES  = 3     # nb max de passes (nacelle + chassis) enchainees
ALIGN_VERIFY_MIN_GAIN_DEG = 2.0   # gain minimal exige d'une passe a la
                                  # suivante, sinon on arrete (une passe de
                                  # plus ne servirait qu'a user la mecanique)
# Derive max toleree du cap ABSOLU de la nacelle entre la fin de la phase 1 et
# la fin de la phase 2. yaw_ground est un temoin fiable et gratuit depuis que
# H1 est confirmee (2026-07-29 : yaw_ground stable a +0.2 deg pendant que le
# chassis tournait de 104.8 deg). Si la reference a bouge, la conclusion
# "chassis aligne sur la balise" ne tient plus, quelle que soit la valeur de
# yaw_rel.
ALIGN_REF_DRIFT_MAX_DEG = 8.0

# ── Convention d'orientation balise (A ETABLIR PAR LE MODE CALIBRATE) ───────
# psi = BEACON_YAW_SIGN * p.yaw + BEACON_YAW_OFFSET_DEG
# psi est defini comme l'ANGLE HORS-AXE : 0 = le robot est pile sur l'axe
# frontal de la balise (il la voit de face), != 0 = il la voit de biais.
# Valeurs etablies par CALIBRATE le 2026-07-27 (yaw_face de reference,
# delta mesure a +57.2 deg apres un deplacement d'environ 45 deg a droite).
BEACON_YAW_SIGN       = +1.0
BEACON_YAW_OFFSET_DEG = +2.4
# Passer a True UNIQUEMENT apres avoir valide les deux valeurs ci-dessus par
# le mode CALIBRATE sur le materiel. False = refus de docker (repli sur simple
# maintien de distance), cf. entete.
BEACON_YAW_VALIDATED  = True

POSE_TIMEOUT_S = 1.5      # meme valeur que rm_cam_beacon.py


# =========================================================
# Helpers
# =========================================================

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def angle_diff_deg(a, b):
    """Plus petite difference angulaire signee a-b, en degres, dans [-180, 180].
    (Meme helper que `_angle_diff_deg` de rm_cam_beacon.py — duplique
    volontairement : ce module ne doit importer aucun fichier existant pour
    rester deployable seul.)"""
    return ((a - b + 180.0) % 360.0) - 180.0


def median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


# 2026-07-28 : logique de decision de `_align_chassis_yaw_rel` extraite en
# fonctions pures (memes principes que `plan_maneuver`) pour etre testable
# via --selftest, sans ROS ni robot.

def chassis_align_tick(yaw_rel, deadband_deg, consecutive_ok):
    """Nouveau compteur de lectures consecutives dans la tolerance (0 si hors
    tolerance -- remet le compteur a zero)."""
    if abs(yaw_rel) < deadband_deg:
        return consecutive_ok + 1
    return 0


def chassis_no_progress(err_before, err_now, min_gain_deg):
    """True si l'erreur absolue n'a pas assez diminue entre deux controles
    espaces dans le temps (divergence ou stagnation)."""
    return (err_before - err_now) < min_gain_deg


def chassis_is_blocked(yaw_rel_ref, yaw_rel_now, min_delta_deg, commands_sent):
    """True si des commandes non nulles ont deja ete envoyees mais yaw_rel n'a
    quasi pas bouge depuis la derniere reference -- chassis physiquement
    bloque (butee, roue coincee, etc.).

    2026-07-30 : la comparaison passe desormais par `angle_diff_deg`. Avant,
    deux angles pourtant deja normalises etaient soustraits directement, ce
    qui casse au passage de +/-180 deg : un chassis STRICTEMENT immobile a
    ref=+179 / now=-179 donnait |diff|=358 deg et passait donc pour "en
    mouvement" -- exactement le cas ou il fallait detecter un blocage."""
    return commands_sent > 0 and \
        abs(angle_diff_deg(yaw_rel_now, yaw_rel_ref)) < min_delta_deg


def gimbal_confirm_tick(err_deg, tol_deg, consecutive_ok):
    """Meme discipline que `chassis_align_tick`, appliquee a l'ecart image de
    la phase 1 : compteur de lectures consecutives sous tolerance, remis a
    zero des qu'une lecture sort de la tolerance (2026-07-30)."""
    if abs(err_deg) < tol_deg:
        return consecutive_ok + 1
    return 0


def align_verify_verdict(residual_deg, tol_deg, prev_residual_deg,
                         min_gain_deg, passes_done, max_passes):
    """Decide de la suite apres une passe d'alignement verifiee (2026-07-30).

    Fonction PURE (aucun I/O, aucun etat) -- donc couverte par --selftest,
    comme `plan_maneuver` et les trois helpers `chassis_*`.

    `residual_deg` est le gisement de la balise mesure dans le repere CHASSIS,
    robot a l'arret. Renvoie l'une des chaines :
      * "ok"        -- residu dans la tolerance : alignement reellement atteint
      * "retry"     -- hors tolerance, budget restant et progres suffisant :
                       une passe de plus vaut le coup
      * "no_gain"   -- la passe precedente n'a pas fait gagner min_gain_deg :
                       insister ne ferait qu'user la mecanique
      * "exhausted" -- budget de passes epuise
    """
    if abs(residual_deg) <= tol_deg:
        return "ok"
    if passes_done >= max_passes:
        return "exhausted"
    if prev_residual_deg is not None and \
            (abs(prev_residual_deg) - abs(residual_deg)) < min_gain_deg:
        return "no_gain"
    return "retry"


class DockAbort(Exception):
    """Levee des qu'un ABORT est demande ou qu'une securite se declenche.
    Remonte jusqu'a `_dock_sequence` qui arrete le robot proprement."""


# =========================================================
# Nœud
# =========================================================

class BeaconDocking:

    def __init__(self):
        # --- etat mesure ---
        self._pose = None          # (x, y, z, yaw_deg, stamp) brut camera
        self._pose_lock = threading.Lock()
        self._yaw_rel = 0.0
        self._yaw_rel_lock = threading.Lock()
        # 2026-07-29 (BUG-080, instrumentation) : /carolus/gimbal_yaw_ground
        # existe et est publie par rm_cam_beacon.py depuis 2026-07-27 mais
        # n'avait jamais ete ecoute ici. On l'ecoute desormais uniquement
        # pour LOGUER yaw_ground et (yaw_ground - yaw_rel) a chaque etape de
        # _align_chassis_yaw_rel -- pas pour changer le comportement. But :
        # que le PROCHAIN run reel sur materiel (quel qu'il soit, pas
        # necessairement le protocole isole de 18-protocole-discriminant-
        # bug080.md) capture automatiquement la preuve qui confirme ou
        # refute H1, sans dependre d'un test dedie pour l'obtenir.
        self._yaw_ground = 0.0
        self._yaw_ground_lock = threading.Lock()
        self._odom = None          # (x, y, yaw_deg)
        self._odom_lock = threading.Lock()

        # --- etat manœuvre ---
        self._abort = False
        self._busy = False
        self._turn_sign = None     # +1/-1, determine par _probe_turn_sign()
        self._gimbal_sign = None   # +1/-1, determine par _probe_gimbal_sign()
        # Cap ABSOLU de la nacelle (yaw_ground) au moment ou la phase 1 a
        # declare la nacelle alignee sur la balise. Sert de temoin de derive
        # de la reference pendant la phase 2 (2026-07-30, cf.
        # ALIGN_REF_DRIFT_MAX_DEG). None tant qu'aucune phase 1 n'a reussi.
        self._gimbal_ref_ground = None
        self._cal_yaw_face = None  # resultat etape 1/2 de CALIBRATE, en attente de l'etape 2
        self._status = "IDLE"

        # --- detection pose republiee (2026-07-28) ---
        # carolus_astrobee.cpp::getFilteredPose() (lignes 560-598) republie
        # l'ANCIENNE pose (valeurs brutes identiques) avec un header.stamp
        # frais quand une nouvelle detection est jugee trop differente et
        # rejetee (lignes 569-576, 583-590). Consequence : _fresh_pose() (basee
        # sur l'heure de reception ROS) ne peut PAS distinguer une detection
        # reellement nouvelle d'une republication. Seul signal exploitable
        # sans toucher au .cpp : comparer les valeurs brutes (x,y,z,yaw) entre
        # deux receptions successives -- une republication les rend bit-a-bit
        # identiques, ce qu'une detection P4P independante ne produit
        # essentiellement jamais (bruit numerique du solveur). Limite connue :
        # heuristique, pas une garantie -- documentee aussi en tete de fichier.
        self._last_raw_pose_values = None   # (x, y, z, yaw_deg) de la derniere reception
        self._pose_repeat_count = 0         # nb de receptions consecutives a l'identique

        # --- ROS ---
        self.pub_cmd = rospy.Publisher("/carolus/cmd_vel", Twist, queue_size=1)
        self.pub_gim = rospy.Publisher("/carolus/gimbal_vel", Twist, queue_size=1)
        self.pub_mode = rospy.Publisher("/carolus/mode", String, queue_size=1)
        self.pub_lock = rospy.Publisher("/carolus/gimbal_lock", String, queue_size=1)
        self.pub_status = rospy.Publisher("/carolus/dock_status", String, queue_size=1)

        rospy.Subscriber("/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/carolus/gimbal_yaw_rel", Float32, self._yaw_rel_cb)
        rospy.Subscriber("/carolus/gimbal_yaw_ground", Float32, self._yaw_ground_cb)
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/carolus/dock", String, self._cmd_cb)

        rospy.Timer(rospy.Duration(0.5), self._status_tick)

        rospy.loginfo("[DOCK] pret — commandes sur /carolus/dock : START / CALIBRATE / CALSTEP2 / ABORT")
        if not BEACON_YAW_VALIDATED:
            rospy.logwarn("[DOCK] BEACON_YAW_VALIDATED=False — le docking complet est "
                          "DESACTIVE tant que la convention d'orientation balise n'a pas "
                          "ete etablie (mode CALIBRATE). START fera un simple maintien "
                          "de distance.")

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------

    def _pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        if not all(map(math.isfinite, [p.x, p.y, p.z, q.x, q.y, q.z, q.w])):
            return
        # Extraction du yaw balise — MEME formule que `_pose_cb` de
        # rm_cam_beacon.py (rotation autour de l'axe y camera). Volontairement
        # identique : c'est la valeur deja loguee en `byaw` depuis des mois, donc
        # la seule pour laquelle on a un historique terrain. Sa limite est connue
        # et documentee dans le compte-rendu de revue joint a ce module (exacte
        # pour une rotation pure autour de y, approchee des que la balise est
        # inclinee ou la nacelle pitchee).
        siny = 2.0 * (q.w * q.y + q.z * q.x)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        raw = (p.x, p.y, p.z, yaw_deg)
        with self._pose_lock:
            self._pose = (p.x, p.y, p.z, yaw_deg, time.time())
            if raw == self._last_raw_pose_values:
                self._pose_repeat_count += 1
            else:
                self._pose_repeat_count = 0
            self._last_raw_pose_values = raw

    def _get_pose_repeat_count(self):
        with self._pose_lock:
            return self._pose_repeat_count

    def _yaw_rel_cb(self, msg):
        with self._yaw_rel_lock:
            self._yaw_rel = float(msg.data)

    def _yaw_ground_cb(self, msg):
        with self._yaw_ground_lock:
            self._yaw_ground = float(msg.data)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        with self._odom_lock:
            self._odom = (p.x, p.y, yaw)

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "ABORT":
            self._abort = True
            rospy.logwarn("[DOCK] ABORT demande")
            return
        if self._busy:
            rospy.logwarn(f"[DOCK] {cmd} ignore : manœuvre deja en cours")
            return
        if cmd == "START":
            threading.Thread(target=self._run, args=(self._dock_sequence,),
                             daemon=True).start()
        elif cmd == "ALIGN_ONLY":
            threading.Thread(target=self._run, args=(self._align_only,),
                             daemon=True).start()
        elif cmd == "APPROACH_ONLY":
            threading.Thread(target=self._run, args=(self._approach_only,),
                             daemon=True).start()
        elif cmd == "CALIBRATE":
            threading.Thread(target=self._run, args=(self._calibrate_step1,),
                             daemon=True).start()
        elif cmd == "CALSTEP2":
            threading.Thread(target=self._run, args=(self._calibrate_step2,),
                             daemon=True).start()

    def _status_tick(self, _event):
        self.pub_status.publish(String(data=self._status))
        # Ligne loggee (en plus du topic /carolus/dock_status) pour etre parsee
        # par carolus_launcher.py exactement comme [BEACON]/[BEACONPOS] deja
        # publies par rm_cam_beacon.py -- meme mecanisme, pas de nouveau pattern.
        rospy.loginfo_throttle(
            1.0, f"[DOCKSTATUS] status={self._status} yaw_validated={BEACON_YAW_VALIDATED}")

    # ---------------------------------------------------------
    # Accesseurs
    # ---------------------------------------------------------

    def _get_pose(self):
        with self._pose_lock:
            return self._pose

    def _fresh_pose(self):
        p = self._get_pose()
        return p is not None and (time.time() - p[4]) < POSE_TIMEOUT_S

    def _get_yaw_rel(self):
        with self._yaw_rel_lock:
            return self._yaw_rel

    def _get_yaw_ground(self):
        with self._yaw_ground_lock:
            return self._yaw_ground

    def _get_odom(self):
        with self._odom_lock:
            return self._odom

    def _check_abort(self):
        if self._abort or rospy.is_shutdown():
            raise DockAbort()

    # ---------------------------------------------------------
    # Commandes bas niveau
    # ---------------------------------------------------------

    def _send_cmd(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub_cmd.publish(t)

    def _send_gimbal(self, yaw_speed=0.0):
        t = Twist()
        t.angular.z = yaw_speed
        self.pub_gim.publish(t)

    def _stop(self):
        for _ in range(3):
            self._send_cmd(0.0, 0.0)
            self._send_gimbal(0.0)
            time.sleep(0.05)

    def _take_control(self):
        """Passe le robot en MANUEL (seul mode ou `/carolus/cmd_vel` et
        `/carolus/gimbal_vel` sont relayes au SDK) et coupe le LOCK balise
        (sinon le tick LOCK ignore nos commandes gimbal, cf. l'arbitrage a 3
        priorites de la boucle MANUEL de rm_cam_beacon.py)."""
        self.pub_mode.publish(String(data="MANUAL"))
        self.pub_lock.publish(String(data="OFF"))
        time.sleep(0.4)   # laisser le mode s'appliquer avant de commander

    # ---------------------------------------------------------
    # Auto-calibration des signes de rotation
    # ---------------------------------------------------------

    def _probe_turn_sign(self):
        """Determine le signe de `cmd_vel.angular.z` qui fait AUGMENTER le yaw
        `/odom`. Evite d'avoir a connaitre a l'avance la convention EP (non
        confirmee dans ce projet, cf. entete)."""
        if self._turn_sign is not None:
            return
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        yaw0 = od[2]
        t0 = time.time()
        while time.time() - t0 < PROBE_DURATION_S:
            self._check_abort()
            self._send_cmd(0.0, PROBE_TURN_DEG_S)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        time.sleep(0.4)   # laisser le chassis s'immobiliser avant de relire
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        delta = angle_diff_deg(od[2], yaw0)
        if abs(delta) < PROBE_MIN_DELTA_DEG:
            rospy.logerr(f"[DOCK] sondage rotation : le chassis n'a pas bouge "
                         f"({delta:.1f} deg) — robot bloque, mode non MANUEL, ou "
                         f"/odom absent")
            raise DockAbort()
        self._turn_sign = 1.0 if delta > 0 else -1.0
        rospy.loginfo(f"[DOCK] sondage rotation : wz>0 -> yaw {delta:+.1f} deg "
                      f"-> turn_sign={self._turn_sign:+.0f}")

    def _probe_gimbal_sign(self):
        """Idem pour la nacelle, sur `/carolus/gimbal_yaw_rel`. Reessaie une
        fois, sens oppose et duree doublee, si le 1er sondage ne bouge pas
        (2026-07-27, observe sur materiel : la nacelle peut etre proche d'une
        butee mecanique dans le 1er sens tente, surtout apres plusieurs essais
        CALIBRATE/docking dans la meme session)."""
        if self._gimbal_sign is not None:
            return
        attempts = [(PROBE_TURN_DEG_S, PROBE_DURATION_S),
                    (-PROBE_TURN_DEG_S, PROBE_DURATION_S * 2)]
        for n, (speed, dur) in enumerate(attempts, start=1):
            y0 = self._get_yaw_rel()
            t0 = time.time()
            while time.time() - t0 < dur:
                self._check_abort()
                self._send_gimbal(speed)
                time.sleep(1.0 / CMD_RATE_HZ)
            self._send_gimbal(0.0)
            time.sleep(0.4)
            delta = angle_diff_deg(self._get_yaw_rel(), y0)
            if abs(delta) >= PROBE_MIN_DELTA_DEG:
                self._gimbal_sign = 1.0 if (delta > 0) == (speed > 0) else -1.0
                rospy.loginfo(f"[DOCK] sondage nacelle (essai {n}/2, cmd={speed:+.0f}) : "
                              f"yaw_rel {delta:+.1f} deg -> gimbal_sign={self._gimbal_sign:+.0f}")
                return
            rospy.logwarn(f"[DOCK] sondage nacelle (essai {n}/2, cmd={speed:+.0f}) : "
                          f"nacelle immobile ({delta:.1f} deg)")
        rospy.logwarn("[DOCK] sondage nacelle : immobile dans les 2 sens apres 2 essais "
                      "— alignement gimbal INDISPONIBLE ce run (0 = inutilisable)")
        self._gimbal_sign = 0.0   # 0 = gimbal inutilisable, on s'en passe

    # ---------------------------------------------------------
    # Primitives de mouvement (asservies sur /odom)
    # ---------------------------------------------------------

    def _null_gimbal(self):
        """Ramene la nacelle dans l'axe du chassis (yaw_rel -> 0) pour que le
        repere camera coincide avec le repere chassis pendant la mesure.

        NB : on n'utilise PAS `gimbal.recenter()` (dispo via
        `/carolus/gimbal_recenter`) — recenter ramene la nacelle a son repere
        POWER-ON, qui n'a aucune raison d'etre aligne avec le chassis courant
        (c'est explicitement documente comme "independant de l'orientation du
        chassis" dans rm_cam_beacon.py). Ici on veut yaw_rel=0, donc on asservit
        sur yaw_rel."""
        if self._gimbal_sign == 0.0:
            return
        self._probe_gimbal_sign()
        if self._gimbal_sign == 0.0:
            return
        t0 = time.time()
        while time.time() - t0 < GIMBAL_NULL_TIMEOUT_S:
            self._check_abort()
            err = self._get_yaw_rel()          # cible = 0
            if abs(err) < GIMBAL_NULL_TOL_DEG:
                break
            # On veut faire DECROITRE yaw_rel : commande de signe oppose a
            # l'erreur, corrigee par le signe mesure au sondage.
            cmd = -self._gimbal_sign * clamp(err, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
            cmd = clamp(cmd, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
            self._send_gimbal(cmd)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._send_gimbal(0.0)
        rospy.loginfo(f"[DOCK] gimbal aligne chassis : yaw_rel={self._get_yaw_rel():.1f} deg")

    def _turn_by(self, delta_deg):
        """Tourne le chassis de delta_deg (signe = convention geometrique de ce
        module : positif = vers la DROITE de la camera, cf. `_measure`).
        Asservi sur le yaw `/odom`, donc insensible a la convention de signe du
        SDK (elle a ete mesuree par `_probe_turn_sign`)."""
        if abs(delta_deg) < TURN_TOL_DEG:
            return
        self._probe_turn_sign()
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        yaw0 = od[2]
        rospy.loginfo(f"[DOCK] rotation : cible={delta_deg:+.1f} deg (repere camera, "
                      f"+ = droite) turn_sign={self._turn_sign:+.0f} "
                      f"yaw_odom_brut_depart={yaw0:+.1f} deg")
        # Duree theorique + marge x3, plafonnee : garde-fou si /odom se fige.
        timeout = min(TURN_TIMEOUT_MAX_S,
                      max(3.0, 3.0 * abs(delta_deg) / max(TURN_WZ_DEG_S, 1.0)))
        # `delta_deg` est exprime dans le repere geometrique du module ; le signe
        # de commande a appliquer est celui mesure au sondage. Le sondage a
        # etabli le lien entre wz>0 et le sens de variation du yaw /odom ; on
        # suppose ici que le yaw /odom croit dans le meme sens que notre angle
        # geometrique. Si le test materiel montre l'inverse, c'est CE signe
        # (et lui seul) qu'il faut inverser.
        target = delta_deg
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._check_abort()
            od = self._get_odom()
            if od is None:
                break
            done = angle_diff_deg(od[2], yaw0) * self._turn_sign
            remaining = target - done
            if abs(remaining) < TURN_TOL_DEG:
                break
            wz = self._turn_sign * clamp(remaining, -TURN_WZ_DEG_S, TURN_WZ_DEG_S)
            # plancher : sous ~5 deg/s le chassis ne demarre pas franchement
            if 0.0 < abs(wz) < 5.0:
                wz = math.copysign(5.0, wz)
            self._send_cmd(0.0, wz)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        time.sleep(0.3)
        od = self._get_odom()
        if od is not None:
            raw_delta = angle_diff_deg(od[2], yaw0)
            got = raw_delta * self._turn_sign
            rospy.loginfo(f"[DOCK] rotation demandee={delta_deg:+.1f} deg obtenue={got:+.1f} deg "
                          f"(yaw_odom_brut : {yaw0:+.1f} -> {od[2]:+.1f} deg, delta_brut="
                          f"{raw_delta:+.1f} deg, turn_sign={self._turn_sign:+.0f})")

    def _track_beacon_gimbal_tick(self):
        """Une correction de nacelle pour garder la balise centree dans l'image
        PENDANT l'avance (2026-07-27, demande utilisateur -- meme objectif que
        LOCK dans rm_cam_beacon.py, mais integre ici : LOCK est coupe pendant
        tout `_dock_sequence` par `_take_control`, les deux systemes ne
        peuvent pas commander la nacelle en meme temps sans se battre).

        Reutilise EXACTEMENT la formule validee de `_null_gimbal`
        (cmd = -gimbal_sign * clamp(erreur)), avec une erreur differente :
        `_null_gimbal` annule yaw_rel (camera alignee CHASSIS, pour la mesure
        a l'arret) ; ici on annule le decalage lateral de la balise DANS
        L'IMAGE (p.x/p.z, meme formule que `_measure`/LOCK), pour que la
        camera SUIVE la balise pendant que le robot avance. Generalisation
        raisonnee du signe sonde (meme sens physique : les deux erreurs
        diminuent quand la nacelle tourne dans le meme sens) mais PAS
        verifiee independamment sur materiel -- a surveiller au premier test.

        2026-07-29 : retourne desormais l'erreur image calculee (ou None si
        aucune pose fraiche exploitable), pour permettre a un appelant de
        verifier que la nacelle est REELLEMENT sur la balise a cet instant,
        au lieu de deviner. Reutilise par `_align_chassis_yaw_rel` (voir
        BUG-080 : `yaw_rel` seul peut ne pas etre fiable ; `err_img`, lui,
        ne depend d'aucune hypothese sur la stabilisation de la nacelle --
        c'est une lecture visuelle directe, immune a l'historique de
        rotation de la plateforme). Changement de signature retrocompatible
        : le seul appelant existant (`_drive_by`) ignorait deja la valeur
        de retour."""
        if self._gimbal_sign is None or self._gimbal_sign == 0.0:
            return None
        p = self._get_pose()
        if p is None or not self._fresh_pose() or abs(p[2]) < 0.05:
            self._send_gimbal(0.0)
            return None
        err = math.degrees(math.atan2(p[0], abs(p[2])))
        if abs(err) < GIMBAL_NULL_TOL_DEG:
            self._send_gimbal(0.0)
            return err
        cmd = -self._gimbal_sign * clamp(err, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
        self._send_gimbal(cmd)
        return err

    def _align_gimbal_to_beacon(self, timeout_s=GIMBAL_NULL_TIMEOUT_S):
        """Phase 1/3 du pipeline simple : pointe la nacelle sur la balise,
        chassis immobile. Reutilise `_track_beacon_gimbal_tick` (meme
        asservissement que pendant l'avance).

        Renvoie True si la nacelle est reellement alignee (a servir de
        reference fiable pour la phase 2), False sinon -- 2026-07-27 : avant
        ce fix, un echec silencieux ici (gimbal_sign=0, nacelle qui ne bouge
        pas) laissait quand meme la phase 2 s'aligner sur une reference
        arbitraire, cause directe d'une collision observee sur materiel."""
        if self._gimbal_sign is None:
            self._probe_gimbal_sign()
        if self._gimbal_sign == 0.0:
            rospy.logerr("[DOCK] phase 1 abandonnee : nacelle inutilisable "
                         "(sondage sans mouvement) — pas de reference fiable")
            return False
        t0 = time.time()
        consecutive_ok = 0
        last_counted_stamp = None   # ne compter que des poses DISTINCTES
        t_last_pose = t0
        no_pose_warned = False
        while time.time() - t0 < timeout_s:
            self._check_abort()
            self._track_beacon_gimbal_tick()
            p = self._get_pose()
            if p is not None and self._fresh_pose() and abs(p[2]) > 0.05:
                t_last_pose = time.time()
                # Une lecture ne compte que si elle vient d'un NOUVEAU message
                # (2026-07-30) : la boucle tourne a CMD_RATE_HZ=10 Hz pour une
                # vision a 2.5 Hz, donc sans ce filtre la meme pose serait
                # comptee 3 fois d'affilee et la "confirmation" ne confirmerait
                # rien du tout.
                if p[4] != last_counted_stamp:
                    last_counted_stamp = p[4]
                    err = math.degrees(math.atan2(p[0], abs(p[2])))
                    consecutive_ok = gimbal_confirm_tick(
                        err, GIMBAL_NULL_TOL_DEG, consecutive_ok)
                    if consecutive_ok >= GIMBAL_CONFIRM_OK:
                        self._send_gimbal(0.0)
                        # yaw_ground : d'abord instrumentation BUG-080
                        # (2026-07-29), desormais AUSSI la reference de derive
                        # relue en fin de phase 2 (2026-07-30).
                        self._gimbal_ref_ground = self._get_yaw_ground()
                        rospy.loginfo(f"[DOCK] nacelle alignee sur la balise "
                                      f"({consecutive_ok}/{GIMBAL_CONFIRM_OK} poses "
                                      f"distinctes confirmees, "
                                      f"yaw_rel={self._get_yaw_rel():+.1f} deg, "
                                      f"yaw_ground={self._gimbal_ref_ground:+.1f} deg, "
                                      f"err_img={err:+.1f} deg)")
                        return True
            elif (not no_pose_warned) and (time.time() - t_last_pose) > GIMBAL_NO_POSE_S:
                no_pose_warned = True
                rospy.logwarn(f"[DOCK] phase 1 : aucune pose fraiche depuis "
                              f"{GIMBAL_NO_POSE_S}s — la balise semble hors du "
                              f"champ camera, la nacelle ne peut pas s'asservir")
            time.sleep(1.0 / CMD_RATE_HZ)
        self._send_gimbal(0.0)
        # 2026-07-30 : deux causes d'echec bien distinctes, jusqu'ici confondues
        # sous le meme message "timeout sans converger" (cascade 2026-07-29,
        # 8 runs sur 11 morts ici sans diagnostic possible depuis les logs).
        no_pose_for = time.time() - t_last_pose
        if no_pose_for > GIMBAL_NO_POSE_S:
            rospy.logerr(f"[DOCK] phase 1 : ECHEC — BALISE PAS EN VUE "
                         f"(aucune pose fraiche depuis {no_pose_for:.1f}s sur "
                         f"{timeout_s}s de budget). Ce n'est pas un probleme "
                         f"d'asservissement : reorienter le robot/la nacelle "
                         f"vers la balise avant de relancer")
        else:
            rospy.logerr(f"[DOCK] phase 1 : ECHEC — balise vue mais nacelle non "
                         f"convergee en {timeout_s}s "
                         f"({consecutive_ok}/{GIMBAL_CONFIRM_OK} confirmations, "
                         f"yaw_rel={self._get_yaw_rel():+.1f} deg, "
                         f"yaw_ground={self._get_yaw_ground():+.1f} deg) — "
                         f"reference non fiable")
        return False

    def _align_chassis_yaw_rel(self, timeout_s=TURN_TIMEOUT_MAX_S):
        """Phase 2/3 du pipeline simple : tourne le CHASSIS pour annuler
        yaw_rel (la nacelle, deja pointee sur la balise en phase 1, sert de
        reference -- une fois yaw_rel~0 le chassis est de facto oriente vers
        la balise).

        N'utilise PAS `_turn_sign` (sonde sur /odom, suspect n°1 du "robot
        part a l'oppose" observe avec `_turn_by`). Reutilise a la place un
        fait DEJA CONFIRME sur ce robot et ce meme chemin de commande
        (/carolus/cmd_vel -> rm_cam_beacon.py -> chassis.drive_speed) : l'etat
        ALIGN existant (journal 2026-06-26, K_BODY_YAW) a mesure que
        wz = +K*yaw_rel fait DECROITRE yaw_rel (106deg -> 1.6deg avec
        wz=+10). Ce module n'a donc PAS besoin de re-sonder ce signe.

        NB fraicheur : `yaw_rel` vient de `/carolus/gimbal_yaw_rel`, publie
        par rm_cam_beacon.py depuis `gimbal.sub_angle` (retour encodeur SDK
        direct) -- PAS derive de `/pose`. Le risque de pose republiee par
        carolus_astrobee.cpp (getFilteredPose) ne s'applique donc pas ici ;
        il est traite dans `_measure()`/`_pose_cb` a la place, la ou `/pose`
        est reellement consomme.

        2026-07-28 (ajout garde-fous, suite a l'absence totale de verification
        de succes constatee dans la version precedente) :
          - retourne True/False (plus aucun appelant ne peut ignorer un echec)
          - exige ALIGN_CONSECUTIVE_OK lectures consecutives dans la tolerance
            avant de declarer un succes (une seule lecture pouvait etre du
            bruit)
          - detecte l'absence de progres/la divergence (compare l'erreur
            absolue a chaque commande envoyee)
          - detecte un chassis physiquement bloque (yaw_rel ne bouge pas du
            tout malgre des commandes non nulles envoyees pendant un temps
            raisonnable)
          - log structure par commande : erreur avant, commande envoyee,
            erreur apres, progres
          - arret + vitesse nulle garantis sur TOUTE sortie (succes, echec,
            timeout, blocage, divergence)

        2026-07-29 (confirmation visuelle continue, suite a BUG-080) :
        `yaw_rel` est defini par le SDK comme "angle nacelle relatif au
        chassis" -- une lecture chassis-relative, PAS un cap absolu. Si la
        nacelle est activement stabilisee (hypothese H1, non confirmee sur
        ce robot -- voir `research-log/18-protocole-discriminant-bug080.md`),
        `yaw_rel` peut porter un historique de rotations anterieures sans
        rapport avec la balise actuelle -- observe sur materiel a +255.1deg
        pour un decalage physique reel de ~90deg (journal 2026-07-28).

        Plutot que d'attendre la resolution de BUG-080 pour corriger la
        cause, un premier correctif (2026-07-29, matin) a tente de rendre la
        fonction robuste QUELLE QUE SOIT la reponse a H1/H2 en faisant
        suivre la balise a la nacelle EN CONTINU pendant la rotation du
        chassis (`_track_beacon_gimbal_tick`). **Revise le meme jour, apres
        premier test materiel** : ce suivi actif a produit une oscillation
        de l'ecart image (-2.9 -> +13.9 -> -22.9 deg observes en un seul
        run) et le chassis n'a jamais converge (echec "pas de progres") --
        signe probable que la correction visuelle active se bat avec une
        stabilisation deja geree par le firmware de la nacelle (H1).

        Version precedente (2026-07-29, apres-midi) : la nacelle ne recevait
        plus aucune commande dans cette fonction, mais un succes n'etait
        declare que si `yaw_rel` ET l'ecart image passivement lu etaient
        TOUS LES DEUX proches de zero -- sinon "signaux incoherents".

        2026-07-29 (deuxieme test materiel, retrait du garde-fou image) :
        ce test a ete le premier a voir yaw_rel converger sur materiel
        (-106.3 -> -1.4 deg, 3/3 lectures puis au-dela) -- ET a fourni la
        preuve la plus propre a ce jour pour H1 : yaw_ground est reste
        strictement stable (+166.4 -> +166.6 deg, delta=+0.2 deg) pendant
        que yaw_rel absorbait +104.8 deg de rotation chassis, nacelle
        totalement immobile (aucune commande gimbal envoyee ce run). Mais
        le garde-fou image a echoue : ecart_image=+30.5 deg au moment de
        la convergence, jamais revenu sous GIMBAL_NULL_TOL_DEG. Le run
        s'est termine en ECHEC par le timeout global plutot que par le
        garde-fou lui-meme (course de vitesse serree, convergence survenue
        tard dans le budget des timeout_s).

        Puisque yaw_ground prouve que la nacelle n'a PAS change de cap
        absolu, l'ecart image ne peut pas venir d'une mauvaise rotation --
        il vient plus probablement d'une derive laterale du chassis
        pendant la rotation (deja identifiee, item 12 de la roadmap), que
        cette fonction ne commande de toute facon pas (elle ne pilote que
        wz, jamais vy). Exiger une confirmation image revenait donc a
        bloquer sur un defaut qu'aucune correction de cap ne peut
        resoudre. Le garde-fou "signaux incoherents" est retire du chemin
        succes/echec : l'ecart image reste lu et loggue (aucun cout, utile
        pour BUG-080 et pour quantifier la derive de l'item 12, avec un
        `logwarn` si l'ecart est notable), mais ne bloque plus une
        convergence yaw_rel par ailleurs propre.
        """
        ALIGN_DEADBAND_DEG = 2.0
        ALIGN_GAIN = 0.8
        ALIGN_MAX_WZ = 10.0
        # 2026-07-28 -- parametres des garde-fous ci-dessous : valeurs de
        # depart raisonnables (coherentes avec TURN_TOL_DEG=2.0 deg et le
        # comportement observe le 2026-07-27, wz=+10 -> 106->1.6deg en
        # quelques secondes), mais AUCUNE n'est issue d'une campagne de
        # mesure dediee sur ce robot -- a calibrer/ajuster si le test isole
        # de phase 2 declenche un faux positif (blocage/divergence signale
        # alors que le chassis convergeait juste lentement) ou un faux
        # negatif (timeout atteint sans que blocage/divergence n'ait ete
        # detecte plus tot).
        ALIGN_CONSECUTIVE_OK = 3        # lectures consecutives requises dans la tolerance
        NO_PROGRESS_WINDOW_S = 3.0      # fenetre glissante d'evaluation du progres
        NO_PROGRESS_MIN_GAIN_DEG = 1.0  # gain minimal attendu sur cette fenetre
        BLOCKED_CHECK_S = 2.0           # duree avant de juger le chassis bloque
        BLOCKED_MIN_DELTA_DEG = 1.0     # variation min de yaw_rel attendue sur BLOCKED_CHECK_S

        # 2026-07-29 (BUG-081) : yaw_rel brut n'est PAS borne a [-180,180] --
        # observe sur materiel a +255.1 deg (cf. journal 2026-07-28). Utilise
        # tel quel, wz=+K*yaw_rel commande une rotation dans le mauvais sens
        # et 2.4x plus longue que necessaire (255 deg au lieu du chemin court
        # -104.9 deg). On normalise systematiquement via angle_diff_deg (deja
        # utilise ailleurs dans ce fichier) a chaque lecture. Ceci ne corrige
        # PAS la cause de la valeur aberrante (BUG-080, toujours ouvert) --
        # seulement la reaction du controleur face a une valeur hors [-180,180].
        yaw_rel_0 = angle_diff_deg(self._get_yaw_rel(), 0.0)
        # 2026-07-29 (BUG-080, instrumentation) : yaw_ground est loggue ici
        # UNIQUEMENT a des fins de preuve -- ne sert a rien dans la logique
        # de commande ci-dessous. But : que ce run, qu'il reussisse ou
        # echoue, laisse dans les logs de quoi trancher H1 vs H2 (voir
        # 18-protocole-discriminant-bug080.md, tableau d'interpretation) :
        # si yaw_ground reste stable pendant que yaw_rel bouge de l'angle
        # tourne par le chassis, H1 est soutenue.
        yaw_ground_0 = self._get_yaw_ground()
        rospy.loginfo(f"[DOCK] alignement chassis : cible yaw_rel=0, "
                      f"erreur initiale={yaw_rel_0:+.1f} deg "
                      f"(yaw_ground initial={yaw_ground_0:+.1f} deg, "
                      f"yaw_ground-yaw_rel={yaw_ground_0 - yaw_rel_0:+.1f} deg -- "
                      f"BUG-080, preuve H1/H2)")

        consecutive_ok = 0
        t0 = time.time()
        t_last_progress_check = t0
        err_at_last_progress_check = abs(yaw_rel_0)
        t_block_ref = t0
        yaw_rel_block_ref = yaw_rel_0
        commands_sent = 0
        last_img_err = None    # dernier ecart image connu (suivi continu, 2026-07-29)

        def _exit(success, reason, final_err):
            self._stop()
            time.sleep(0.3)
            final_check = self._get_yaw_rel()
            final_ground = self._get_yaw_ground()  # BUG-080 instrumentation, 2026-07-29
            rospy.loginfo(f"[DOCK] alignement chassis termine : "
                          f"{'SUCCES' if success else 'ECHEC'} ({reason}) — "
                          f"erreur finale={final_err:+.1f} deg, "
                          f"yaw_rel post-arret={final_check:+.1f} deg, "
                          f"yaw_ground post-arret={final_ground:+.1f} deg, "
                          f"delta yaw_rel sur ce run={final_check - yaw_rel_0:+.1f} deg, "
                          f"delta yaw_ground sur ce run={final_ground - yaw_ground_0:+.1f} deg, "
                          f"{commands_sent} commande(s) envoyee(s)")
            return success

        while True:
            self._check_abort()
            now = time.time()

            # 2026-07-29, revu le meme jour apres test materiel : la nacelle
            # NE reçoit plus de commande ici. Le premier essai (suivi visuel
            # actif via _track_beacon_gimbal_tick pendant toute la phase 2)
            # a produit une oscillation de l'ecart image (-2.9 -> +13.9 ->
            # -22.9 deg pendant que le chassis tournait) et le run a echoue
            # sur "pas de progres" -- signe que notre correction se bat
            # probablement avec une stabilisation deja active cote firmware
            # (hypothese H1). Retour a une nacelle IMMOBILE pendant la
            # rotation du chassis (comme le 2026-07-28), sur demande
            # utilisateur directe suite a cette observation.
            #
            # La lecture d'ecart image est conservee, mais devient PASSIVE :
            # meme formule que _track_beacon_gimbal_tick (atan2 sur la pose
            # courante), sans jamais appeler _send_gimbal(). Objectif
            # inchange -- une confirmation independante de yaw_rel avant de
            # declarer un succes -- juste sans plus commander la nacelle.
            p = self._get_pose()
            if p is not None and self._fresh_pose() and abs(p[2]) > 0.05:
                last_img_err = math.degrees(math.atan2(p[0], abs(p[2])))

            if now - t0 > timeout_s:
                return _exit(False, f"timeout {timeout_s}s",
                             angle_diff_deg(self._get_yaw_rel(), 0.0))

            yaw_rel = angle_diff_deg(self._get_yaw_rel(), 0.0)  # BUG-081
            err = abs(yaw_rel)

            consecutive_ok = chassis_align_tick(yaw_rel, ALIGN_DEADBAND_DEG, consecutive_ok)
            if consecutive_ok > 0:
                self._send_cmd(0.0, 0.0)
                rospy.loginfo(f"[DOCK] alignement chassis : dans tolerance "
                              f"({yaw_rel:+.1f} deg), {consecutive_ok}/{ALIGN_CONSECUTIVE_OK}, "
                              f"ecart image={'N/A' if last_img_err is None else f'{last_img_err:+.1f} deg'}")
                if consecutive_ok >= ALIGN_CONSECUTIVE_OK:
                    # 2026-07-29 (retrait du garde-fou image, cf. docstring) :
                    # yaw_rel converge -> succes. L'ecart image est encore
                    # loggue (gratuit, utile pour BUG-080 / item 12 derive
                    # laterale) mais ne bloque plus la conclusion -- une
                    # correction de cap ne peut de toute facon pas rattraper
                    # une derive de position.
                    if last_img_err is not None and abs(last_img_err) >= GIMBAL_NULL_TOL_DEG:
                        rospy.logwarn(f"[DOCK] alignement chassis : yaw_rel converge mais "
                                      f"ecart image={last_img_err:+.1f} deg non confirme -- "
                                      f"probable derive laterale (item 12), n'empeche plus "
                                      f"le succes (cf. BUG-080)")
                    img_note = ("image indisponible" if last_img_err is None
                                else f"ecart image={last_img_err:+.1f} deg")
                    return _exit(True, f"convergence stable yaw_rel ({img_note})", yaw_rel)
                time.sleep(1.0 / CMD_RATE_HZ)
                continue

            # Detection de blocage physique : commandes non nulles envoyees
            # depuis BLOCKED_CHECK_S sans que yaw_rel ait bouge de facon
            # significative.
            if now - t_block_ref > BLOCKED_CHECK_S:
                if chassis_is_blocked(yaw_rel_block_ref, yaw_rel, BLOCKED_MIN_DELTA_DEG, commands_sent):
                    return _exit(False, f"chassis bloque (yaw_rel immobile sur "
                                        f"{BLOCKED_CHECK_S}s malgre commande)", yaw_rel)
                t_block_ref = now
                yaw_rel_block_ref = yaw_rel

            # Detection d'absence de progres / divergence sur une fenetre glissante.
            if now - t_last_progress_check > NO_PROGRESS_WINDOW_S:
                if chassis_no_progress(err_at_last_progress_check, err, NO_PROGRESS_MIN_GAIN_DEG):
                    return _exit(False, f"pas de progres sur {NO_PROGRESS_WINDOW_S}s "
                                        f"({err_at_last_progress_check:.1f} -> {err:.1f} deg)", yaw_rel)
                t_last_progress_check = now
                err_at_last_progress_check = err

            wz = clamp(ALIGN_GAIN * yaw_rel, -ALIGN_MAX_WZ, ALIGN_MAX_WZ)
            self._send_cmd(0.0, wz)
            commands_sent += 1
            time.sleep(1.0 / CMD_RATE_HZ)
            # 2026-07-30 (BUG-084) : `erreur_apres` etait la SEULE lecture de
            # yaw_rel de cette fonction a ne pas passer par angle_diff_deg,
            # alors que `erreur_avant` (ligne ~917) l'utilise. Les deux moities
            # de la meme phrase de log affichaient donc le meme angle physique
            # dans deux conventions differentes -- d'ou des lignes comme
            # "erreur_avant=-106.2 deg ... erreur_apres=+253.8 deg" (capture
            # materielle 2026-07-29), qui se lisent comme une divergence
            # spectaculaire alors que rien d'anormal ne se passait. Aucun
            # impact sur la commande envoyee (cette valeur n'est que loggue),
            # mais un impact reel sur le diagnostic : c'est precisement sur ces
            # lignes qu'on s'appuie pour comprendre un run rate.
            err_after = abs(angle_diff_deg(self._get_yaw_rel(), 0.0))
            # yaw_ground ajoute ici (BUG-080, instrumentation 2026-07-29) :
            # un ecart croissant entre yaw_rel et yaw_ground pendant que le
            # chassis tourne est la signature meme de H1 -- gratuit a logger,
            # ne change rien a la commande envoyee.
            rospy.loginfo(f"[DOCK] alignement chassis : erreur_avant={yaw_rel:+.1f} deg "
                          f"commande wz={wz:+.1f} erreur_apres={err_after:+.1f} deg "
                          f"yaw_ground={self._get_yaw_ground():+.1f} deg "
                          f"ecart_image={'N/A' if last_img_err is None else f'{last_img_err:+.1f} deg'}")

    def _verify_alignment(self):
        """Mesure le VRAI gisement de la balise dans le repere CHASSIS, robot
        arrete et stabilise. Renvoie l'angle en degres, ou None si la mesure
        n'a pas pu etre faite (2026-07-30).

        Pourquoi cette fonction existe : jusqu'ici, "chassis aligne sur la
        balise" etait deduit de `yaw_rel ~ 0`, qui ne dit que "chassis aligne
        sur la NACELLE". Le pas manquant -- la nacelle pointe-t-elle encore la
        balise ? -- n'etait jamais mesure. C'est le seul endroit du module ou
        cette question recoit une reponse directe.

        Pourquoi la mesure est valide dans le repere chassis : `_measure()`
        renvoie un gisement dans le repere CAMERA, et documente lui-meme que ce
        repere vaut repere chassis quand la nacelle est alignee sur le chassis.
        C'est exactement la situation ici, par construction : on n'est appele
        qu'apres convergence de la phase 2, donc |yaw_rel| < ALIGN_DEADBAND_DEG
        (2 deg). L'approximation est donc bornee et connue.

        Pourquoi on n'ajoute PAS yaw_rel au resultat pour etre "exact" : le
        signe relatif de yaw_rel et du gisement camera est precisement ce qui
        n'est toujours pas confirme sur ce robot (BUG-080). Ajouter un terme de
        signe inconnu a une mesure correcte la degraderait. On garde donc la
        mesure brute, bornee a +/-2 deg pres, et on logue yaw_rel a cote pour
        que l'operateur voie de ses yeux qu'il est bien petit.

        Pourquoi un echec de mesure ne fait pas echouer l'alignement : ne pas
        pouvoir verifier n'est pas pire que l'etat anterieur (qui ne verifiait
        jamais). On degrade en "non verifie" plutot que de transformer un run
        correct en echec -- c'est exactement le piege du garde-fou image
        retire le 2026-07-29. Un ABORT utilisateur, lui, continue de remonter.
        """
        self._stop()
        time.sleep(ALIGN_SETTLE_S)
        yaw_rel_now = angle_diff_deg(self._get_yaw_rel(), 0.0)
        try:
            _rng, bearing, _offaxis = self._measure()
        except DockAbort:
            if self._abort or rospy.is_shutdown():
                raise           # vrai ABORT utilisateur : ne pas l'avaler
            rospy.logwarn("[DOCK] verification : mesure impossible (balise "
                          "perdue ou dispersion excessive) — alignement laisse "
                          "NON VERIFIE plutot que declare en echec")
            return None
        rospy.loginfo(f"[DOCK] verification (robot arrete, {ALIGN_SETTLE_S}s de "
                      f"stabilisation) : gisement balise/chassis="
                      f"{bearing:+.1f} deg (yaw_rel residuel={yaw_rel_now:+.1f} deg, "
                      f"tolerance={TOL_BEARING_DEG} deg)")
        return bearing

    def _align_chassis_to_beacon(self, label, timeout_s=TURN_TIMEOUT_MAX_S,
                                 budget_left=None):
        """ALIGN complet et VERIFIE (2026-07-30). Enchaine (phase nacelle +
        phase chassis), puis MESURE le residu reel a l'arret, et recommence
        tant que le residu depasse la tolerance et que ca progresse.

        Renvoie `(succes, statut, residu_deg)`.

        Pourquoi une boucle exterieure plutot qu'une correction directe : pour
        corriger un residu de R degres il faudrait tourner le chassis de R,
        donc connaitre le signe reliant gisement camera et sens de rotation
        chassis -- c'est `_turn_by`, dont le signe est justement le suspect
        n°1 de BUG-077 (robot parti a l'oppose). Re-executer la paire
        nacelle+chassis fait le meme travail sans introduire aucune hypothese
        de signe nouvelle : la phase 1 absorbe le residu dans `yaw_rel` (elle
        ne fait que suivre visuellement la balise), la phase 2 le ramene a
        zero par le seul chemin dont le signe est confirme sur materiel
        (journal 2026-06-26). C'est une iteration de point fixe qui ne
        reutilise que du deja-valide.

        Cout dans le cas nominal : nul. Si la premiere passe est deja dans la
        tolerance -- le cas attendu quand tout va bien -- la boucle sort
        immediatement et le comportement est celui d'avant, plus une mesure.
        """
        prev_residual = None
        last_residual = None
        for attempt in range(1, ALIGN_VERIFY_PASSES + 1):
            if budget_left is not None and budget_left() <= 0:
                return False, "SEQUENCE_TIMEOUT", last_residual

            rospy.loginfo(f"[DOCK] {label} — passe {attempt}/{ALIGN_VERIFY_PASSES} "
                          f"(a) alignement nacelle sur la balise")
            gim_timeout = (GIMBAL_NULL_TIMEOUT_S if budget_left is None
                           else min(GIMBAL_NULL_TIMEOUT_S, budget_left()))
            if not self._align_gimbal_to_beacon(timeout_s=gim_timeout):
                return False, "GIMBAL_ALIGN_FAILED", last_residual
            ref_ground = self._gimbal_ref_ground

            rospy.loginfo(f"[DOCK] {label} — passe {attempt}/{ALIGN_VERIFY_PASSES} "
                          f"(b) alignement chassis (yaw_rel -> 0)")
            chassis_timeout = (timeout_s if budget_left is None
                               else min(timeout_s, budget_left()))
            if not self._align_chassis_yaw_rel(timeout_s=chassis_timeout):
                return False, "CHASSIS_ALIGN_FAILED", last_residual

            # Temoin de derive de la reference : si le cap ABSOLU de la nacelle
            # a bouge entre la fin de la phase 1 et la fin de la phase 2, alors
            # "le chassis est aligne sur la nacelle" ne veut plus dire "le
            # chassis est aligne sur la balise". Gratuit a verifier depuis que
            # H1 est confirmee (2026-07-29).
            if ref_ground is not None:
                drift = angle_diff_deg(self._get_yaw_ground(), ref_ground)
                if abs(drift) > ALIGN_REF_DRIFT_MAX_DEG:
                    rospy.logwarn(f"[DOCK] {label} : la reference nacelle a DERIVE de "
                                  f"{drift:+.1f} deg (cap absolu) pendant la rotation "
                                  f"chassis, au-dela des {ALIGN_REF_DRIFT_MAX_DEG} deg "
                                  f"toleres — l'alignement obtenu vise donc une "
                                  f"direction differente de celle mesuree en phase 1")
                else:
                    rospy.loginfo(f"[DOCK] {label} : reference nacelle stable "
                                  f"(derive cap absolu={drift:+.1f} deg)")

            residual = self._verify_alignment()
            if residual is None:
                return True, "ALIGN_DONE_UNVERIFIED", None
            last_residual = residual

            verdict = align_verify_verdict(residual, TOL_BEARING_DEG,
                                           prev_residual, ALIGN_VERIFY_MIN_GAIN_DEG,
                                           attempt, ALIGN_VERIFY_PASSES)
            if verdict == "ok":
                rospy.loginfo(f"[DOCK] {label} : ALIGNEMENT CONFIRME en {attempt} "
                              f"passe(s) — gisement residuel={residual:+.1f} deg "
                              f"(<= {TOL_BEARING_DEG} deg)")
                return True, "ALIGN_DONE", residual
            if verdict == "exhausted":
                rospy.logerr(f"[DOCK] {label} : {ALIGN_VERIFY_PASSES} passes epuisees, "
                             f"gisement residuel={residual:+.1f} deg toujours hors "
                             f"tolerance ({TOL_BEARING_DEG} deg)")
                return False, "ALIGN_NOT_CONVERGED", residual
            if verdict == "no_gain":
                rospy.logerr(f"[DOCK] {label} : passe {attempt} sans gain reel "
                             f"({prev_residual:+.1f} -> {residual:+.1f} deg, "
                             f"< {ALIGN_VERIFY_MIN_GAIN_DEG} deg) — une passe de plus "
                             f"n'y changerait rien, arret")
                return False, "ALIGN_NOT_CONVERGED", residual

            rospy.logwarn(f"[DOCK] {label} : gisement residuel={residual:+.1f} deg "
                          f"hors tolerance ({TOL_BEARING_DEG} deg) — passe "
                          f"supplementaire")
            prev_residual = residual

        return False, "ALIGN_NOT_CONVERGED", last_residual

    def _drive_by(self, dist_m):
        """Avance en ligne droite de dist_m (>0 uniquement — la marche arriere
        n'est pas utilisee par la manœuvre, et elle est aveugle cote capteurs).
        Asservi sur le deplacement mesure dans `/odom`. Nacelle asservie sur la
        balise pendant l'avance (`_track_beacon_gimbal_tick`, 2026-07-27)."""
        if dist_m < DRIVE_TOL_M:
            return
        if dist_m > ABSURD_SEGMENT_M:
            rospy.logerr(f"[DOCK] segment planifie aberrant ({dist_m:.2f} m > "
                         f"{ABSURD_SEGMENT_M} m) — mesure forcement fausse, abandon")
            raise DockAbort()
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        self._probe_gimbal_sign()
        x0, y0 = od[0], od[1]
        timeout = min(DRIVE_TIMEOUT_MAX_S,
                      max(3.0, 3.0 * dist_m / max(DRIVE_VX_M_S, 0.01)))
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._check_abort()
            od = self._get_odom()
            if od is None:
                break
            travelled = math.hypot(od[0] - x0, od[1] - y0)
            if travelled >= dist_m - DRIVE_TOL_M:
                break
            self._send_cmd(DRIVE_VX_M_S, 0.0)
            self._track_beacon_gimbal_tick()
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        self._send_gimbal(0.0)
        time.sleep(0.3)
        od = self._get_odom()
        if od is not None:
            got = math.hypot(od[0] - x0, od[1] - y0)
            rospy.loginfo(f"[DOCK] avance demandee={dist_m:.2f} m obtenue={got:.2f} m")

    # ---------------------------------------------------------
    # Mesure
    # ---------------------------------------------------------

    def _measure(self):
        """Agrege MEAS_SAMPLES poses (robot immobile, gimbal aligne chassis) et
        renvoie (range_m, bearing_deg, offaxis_deg).

        Repere 2D utilise dans tout ce module (== repere chassis puisque le
        gimbal est aligne) :
          * "avant"  = axe optique camera
          * "droite" = +x camera
          * bearing  = atan2(x, |z|)  -> positif = balise a DROITE
            (formule STRICTEMENT identique a celle du LOCK et de
            `_gimbal_servo_yaw` dans rm_cam_beacon.py, validee sur materiel le
            2026-06-26 — on ne re-derive pas une convention deja eprouvee)
          * range    = hypot(x, z)  -> distance vraie, pas |z|. La difference
            avec |z| (utilise par APPROACH comme "depth") est negligeable de
            face mais reelle en biais, cas justement vise par le docking.
          * offaxis  = BEACON_YAW_SIGN * yaw + BEACON_YAW_OFFSET_DEG
            -> 0 = on voit la balise de face (on est sur son axe frontal)
        """
        MEAS_MAX_ATTEMPTS = 3   # nb de lots re-tentes si dispersion excessive
        for attempt in range(1, MEAS_MAX_ATTEMPTS + 1):
            samples = []
            t0 = time.time()
            last_stamp = 0.0
            stale_skipped = 0
            while len(samples) < MEAS_SAMPLES:
                self._check_abort()
                if time.time() - t0 > MEAS_TIMEOUT_S:
                    rospy.logerr(f"[DOCK] mesure : seulement {len(samples)}/{MEAS_SAMPLES} "
                                 f"poses en {MEAS_TIMEOUT_S}s — balise perdue ?")
                    raise DockAbort()
                p = self._get_pose()
                if p is None or (time.time() - p[4]) > POSE_TIMEOUT_S:
                    time.sleep(0.05)
                    continue
                if p[4] == last_stamp:      # meme pose que le tour precedent (pas encore de nouveau message)
                    time.sleep(0.05)
                    continue
                last_stamp = p[4]
                # Pose republiee par carolus_astrobee.cpp (getFilteredPose,
                # lignes 560-598) : valeurs brutes identiques a la reception
                # precedente malgre un nouveau message/timestamp. Pas une
                # observation independante -> ne compte pas dans l'echantillon.
                if self._get_pose_repeat_count() > 0:
                    stale_skipped += 1
                    time.sleep(0.05)
                    continue
                x, _y, z, yaw_deg = p[0], p[1], p[2], p[3]
                if abs(z) < 0.05:
                    continue
                rng = math.hypot(x, z)
                bearing = math.degrees(math.atan2(x, abs(z)))
                offaxis = BEACON_YAW_SIGN * yaw_deg + BEACON_YAW_OFFSET_DEG
                offaxis = angle_diff_deg(offaxis, 0.0)   # ramene dans [-180, 180]
                samples.append((rng, bearing, offaxis))
                time.sleep(0.05)

            rng = median([s[0] for s in samples])
            bearing = median([s[1] for s in samples])
            offaxis = median([s[2] for s in samples])

            # Dispersion : un P4P qui saute d'une solution a l'autre se voit ici.
            spread = max(s[2] for s in samples) - min(s[2] for s in samples)
            rospy.loginfo(f"[DOCK] mesure (essai {attempt}/{MEAS_MAX_ATTEMPTS}) : "
                          f"range={rng:.3f} m bearing={bearing:+.1f} deg "
                          f"offaxis={offaxis:+.1f} deg (dispersion={spread:.1f} deg sur "
                          f"{len(samples)} poses, {stale_skipped} pose(s) repetee(s) ignoree(s))")
            if spread <= MEAS_MAX_SPREAD_DEG:
                return rng, bearing, offaxis
            rospy.logwarn(f"[DOCK] dispersion angulaire elevee ({spread:.1f} deg > "
                          f"{MEAS_MAX_SPREAD_DEG}) — mesure rejetee, "
                          f"{'nouvelle tentative' if attempt < MEAS_MAX_ATTEMPTS else 'abandon'}")

        rospy.logerr(f"[DOCK] mesure : dispersion excessive sur {MEAS_MAX_ATTEMPTS} "
                     f"tentatives — orientation balise non exploitable, abandon")
        raise DockAbort()

    # ---------------------------------------------------------
    # Planification de la manœuvre
    # ---------------------------------------------------------

    @staticmethod
    def plan_maneuver(rng, bearing_deg, offaxis_deg, dock_distance=DOCK_DISTANCE_M):
        """Calcule la manœuvre tourner-avancer-tourner amenant le robot au point
        de docking. Fonction PURE (aucun I/O, aucun etat) — donc testable hors
        robot, cf. le bloc __main__ en fin de fichier.

        Geometrie, dans le repere 2D decrit par `_measure` (robot a l'origine,
        regardant vers "avant") :

          B  = position de la balise            = rng * (sin(bearing), cos(bearing))
          Le vecteur B->robot fait, vu de la balise, un angle `offaxis` avec la
          normale a sa face. La normale sortante de la balise s'obtient donc en
          faisant tourner la direction B->robot de -offaxis.
          G  = point de docking                 = B + dock_distance * normale
          On veut finir EN G, tourne vers B.

        Renvoie (turn1_deg, drive_m, turn2_deg), angles positifs = vers la
        droite (meme convention que `bearing`)."""
        br = math.radians(bearing_deg)
        bx = rng * math.sin(br)      # composante droite
        bf = rng * math.cos(br)      # composante avant

        # Direction balise -> robot, exprimee comme un angle dans notre repere.
        phi = math.atan2(-bx, -bf)
        # Normale sortante de la face de la balise : on annule l'angle hors-axe.
        n = phi - math.radians(offaxis_deg)

        gx = bx + dock_distance * math.sin(n)
        gf = bf + dock_distance * math.cos(n)

        drive = math.hypot(gx, gf)

        # Cas degenere : le robot est DEJA sur le point de docking. `atan2(gx, gf)`
        # tournerait alors sur du bruit numerique et renverrait une direction
        # arbitraire (mesure : -90 deg pour un robot pourtant parfaitement place).
        # Il ne reste qu'a pivoter vers la balise. Detecte par le test cas 1.
        if drive < DRIVE_TOL_M:
            return 0.0, 0.0, bearing_deg

        turn1 = math.degrees(math.atan2(gx, gf))
        # Une fois en G et oriente selon turn1, l'angle a rattraper pour viser la
        # balise est la difference entre la direction G->B et turn1.
        head_gb = math.degrees(math.atan2(bx - gx, bf - gf))
        turn2 = angle_diff_deg(head_gb, turn1)
        return turn1, drive, turn2

    # ---------------------------------------------------------
    # Sequences
    # ---------------------------------------------------------

    def _run(self, fn):
        """Enveloppe commune : flags, arret propre, jamais d'exception qui
        laisserait le robot en mouvement."""
        self._busy = True
        self._abort = False
        try:
            fn()
        except DockAbort:
            self._status = "ABORTED"
            rospy.logwarn("[DOCK] sequence interrompue")
        except Exception as e:                      # noqa: BLE001 — filet de securite
            self._status = "ERROR"
            rospy.logerr(f"[DOCK] erreur inattendue : {e}")
        finally:
            self._stop()
            self._busy = False
            rospy.loginfo(f"[DOCK] etat final : {self._status}")

    def _align_only(self):
        """Commande ALIGN_ONLY (2026-07-28, demande utilisateur : pouvoir
        tester la rotation chassis isolement, sans jamais avancer).

        Reprend exactement les phases 1+2 de `_dock_sequence` (alignement
        nacelle puis chassis, memes fonctions, memes garde-fous) mais
        s'arrete la : AUCUN appel a `_measure()`/`_drive_by()`, donc aucune
        avance possible sous aucune condition.

        Pourquoi la phase 1 (nacelle) est incluse alors que la demande ne
        parle que du chassis : `yaw_rel` (utilise par la phase 2 pour
        orienter le chassis) est l'angle nacelle/chassis, PAS l'angle
        chassis/balise. Il n'a de sens comme reference pour aligner le
        chassis sur la balise QUE si la nacelle pointe deja sur la balise
        (phase 1). Sans ca, la phase 2 alignerait le chassis sur une
        direction arbitraire. Ce n'est donc pas une fonctionnalite ajoutee :
        c'est la meme dependance qui existe deja dans `_dock_sequence`,
        reutilisee telle quelle."""
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === ALIGN_ONLY : alignement nacelle puis chassis, SANS avance ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] ALIGN_ONLY : aucune pose fraiche sur /pose — "
                         "balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        # 2026-07-30 : passe par la boucle VERIFIEE plutot que d'appeler les
        # deux phases a la suite. Meme travail, plus une mesure de controle a
        # l'arret entre chaque passe -- c'est cette mesure qui manquait pour
        # que "SUCCES" veuille dire quelque chose (run du 2026-07-29 : SUCCES
        # annonce avec 26 deg d'ecart reel).
        ok, status, residual = self._align_chassis_to_beacon("ALIGN_ONLY",
                                                             timeout_s=TURN_TIMEOUT_MAX_S)
        self._status = status
        if not ok:
            rospy.logerr(f"[DOCK] ALIGN_ONLY : ECHEC ({status}"
                         f"{'' if residual is None else f', gisement residuel={residual:+.1f} deg'})"
                         f" — aucune avance effectuee")
            return

        if status == "ALIGN_DONE_UNVERIFIED":
            rospy.logwarn("[DOCK] ALIGN_ONLY termine mais NON VERIFIE : "
                          "yaw_rel a converge, la mesure de controle n'a pas pu "
                          "etre faite (balise perdue a l'arret ?)")
            return

        rospy.loginfo(f"[DOCK] ALIGN_ONLY termine : chassis aligne sur la balise, "
                      f"verifie a {residual:+.1f} deg, aucune avance effectuee")

    def _approach_only(self):
        """Commande APPROACH_ONLY (2026-07-28, demande utilisateur : avance
        seule, sans jamais tourner le chassis).

        Verifie D'ABORD que le chassis est deja aligne (meme tolerance que
        `_align_chassis_yaw_rel`, TURN_TOL_DEG) avant tout mouvement. Si ce
        n'est pas le cas : aucune avance, statut NOT_ALIGNED explicite,
        aucune tentative de correction automatique.

        Point signale (pas une rotation chassis, mais a mentionner par
        transparence) : `_drive_by()`, reutilisee ici telle quelle, fait
        legerement bouger la NACELLE (pas le chassis) pendant l'avance --
        `_track_beacon_gimbal_tick()` a chaque iteration pour garder la
        balise dans le champ pendant que le robot avance, plus un sondage
        ponctuel `_probe_gimbal_sign()` la toute premiere fois (mecanisme
        deja existant, ajoute le 2026-07-27, pas modifie ici). Aucune des
        deux ne fait tourner le chassis."""
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === APPROACH_ONLY : avance seule, SANS rotation chassis ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] APPROACH_ONLY : aucune pose fraiche sur /pose — "
                         "balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        yaw_rel = self._get_yaw_rel()
        if abs(yaw_rel) >= TURN_TOL_DEG:
            self._status = "NOT_ALIGNED"
            rospy.logerr(f"[DOCK] APPROACH_ONLY : robot non aligne "
                         f"(yaw_rel={yaw_rel:+.1f} deg, tolerance="
                         f"{TURN_TOL_DEG:.1f} deg) — pas d'avance. "
                         f"Lancer ALIGN_ONLY d'abord.")
            return

        rng, bearing, off = self._measure()
        rospy.loginfo(f"[DOCK] APPROACH_ONLY : deja aligne (yaw_rel={yaw_rel:+.1f} deg), "
                      f"avance de {max(0.0, rng - DOCK_DISTANCE_M):.2f} m "
                      f"(range mesure={rng:.2f} m, cible={DOCK_DISTANCE_M:.2f} m)")
        self._drive_by(max(0.0, rng - DOCK_DISTANCE_M))
        rng2, bearing2, off2 = self._measure()
        rospy.loginfo(f"[DOCK] APPROACH_ONLY termine : range={rng2:.3f} m "
                      f"bearing={bearing2:+.1f} deg (offaxis={off2:+.1f} deg "
                      f"non corrige)")
        self._status = "APPROACH_DONE"

    def _dock_sequence(self):
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === debut docking ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] aucune pose fraiche sur /pose — balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        if SIMPLE_APPROACH_ONLY or not BEACON_YAW_VALIDATED:
            # Mode simple (voir SIMPLE_APPROACH_ONLY en tete de fichier) ou repli
            # documente (convention d'orientation non validee, on ne peut pas
            # viser l'axe frontal) : pas d'alignement sur l'axe frontal (offaxis
            # ignore), pas de boucle de convergence.
            #
            # Pipeline en 3 phases (2026-07-27, demande utilisateur suite au
            # comportement observe avec un simple _turn_by(bearing) direct) :
            #   1. Aligner la NACELLE sur la balise (asservissement image, deja
            #      utilise pendant l'avance -- _track_beacon_gimbal_tick).
            #   2. Aligner le CHASSIS en annulant yaw_rel -- PAS via _turn_by
            #      (qui suppose un lien odom-yaw/bearing camera jamais
            #      confirme, suspect n°1 du "robot part a l'oppose"). On
            #      reutilise a la place un fait DEJA CONFIRME sur ce robot et
            #      ce chemin de commande (/carolus/cmd_vel) : l'etat ALIGN de
            #      rm_cam_beacon.py (journal 2026-06-26, K_BODY_YAW) --
            #      wz = +K*yaw_rel fait DECROITRE yaw_rel (mesure : 106deg ->
            #      1.6deg avec wz=+10). Une fois yaw_rel~0 et la nacelle sur la
            #      balise (phase 1), le chassis est de facto pointe dessus.
            #   3. Avancer tout droit (chassis deja oriente) jusqu'a
            #      DOCK_DISTANCE_M, nacelle continue de suivre (_drive_by).
            reason = ("mode simple demande" if SIMPLE_APPROACH_ONLY
                      else "convention orientation balise NON validee")
            rospy.loginfo(f"[DOCK] {reason} -> pipeline 3 phases : "
                          f"aligner nacelle, aligner chassis (yaw_rel), avancer")

            seq_t0 = time.time()

            def _seq_timeout_left():
                return SEQUENCE_TIMEOUT_S - (time.time() - seq_t0)

            # Phases 1+2 : boucle VERIFIEE (2026-07-30). Les deux garde-fous
            # historiques restent en vigueur a l'identique -- un echec de la
            # nacelle (2026-07-27, cause directe d'une collision) comme un
            # echec du chassis (2026-07-28) interdisent toujours l'avance. Ce
            # qui change : le succes lui-meme est desormais mesure, et non plus
            # deduit de yaw_rel seul. Un residu hors tolerance interdit
            # egalement l'avance, alors qu'avant il passait inapercu (le
            # `_measure()` de la phase 3 mesurait deja ce gisement... et le
            # jetait, cf. `_off` ignore ci-dessous).
            rospy.loginfo("[DOCK] phases 1-2/3 : alignement nacelle + chassis (verifie)")
            if _seq_timeout_left() <= 0:
                self._status = "SEQUENCE_TIMEOUT"
                rospy.logerr("[DOCK] abandon : timeout global atteint avant meme la phase 1")
                return
            ok, status, residual = self._align_chassis_to_beacon(
                "docking phases 1-2",
                timeout_s=TURN_TIMEOUT_MAX_S,
                budget_left=_seq_timeout_left)
            if not ok:
                self._status = status
                rospy.logerr(f"[DOCK] abandon : alignement non atteint ({status}"
                             f"{'' if residual is None else f', residu={residual:+.1f} deg'})"
                             f" — pas d'avance (securite)")
                return
            if status == "ALIGN_DONE_UNVERIFIED":
                self._status = "CHASSIS_ALIGN_FAILED"
                rospy.logerr("[DOCK] abandon : alignement non VERIFIABLE (mesure "
                             "de controle impossible) — pas d'avance. Contrairement "
                             "a ALIGN_ONLY, une avance engage le robot vers un "
                             "obstacle : on n'avance pas sur un alignement non "
                             "confirme.")
                return

            rospy.loginfo("[DOCK] phase 3/3 : avance")
            if _seq_timeout_left() <= 0:
                self._status = "SEQUENCE_TIMEOUT"
                rospy.logerr("[DOCK] abandon : timeout global atteint avant la phase 3")
                self._stop()
                return
            rng, bearing, _off = self._measure()
            self._drive_by(max(0.0, rng - DOCK_DISTANCE_M))
            rng2, bearing2, off2 = self._measure()
            rospy.loginfo(f"[DOCK] approche simple terminee : range={rng2:.3f} m "
                          f"bearing={bearing2:+.1f} deg (offaxis={off2:+.1f} deg "
                          f"non corrige)")
            self._status = "RANGE_ONLY"
            return

        self._null_gimbal()

        prev_offaxis = None
        for i in range(1, MAX_ITERATIONS + 1):
            self._check_abort()
            rospy.loginfo(f"[DOCK] --- iteration {i}/{MAX_ITERATIONS} ---")
            self._null_gimbal()
            rng, bearing, offaxis = self._measure()

            if (abs(rng - DOCK_DISTANCE_M) < TOL_RANGE_M
                    and abs(offaxis) < TOL_OFFAXIS_DEG
                    and abs(bearing) < TOL_BEARING_DEG):
                rospy.loginfo(f"[DOCK] ✅ docke : range={rng:.3f} m "
                              f"offaxis={offaxis:+.1f} deg bearing={bearing:+.1f} deg "
                              f"(iteration {i})")
                self._status = "DOCKED"
                return

            # Garde-fou anti-divergence : si l'angle hors-axe ne s'ameliore pas,
            # c'est le symptome typique d'un BEACON_YAW_SIGN faux -> on arrete
            # plutot que de tourner autour de la balise indefiniment.
            if prev_offaxis is not None:
                gain = abs(prev_offaxis) - abs(offaxis)
                if gain < MIN_PROGRESS_DEG:
                    rospy.logerr(f"[DOCK] pas de progres sur l'angle hors-axe "
                                 f"({abs(prev_offaxis):.1f} -> {abs(offaxis):.1f} deg). "
                                 f"Cause la plus probable : BEACON_YAW_SIGN inverse. "
                                 f"Refaire le mode CALIBRATE. Arret.")
                    self._status = "NO_PROGRESS"
                    return
            prev_offaxis = offaxis

            turn1, drive, turn2 = self.plan_maneuver(rng, bearing, offaxis)
            rospy.loginfo(f"[DOCK] manœuvre : tourner {turn1:+.1f} deg, "
                          f"avancer {drive:.2f} m, tourner {turn2:+.1f} deg")

            self._turn_by(turn1)
            if drive > MAX_SEGMENT_M:
                # Detour long (cas typique : tres hors-axe et loin). On n'avance
                # pas 3 m en aveugle sur une mesure a 2.5 Hz : on tronque, et on
                # laisse l'iteration suivante re-planifier depuis une mesure
                # fraiche. `turn2` est volontairement SAUTE — il n'a de sens
                # qu'arrive au point de docking, pas a mi-chemin.
                rospy.loginfo(f"[DOCK] segment tronque a {MAX_SEGMENT_M} m "
                              f"(plan={drive:.2f} m) — re-mesure a l'iteration suivante")
                self._drive_by(MAX_SEGMENT_M)
                # Manœuvre incomplete : le controle anti-divergence ci-dessus
                # compare deux etats de FIN de manœuvre. Le neutraliser pour le
                # tour suivant, sinon un detour tronque (progres angulaire
                # normalement faible) serait pris a tort pour un signe inverse.
                prev_offaxis = None
                continue
            self._drive_by(drive)
            self._turn_by(turn2)

        rospy.logwarn(f"[DOCK] {MAX_ITERATIONS} iterations sans converger — arret")
        self._status = "NOT_CONVERGED"

    def _calibrate_step1(self):
        """Etablit la convention d'orientation balise, seule inconnue que le
        robot ne peut pas mesurer seul.

        Protocole en 2 clics independants (repond a la question 6 de
        research-log/07-perplexity/17-docking-position-fixe-balise.md : valider
        une orientation monoculaire sans banc de test ni motion capture).
        Volontairement PAS de minuteur bloquant entre les deux etapes (version
        initiale : un delai fixe de 20s embarque dans la meme sequence,
        illisible si les logs T5 defilent — corrige le 2026-07-27) : chaque
        etape attend un ordre explicite (bouton GUI dedie), a executer au
        rythme de l'utilisateur, pas dans une fenetre a rater.

          1. (CALIBRATE) Placer le robot EN FACE de la balise, sur son axe
             frontal, a ~1 m, PUIS cliquer CALIBRATE. -> on lit yaw_face.
          2. (CAL STEP 2) Deplacer le robot d'environ 30 deg SUR LA DROITE de
             la balise (balise immobile), PUIS cliquer CAL STEP 2 une fois en
             place. -> on lit yaw_right.
        De (1) on tire l'offset ; du SENS de variation entre (1) et (2) on tire
        le signe.
        """
        self._status = "CALIBRATING"
        self._take_control()
        self._stop()

        rospy.loginfo("[DOCK][CAL] Etape 1/2 — mesure en cours (3 s, ne pas bouger "
                      "le robot)...")
        for _ in range(3):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        p = self._get_pose()
        if p is None or not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] pas de pose fraiche — abandon. Verifier que "
                         "la balise est bien visible, relancer CALIBRATE.")
            self._status = "CAL_FAILED"
            return
        self._cal_yaw_face = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] etape 1 OK : yaw brut de face = "
                      f"{self._cal_yaw_face:+.1f} deg")
        rospy.loginfo("[DOCK][CAL] -> deplace maintenant le robot d'environ 30 deg "
                      "vers la DROITE de la balise (balise immobile), a ton rythme, "
                      "puis clique CAL STEP 2. Statut GUI : CAL_STEP1_DONE.")
        self._status = "CAL_STEP1_DONE"

    def _calibrate_step2(self):
        if self._cal_yaw_face is None:
            rospy.logerr("[DOCK][CAL] etape 1 pas encore faite — clique d'abord "
                         "CALIBRATE.")
            self._status = "CAL_FAILED"
            return
        self._status = "CALIBRATING"
        self._take_control()
        self._stop()

        rospy.loginfo("[DOCK][CAL] Etape 2/2 — mesure en cours (3 s, ne pas bouger "
                      "le robot)...")
        for _ in range(3):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        if not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] pas de pose fraiche — abandon")
            self._status = "CAL_FAILED"
            return
        yaw_right = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] etape 2 OK : yaw brut de biais = "
                      f"{yaw_right:+.1f} deg")

        delta = angle_diff_deg(yaw_right, self._cal_yaw_face)
        if abs(delta) < 5.0:
            rospy.logerr(f"[DOCK][CAL] variation trop faible ({delta:+.1f} deg) : "
                         f"soit le robot n'a pas ete deplace, soit l'orientation "
                         f"renvoyee par Carolus n'est pas exploitable a cette "
                         f"distance. Calibration NON concluante — clique CALIBRATE "
                         f"pour recommencer depuis l'etape 1.")
            self._status = "CAL_INCONCLUSIVE"
            self._cal_yaw_face = None
            return

        sign = +1.0 if delta > 0 else -1.0
        offset = -sign * self._cal_yaw_face
        rospy.loginfo("[DOCK][CAL] ================ RESULTAT ================")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_SIGN       = {sign:+.1f}")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_OFFSET_DEG = {offset:+.1f}")
        rospy.loginfo("[DOCK][CAL] BEACON_YAW_VALIDATED  = True")
        rospy.loginfo("[DOCK][CAL] -> reporter ces 3 valeurs en tete de "
                      "beacon_docking.py, puis relancer le nœud.")
        rospy.loginfo("[DOCK][CAL] ===========================================")
        self._status = "CAL_DONE"
        self._cal_yaw_face = None


# =========================================================
# Auto-test de la geometrie (hors robot)
# =========================================================

def _self_test():
    """Verifie `plan_maneuver` sur des cas dont la solution est evidente a la
    main. Executable sans ROS ni robot : `python3 beacon_docking.py --selftest`."""
    ok = True

    def check(name, got, expected, tol):
        """Comparaison ANGULAIRE (gere le repliement +/-180)."""
        nonlocal ok
        good = abs(angle_diff_deg(got, expected)) < tol
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected:+.2f}, obtenu {got:+.2f}")
        ok = ok and good

    def check_val(name, got, expected, tol):
        """Comparaison SCALAIRE (distances)."""
        nonlocal ok
        good = abs(got - expected) < tol
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected:.2f}, obtenu {got:.2f}")
        ok = ok and good

    print("Cas 1 — deja parfaitement docke (de face, a la bonne distance) :")
    t1, d, t2 = BeaconDocking.plan_maneuver(DOCK_DISTANCE_M, 0.0, 0.0)
    check("turn1", t1, 0.0, 1e-6)
    print(f"  {'OK ' if abs(d) < 1e-6 else 'FAIL'} drive: attendu 0.00, obtenu {d:.2f}")
    ok = ok and abs(d) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    print("Cas 2 — de face, trop loin (2 m au lieu de 0.70) :")
    t1, d, t2 = BeaconDocking.plan_maneuver(2.0, 0.0, 0.0)
    check("turn1", t1, 0.0, 1e-6)
    print(f"  {'OK ' if abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6 else 'FAIL'} "
          f"drive: attendu {2.0 - DOCK_DISTANCE_M:.2f}, obtenu {d:.2f}")
    ok = ok and abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    print("Cas 3 — balise vue de face mais decalee a droite (bearing=+20) :")
    # On la voit de face (offaxis=0) => on est deja sur son axe frontal ;
    # il suffit donc de pivoter vers elle et d'ajuster la distance.
    t1, d, t2 = BeaconDocking.plan_maneuver(2.0, 20.0, 0.0)
    check("turn1", t1, 20.0, 1e-6)
    print(f"  {'OK ' if abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6 else 'FAIL'} "
          f"drive: attendu {2.0 - DOCK_DISTANCE_M:.2f}, obtenu {d:.2f}")
    ok = ok and abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    # Cas de biais : on ne connait pas la reponse "a la main", on verifie donc
    # les 3 PROPRIETES que la manœuvre doit garantir par construction, en
    # rejouant la geometrie a partir de son propre resultat.
    for rng_in, bearing_in, offaxis_in in [(2.0, 0.0, 40.0), (1.5, -25.0, -35.0),
                                           (3.0, 15.0, 70.0), (1.2, 40.0, -10.0)]:
        print(f"Cas de biais — range={rng_in} bearing={bearing_in:+.0f} "
              f"offaxis={offaxis_in:+.0f} :")
        t1, d, t2 = BeaconDocking.plan_maneuver(rng_in, bearing_in, offaxis_in)
        print(f"  info  turn1={t1:+.1f} deg  drive={d:.2f} m  turn2={t2:+.1f} deg")

        br = math.radians(bearing_in)
        bx, bf = rng_in * math.sin(br), rng_in * math.cos(br)          # balise
        gx, gf = d * math.sin(math.radians(t1)), d * math.cos(math.radians(t1))  # arrivee

        # (1) le robot finit a DOCK_DISTANCE_M de la balise
        final_range = math.hypot(bx - gx, bf - gf)
        check_val("distance finale", final_range, DOCK_DISTANCE_M, 1e-6)

        # (2) le robot finit tourne VERS la balise (cap final == direction G->B)
        heading_final = t1 + t2
        dir_gb = math.degrees(math.atan2(bx - gx, bf - gf))
        check("cap final vers la balise", heading_final, dir_gb, 1e-6)

        # (3) le robot finit SUR L'AXE FRONTAL de la balise : la direction
        #     balise->robot doit coincider avec la normale sortante de sa face.
        phi_in = math.atan2(-bx, -bf)
        normal = math.degrees(phi_in - math.radians(offaxis_in))
        dir_bg = math.degrees(math.atan2(gx - bx, gf - bf))
        check("robot sur l'axe frontal", dir_bg, normal, 1e-6)

    # 2026-07-28 : logique de decision de _align_chassis_yaw_rel, extraite en
    # fonctions pures pour etre testable ici sans ROS ni robot.

    def check_bool(name, got, expected):
        nonlocal ok
        good = got == expected
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected}, obtenu {got}")
        ok = ok and good

    print("Chassis — succes d'alignement (3 lectures consecutives requises) :")
    c = 0
    c = chassis_align_tick(1.0, 2.0, c)   # dans tolerance -> 1
    check_val("compteur apres 1ere lecture OK", c, 1, 1e-9)
    c = chassis_align_tick(15.0, 2.0, c)  # hors tolerance -> reset a 0
    check_val("compteur reinitialise si une lecture sort de tolerance", c, 0, 1e-9)
    c = chassis_align_tick(0.5, 2.0, c)
    c = chassis_align_tick(0.5, 2.0, c)
    c = chassis_align_tick(0.5, 2.0, c)
    check_val("compteur apres 3 lectures OK consecutives", c, 3, 1e-9)

    print("Chassis — absence de progres / divergence :")
    check_bool("pas de progres (erreur quasi inchangee)",
               chassis_no_progress(10.0, 9.5, 1.0), True)
    check_bool("progres suffisant (erreur nettement reduite)",
               chassis_no_progress(10.0, 5.0, 1.0), False)
    check_bool("divergence (erreur qui augmente)",
               chassis_no_progress(10.0, 15.0, 1.0), True)

    print("Chassis — detection de blocage physique :")
    check_bool("bloque (commandes envoyees, yaw_rel immobile)",
               chassis_is_blocked(50.0, 49.5, 1.0, commands_sent=5), True)
    check_bool("pas bloque (yaw_rel a suffisamment bouge)",
               chassis_is_blocked(50.0, 40.0, 1.0, commands_sent=5), False)
    check_bool("pas de verdict avant la 1ere commande (evite un faux positif au demarrage)",
               chassis_is_blocked(50.0, 50.0, 1.0, commands_sent=0), False)
    # 2026-07-30 : non-regression du passage a angle_diff_deg. Avec l'ancienne
    # soustraction directe, ce cas (chassis STRICTEMENT immobile a cheval sur
    # +/-180) donnait |diff|=358 deg et repondait donc "pas bloque".
    check_bool("bloque meme a cheval sur +/-180 deg (ex-faux negatif)",
               chassis_is_blocked(179.7, -179.8, 1.0, commands_sent=5), True)

    print("\nNacelle — confirmation multi-lectures (phase 1) :")
    check_bool("1re lecture sous tolerance ne suffit plus",
               gimbal_confirm_tick(1.0, 3.0, 0) >= GIMBAL_CONFIRM_OK, False)
    check_val("compteur incremente sous tolerance",
              gimbal_confirm_tick(1.0, 3.0, 2), 3.0, 0.001)
    check_val("compteur remis a zero hors tolerance",
              gimbal_confirm_tick(5.0, 3.0, 2), 0.0, 0.001)

    print("\nVerification terminale de l'alignement :")
    check_bool("residu dans la tolerance -> termine",
               align_verify_verdict(3.0, 6.0, None, 2.0, 1, 3) == "ok", True)
    check_bool("residu hors tolerance a la 1re passe -> on retente",
               align_verify_verdict(20.0, 6.0, None, 2.0, 1, 3) == "retry", True)
    check_bool("progres franc entre deux passes -> on retente",
               align_verify_verdict(9.0, 6.0, 20.0, 2.0, 2, 3) == "retry", True)
    check_bool("passe sans gain reel -> on arrete (ne pas user la mecanique)",
               align_verify_verdict(19.0, 6.0, 20.0, 2.0, 2, 3) == "no_gain", True)
    check_bool("budget de passes epuise -> echec",
               align_verify_verdict(20.0, 6.0, 40.0, 2.0, 3, 3) == "exhausted", True)
    # Le cas materiel du 2026-07-29 (run 1) : yaw_rel converge a +0.6 deg mais
    # gisement reel a -26.3 deg. L'ancien code annoncait SUCCES ; le nouveau
    # doit refuser de conclure et demander une passe de plus.
    check_bool("cas reel 2026-07-29 (residu -26.3 deg) n'est PAS un succes",
               align_verify_verdict(-26.3, TOL_BEARING_DEG, None,
                                    ALIGN_VERIFY_MIN_GAIN_DEG, 1,
                                    ALIGN_VERIFY_PASSES) == "retry", True)

    print("\nRESULTAT :", "TOUS LES CAS PASSENT" if ok else "ECHEC")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_self_test())
    if not _ROS_AVAILABLE:
        print(f"ROS indisponible dans cet environnement ({_ROS_IMPORT_ERROR}).\n"
              f"Sourcer ROS (source /opt/ros/noetic/setup.bash) pour lancer le nœud,\n"
              f"ou utiliser --selftest pour verifier la geometrie hors robot.")
        sys.exit(1)
    rospy.init_node("beacon_docking")
    BeaconDocking()
    rospy.spin()
