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

Puis, depuis un autre terminal :

    # calibration de la convention d'orientation balise (a faire une fois)
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'CALIBRATE'"

    # docking
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'START'"

    # arret d'urgence
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'ABORT'"

Etat publie en continu sur `/carolus/dock_status` (String), et logue en
`[DOCK] ...` (meme convention de prefixe que `[APPROACH]`/`[BEACON]`, donc
parsable par `carolus_launcher.py` si on veut l'y brancher plus tard).

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
DOCK_DISTANCE_M = 0.70    # distance finale robot<->balise (identique a
                          # STOP_DISTANCE_M de rm_cam_beacon.py : meme point
                          # d'arret, on ajoute juste l'alignement angulaire)

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

# ── Convention d'orientation balise (A ETABLIR PAR LE MODE CALIBRATE) ───────
# psi = BEACON_YAW_SIGN * p.yaw + BEACON_YAW_OFFSET_DEG
# psi est defini comme l'ANGLE HORS-AXE : 0 = le robot est pile sur l'axe
# frontal de la balise (il la voit de face), != 0 = il la voit de biais.
BEACON_YAW_SIGN       = +1.0
BEACON_YAW_OFFSET_DEG = 0.0
# Passer a True UNIQUEMENT apres avoir valide les deux valeurs ci-dessus par
# le mode CALIBRATE sur le materiel. False = refus de docker (repli sur simple
# maintien de distance), cf. entete.
BEACON_YAW_VALIDATED  = False

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
        self._odom = None          # (x, y, yaw_deg)
        self._odom_lock = threading.Lock()

        # --- etat manœuvre ---
        self._abort = False
        self._busy = False
        self._turn_sign = None     # +1/-1, determine par _probe_turn_sign()
        self._gimbal_sign = None   # +1/-1, determine par _probe_gimbal_sign()
        self._status = "IDLE"

        # --- ROS ---
        self.pub_cmd = rospy.Publisher("/carolus/cmd_vel", Twist, queue_size=1)
        self.pub_gim = rospy.Publisher("/carolus/gimbal_vel", Twist, queue_size=1)
        self.pub_mode = rospy.Publisher("/carolus/mode", String, queue_size=1)
        self.pub_lock = rospy.Publisher("/carolus/gimbal_lock", String, queue_size=1)
        self.pub_status = rospy.Publisher("/carolus/dock_status", String, queue_size=1)

        rospy.Subscriber("/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/carolus/gimbal_yaw_rel", Float32, self._yaw_rel_cb)
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/carolus/dock", String, self._cmd_cb)

        rospy.Timer(rospy.Duration(0.5), self._status_tick)

        rospy.loginfo("[DOCK] pret — commandes sur /carolus/dock : START / CALIBRATE / ABORT")
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
        with self._pose_lock:
            self._pose = (p.x, p.y, p.z, yaw_deg, time.time())

    def _yaw_rel_cb(self, msg):
        with self._yaw_rel_lock:
            self._yaw_rel = float(msg.data)

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
        elif cmd == "CALIBRATE":
            threading.Thread(target=self._run, args=(self._calibrate_sequence,),
                             daemon=True).start()

    def _status_tick(self, _event):
        self.pub_status.publish(String(data=self._status))

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
        """Idem pour la nacelle, sur `/carolus/gimbal_yaw_rel`."""
        if self._gimbal_sign is not None:
            return
        y0 = self._get_yaw_rel()
        t0 = time.time()
        while time.time() - t0 < PROBE_DURATION_S:
            self._check_abort()
            self._send_gimbal(PROBE_TURN_DEG_S)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._send_gimbal(0.0)
        time.sleep(0.4)
        delta = angle_diff_deg(self._get_yaw_rel(), y0)
        if abs(delta) < PROBE_MIN_DELTA_DEG:
            rospy.logwarn(f"[DOCK] sondage gimbal : nacelle immobile ({delta:.1f} deg) "
                          f"— alignement gimbal saute, mesure faite telle quelle")
            self._gimbal_sign = 0.0   # 0 = gimbal inutilisable, on s'en passe
            return
        self._gimbal_sign = 1.0 if delta > 0 else -1.0
        rospy.loginfo(f"[DOCK] sondage gimbal : cmd>0 -> yaw_rel {delta:+.1f} deg "
                      f"-> gimbal_sign={self._gimbal_sign:+.0f}")

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
            got = angle_diff_deg(od[2], yaw0) * self._turn_sign
            rospy.loginfo(f"[DOCK] rotation demandee={delta_deg:+.1f} deg obtenue={got:+.1f} deg")

    def _drive_by(self, dist_m):
        """Avance en ligne droite de dist_m (>0 uniquement — la marche arriere
        n'est pas utilisee par la manœuvre, et elle est aveugle cote capteurs).
        Asservi sur le deplacement mesure dans `/odom`."""
        if dist_m < DRIVE_TOL_M:
            return
        if dist_m > ABSURD_SEGMENT_M:
            rospy.logerr(f"[DOCK] segment planifie aberrant ({dist_m:.2f} m > "
                         f"{ABSURD_SEGMENT_M} m) — mesure forcement fausse, abandon")
            raise DockAbort()
        od = self._get_odom()
        if od is None:
            raise DockAbort()
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
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
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
        samples = []
        t0 = time.time()
        last_stamp = 0.0
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
            if p[4] == last_stamp:      # meme pose que le tour precedent
                time.sleep(0.05)
                continue
            last_stamp = p[4]
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
        rospy.loginfo(f"[DOCK] mesure : range={rng:.3f} m bearing={bearing:+.1f} deg "
                      f"offaxis={offaxis:+.1f} deg (dispersion={spread:.1f} deg sur "
                      f"{len(samples)} poses)")
        if spread > MEAS_MAX_SPREAD_DEG:
            rospy.logwarn(f"[DOCK] dispersion angulaire elevee ({spread:.1f} deg > "
                          f"{MEAS_MAX_SPREAD_DEG}) — orientation balise peu fiable "
                          f"a cette distance/cet angle")
        return rng, bearing, offaxis

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

    def _dock_sequence(self):
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === debut docking ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] aucune pose fraiche sur /pose — balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        self._null_gimbal()

        if not BEACON_YAW_VALIDATED:
            # Repli documente : sans convention d'orientation validee, on ne
            # peut pas viser l'axe frontal. On se contente d'amener la distance
            # a la consigne, ce qui equivaut a l'APPROACH existant — mais on le
            # DIT, on ne fait pas semblant d'avoir dock.
            rospy.logwarn("[DOCK] convention orientation balise NON validee -> "
                          "repli : maintien de distance seul (pas d'alignement axe)")
            rng, bearing, _off = self._measure()
            self._turn_by(bearing)
            self._drive_by(max(0.0, rng - DOCK_DISTANCE_M))
            rng2, bearing2, off2 = self._measure()
            rospy.loginfo(f"[DOCK] repli termine : range={rng2:.3f} m "
                          f"bearing={bearing2:+.1f} deg (offaxis={off2:+.1f} deg "
                          f"NON corrige, convention non validee)")
            self._status = "RANGE_ONLY"
            return

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

    def _calibrate_sequence(self):
        """Etablit la convention d'orientation balise, seule inconnue que le
        robot ne peut pas mesurer seul.

        Protocole (repond a la question 6 de research-log/07-perplexity/
        17-docking-position-fixe-balise.md : valider une orientation monoculaire
        sans banc de test ni motion capture) :
          1. Placer le robot EN FACE de la balise, sur son axe frontal, a ~1 m.
             -> on lit yaw_face. La convention doit donner offaxis = 0 ici.
          2. Deplacer le robot d'environ 30 deg SUR LA DROITE de la balise
             (le robot voit maintenant la balise de biais), sans changer la
             balise de place. -> on lit yaw_right.
        De (1) on tire l'offset ; du SENS de variation entre (1) et (2) on tire
        le signe. Deux mesures suffisent car on ne cherche que ces deux
        parametres.
        """
        self._status = "CALIBRATING"
        self._take_control()
        self._stop()

        rospy.loginfo("[DOCK][CAL] Etape 1/2 — placer le robot EN FACE de la balise "
                      "(sur son axe frontal, ~1 m), puis relancer CALIBRATE une fois "
                      "en place. Mesure dans 10 s...")
        for _ in range(10):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        p = self._get_pose()
        if p is None or not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] pas de pose fraiche — abandon")
            self._status = "CAL_FAILED"
            return
        yaw_face = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] etape 1 : yaw brut de face = {yaw_face:+.1f} deg")

        rospy.loginfo("[DOCK][CAL] Etape 2/2 — deplacer MAINTENANT le robot d'environ "
                      "30 deg vers la DROITE de la balise (balise immobile). "
                      "Mesure dans 20 s...")
        for _ in range(20):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        if not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] pas de pose fraiche — abandon")
            self._status = "CAL_FAILED"
            return
        yaw_right = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] etape 2 : yaw brut de biais = {yaw_right:+.1f} deg")

        delta = angle_diff_deg(yaw_right, yaw_face)
        if abs(delta) < 5.0:
            rospy.logerr(f"[DOCK][CAL] variation trop faible ({delta:+.1f} deg) : "
                         f"soit le robot n'a pas ete deplace, soit l'orientation "
                         f"renvoyee par Carolus n'est pas exploitable a cette "
                         f"distance. Calibration NON concluante.")
            self._status = "CAL_INCONCLUSIVE"
            return

        sign = +1.0 if delta > 0 else -1.0
        offset = -sign * yaw_face
        rospy.loginfo("[DOCK][CAL] ================ RESULTAT ================")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_SIGN       = {sign:+.1f}")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_OFFSET_DEG = {offset:+.1f}")
        rospy.loginfo("[DOCK][CAL] BEACON_YAW_VALIDATED  = True")
        rospy.loginfo("[DOCK][CAL] -> reporter ces 3 valeurs en tete de "
                      "beacon_docking.py, puis relancer le nœud.")
        rospy.loginfo("[DOCK][CAL] ===========================================")
        self._status = "CAL_DONE"


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
