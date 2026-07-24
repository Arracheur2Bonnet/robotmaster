#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Caméra + suivi de balise — connexion SDK unique.
Remplace rm_cam_ros.py + follow_beacon.py (deux connexions → conflit drive_speed).
"""

import os
import sys
import threading
import time
import math
import queue
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(__file__))
from map_collision import MapCollision

import rospy
import tf2_ros
import tf.transformations as tft
from robomaster import robot
from robomaster.battery import BatterySubject as _BatBase
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge


# =========================================================
# CONFIG
# =========================================================

# Cadence de publication /camera/color/image_raw. 30 = cadence native du capteur S1
# (2026-07-22 : monte de 20 -> 30 pour donner a Carolus des frames plus fraiches).
# ATTENTION : le vrai goulot de Carolus (~2.5 Hz) est le TRANSPORT RESEAU des images
# (Pi -> PC labo), pas cette cadence. Si la latence des poses empire apres ce bump
# (poses plus vieilles, /pose qui trainerait), REVENIR a 20 (voire 15) : plus de frames
# = plus de bande passante sur le lien deja sature. Le vrai gain viendra de F0.C
# (Carolus deplace sur le Pi), pas d'ici.
TARGET_FPS       = 30
STOP_DISTANCE_M  = 0.70
POSE_TIMEOUT_S   = 1.5

MAX_VX  = 0.20
MAX_WZ  = 10.0
MIN_VX  = 0.06
K_VX    = 0.6

# ── SEARCH : sweep gimbal (robot immobile) + avancement chassis si echec ──────
# Le gimbal est stabilise inertiellement (yaw_ground constant si le chassis tourne).
# On balaye le gimbal en lisant yaw_rel (angle gimbal vs chassis) ; le chassis ne
# bouge PAS pendant le sweep. Si un balayage complet de la plage echoue -> recenter
# (gimbal aligne sur le corps) + avancer 1 m en ligne droite + recommencer.
SEARCH_GIMBAL_YAW_S = 20.0    # consigne drive_speed gimbal (~20 deg/s reel, sens direct)
SEARCH_YAW_LIMIT    = 200.0   # amplitude max du sweep en yaw_rel (deg, < limite meca 250)
SEARCH_STEP_M       = 1.0     # avancement chassis si sweep complet sans detection (m)
SEARCH_STEP_VX      = 0.15    # vitesse d'avancement du pas (m/s)
SEARCH_MAX_STEPS    = 3       # nb max d'avancements avant arret (protege le cable Ethernet)
SEARCH_GRACE_S      = 2.0     # pause initiale avant de commencer a balayer
RECENTER_TIMEOUT_S  = 6.0     # timeout wait_for_completed du recenter

# ── Motif en eventail : axe epuise (SEARCH_MAX_STEPS atteint) -> tourner et recommencer ──
# Recommandations Perplexity 08 (research-log/07-perplexity/08-strategie-recherche-couverture.md) :
# increment 60deg, 6 axes (2 passes de 3), borne globale 90s ou ~13.5m cumules.
SEARCH_FAN_AXIS_COUNT    = 6      # nb total d'axes du motif en eventail (2 passes de 3)
SEARCH_FAN_AXIS_INC_DEG  = 60.0   # increment de rotation entre deux axes (deg)
SEARCH_FAN_ROT_WZ        = 30.0   # vitesse rotation chassis entre axes (deg/s)
SEARCH_FAN_ROT_TIMEOUT_S = 8.0    # securite : 60deg a 30deg/s ~= 2s, marge x4
SEARCH_TOTAL_TIMEOUT_S   = 90.0   # borne recherche autonome avant abandon (s)
SEARCH_TOTAL_DIST_M      = 13.5   # borne recherche autonome avant abandon (m cumules, milieu plage 12-15m)

# ── APPROACH : gimbal verrouille sur la balise + chassis suit l'angle gimbal ──
K_GIM_YAW   = 2.0   # deg/s de servo gimbal par degre d'erreur laterale balise
GIM_YAW_MAX = 50.0  # vitesse max servo gimbal (deg/s)
K_BODY_YAW  = 0.8   # gain rotation chassis pour annuler yaw_rel (alignement corps)
GIM_YAW_SIGN = -1   # signe du servo gimbal : -1 confirme au test 2026-06-26 (drive_speed(positive) = gauche, yaw_err_deg<0 = gauche -> sign negatif converge)

# ── Servo PITCH (centrage vertical) — utilise UNIQUEMENT par l'auto-suivi LOCK en
# mode MANUEL (2026-07-22), pas par APPROACH (etat teste laisse intact). Garde les 4
# LED de la balise dans le champ -> detection plus stable sous angle.
#
# ⚠️ DESACTIVE le 2026-07-22 apres INCIDENT MATERIEL (BUG-058) : le signe du pitch
# n'ayant pas ete confirme, au 1er clic LOCK la nacelle a pitche en butee et accroche
# le cable RNDIS -> surcharge moteur -> LED rouge. Le suivi pitch reste DESACTIVE tant
# que (a) le signe n'est pas verifie a la main gimbal libre, ET (b) des limites d'angle
# de nacelle ne sont pas ajoutees pour empecher tout emballement d'atteindre un cable.
# Passer GIM_PITCH_TRACK_ENABLED=True seulement apres ces deux conditions.
GIM_PITCH_TRACK_ENABLED = False
GIM_PITCH_SPEED_MAX = 30.0   # vitesse max servo pitch (deg/s, conservateur)
GIM_PITCH_SIGN      = -1     # ⚠️ signe NON confirme (cf. incident BUG-058)

# ── LOCK BALISE : centrage periodique par mouvement relatif (2026-07-23) ──────
# Historique : un LOCK v1 (servo P continu a 20Hz, gating/rampe/rejet d'aberration)
# a existe puis a ete retire le 2026-07-23 (juge redondant/moins pratique que le
# centrage periodique ci-dessous). Seul ce mecanisme subsiste desormais : toutes les
# GIM_LOCK_PERIOD_S secondes (configurable en direct via /carolus/gimbal_lock_period),
# on lit l'erreur d'angle balise->centre camera et on envoie UNE commande de position
# relative gimbal.move(yaw=...) qui la ramene au centre, independamment du mouvement
# du chassis. gimbal.move() caracterise ASYNC dans test_gimbal_sweep.py (rend la main
# immediatement).
GIM_LOCK_PERIOD_S_DEFAULT = 3.0   # periode par defaut (s) ; repli si valeur recue invalide
# Vitesse au plafond SDK (2026-07-23, demande explicite utilisateur : vitesse max).
# Le SDK documente move(yaw_speed=...) dans [0, 540] deg/s. Jamais teste au-dela de
# 80 deg/s sur CE robot avant ce changement (test_gimbal_sweep.py), mais le risque
# BUG-058 (butee mecanique + cable accroche) est juge faible ici : ce sont des
# mouvements ponctuels de PETIT angle borne (deadband/max_err ci-dessous, pas un grand
# balayage vers une butee) -- une vitesse plus elevee raccourcit juste la duree du
# mouvement, pas son amplitude angulaire.
GIM_LOCK_YAW_SPEED   = 540.0
GIM_LOCK_MAX_ERR_DEG = 45.0   # au-dela, pose aberrante probable -> ignorer ce tick
# Deadband (2026-07-23) : sous ce seuil, ne pas re-corriger -- evite de "chasser" un
# residu de bruit de mesure P4P / imprecision mecanique qui ne descendra plus.
# 5.0° confirme par test materiel comme le bon reglage (reduction a 3.0° tentee puis
# revert le meme soir, sur demande utilisateur -- 5.0 etait deja le comportement
# juge correct).
GIM_LOCK_DEADBAND_DEG = 5.0
# RECENTRER CAM (2026-07-23 nuit) : gimbal.recenter() est une action ASYNC qui peut
# prendre jusqu'a ~0.7s (grand angle a 360 deg/s). La boucle MANUEL reemet
# drive_speed(0,0) a 20Hz, ce qui ANNULE l'action au tick suivant (~50ms) -> le
# recenter n'aboutit jamais. Fix : une fenetre "gimbal occupe" pendant laquelle la
# boucle MANUEL (et le tick LOCK) suspendent leurs commandes, laissant l'action
# aboutir. C'est aussi pourquoi le LOCK (move() de petit angle, fini en <20ms)
# marchait sans fenetre alors que le recenter non.
GIMBAL_RECENTER_BUSY_S = 2.5
# Pitch NON inclus par defaut : suivi pitch desactive depuis l'incident BUG-058
# (nacelle en butee -> cable accroche). Reutilise le garde-fou GIM_PITCH_TRACK_ENABLED --
# ne pas activer sans (a) verifier GIM_PITCH_SIGN et (b) limites d'angle.

# ── ALIGN : pre-orientation chassis avant APPROACH ──────────────────────────────
# Perplexity 05 + littérature visual servoing (look-and-move) :
# orienter d'abord, approcher ensuite. Gimbal OFF pendant ALIGN (world-stabilise suffit).
ALIGN_YAW_THRESHOLD = 12.0   # |yaw_rel| < X deg -> compter pose valide
ALIGN_YAW_DEADBAND  = 2.0    # |yaw_rel| < X deg -> wz = 0 (evite chattering)
ALIGN_MAX_WZ        = 8.0    # deg/s max pendant ALIGN (plus doux qu'APPROACH)
ALIGN_TIMEOUT_ALPHA = 1.5    # T_max = alpha * |yaw_rel_init| / ALIGN_MAX_WZ
ALIGN_TIMEOUT_MAX_S = 12.0   # cap du timeout dynamique (s)
ALIGN_VALID_POSES   = 3      # nb poses consecutives valides requis avant APPROACH

MANUAL_CMDVEL_TIMEOUT = 0.5   # arret auto si pas de commande depuis X secondes

# ── Évitement d'obstacles ────────────────────────────────────────────────────
OBSTACLE_TOF_CM     = 50    # distance TOF frontale (cm) → stop d'urgence
LOOKAHEAD_M         = 0.60  # distance de prévision collision devant le robot (m)
MAP_JSON_PATH       = "/home/ubuntu/carolus_map.json"
MAP_INFLATION_CELLS = 0     # pas d'inflation — obstacles dessinés au bloc près sur la map

GIMBAL_PITCH_MAX = 55.0   # deg/s (limite mecanique approximative)
GIMBAL_YAW_MAX   = 90.0

# ── Covariance /odom (2026-07-22) ────────────────────────────────────────────
# Diagonale non nulle : un EKF (F3, robot_localization) refuse une covariance
# toute-a-zero (interpretee comme "certitude parfaite" -> filtre casse). Ordre ROS
# 6x6 aplati : [x, y, z, roll, pitch, yaw]. Robot 2D -> z/roll/pitch verrouilles a
# grande variance (_ODOM_BIG). Valeurs indicatives (odometrie roue correcte a court
# terme, derive lente) — a affiner en F3 si besoin. Constantes reutilisees a chaque
# publication (pas de reconstruction de liste par tick).
_ODOM_BIG = 1e6
_ODOM_POSE_COV = [
    0.02, 0.0,  0.0,       0.0,       0.0,       0.0,
    0.0,  0.02, 0.0,       0.0,       0.0,       0.0,
    0.0,  0.0,  _ODOM_BIG, 0.0,       0.0,       0.0,
    0.0,  0.0,  0.0,       _ODOM_BIG, 0.0,       0.0,
    0.0,  0.0,  0.0,       0.0,       _ODOM_BIG, 0.0,
    0.0,  0.0,  0.0,       0.0,       0.0,       0.05,
]
_ODOM_TWIST_COV = [
    0.02, 0.0,  0.0,       0.0,       0.0,       0.0,
    0.0,  0.02, 0.0,       0.0,       0.0,       0.0,
    0.0,  0.0,  _ODOM_BIG, 0.0,       0.0,       0.0,
    0.0,  0.0,  0.0,       _ODOM_BIG, 0.0,       0.0,
    0.0,  0.0,  0.0,       0.0,       _ODOM_BIG, 0.0,
    0.0,  0.0,  0.0,       0.0,       0.0,       0.05,
]


# =========================================================
# Helpers
# =========================================================

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def _angle_diff_deg(a, b):
    """Plus petite difference angulaire signee a-b, en degres, dans [-180, 180]."""
    return ((a - b + 180.0) % 360.0) - 180.0


class _FullBatSubject(_BatBase):
    """Expose température, courant et tension ADC que BatterySubject.data_info() cache."""
    def data_info(self):
        # struct '<HhiBB' : adc_value(uint16), temperature(int16 en 0.1°C), current(int32 en mA), percent(uint8)
        temp_c = self._temperature / 10.0
        return (self._percent, temp_c, self._current, self._adc_value)


@dataclass
class PoseData:
    x:     float
    y:     float
    z:     float
    stamp: float
    yaw:   float = 0.0   # yaw balise en degrés (frame camera → estimation orientation)


class PoseBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None

    def update(self, x, y, z, yaw=0.0):
        with self._lock:
            self._pose = PoseData(x=float(x), y=float(y), z=float(z),
                                   stamp=time.time(), yaw=float(yaw))

    def get(self):
        with self._lock:
            return self._pose

    def invalidate(self):
        """Vide le buffer a chaque transition d'etat pour rejeter les poses stale."""
        with self._lock:
            self._pose = None


# =========================================================
# Node principal
# =========================================================

class EPCameraBeaconFollower:

    def __init__(self):
        self.bridge   = CvBridge()
        self.posebuf  = PoseBuffer()
        self.running  = True

        self.search_dir    = 1       # sens du sweep gimbal (+1 / -1)
        self.search_edges  = 0       # nb de bords atteints (2 = cycle complet -> avancer)
        self.search_steps  = 0       # nb d'avancements effectues sans detection sur l'axe courant
        self._search_since = time.time()

        # Motif en eventail (axe epuise -> rotation chassis -> axe suivant)
        self.search_axis_idx       = 0     # index de l'axe courant (0..SEARCH_FAN_AXIS_COUNT-1)
        self._search_dist_total_m  = 0.0   # distance cumulee avancee sur l'episode de recherche courant
        self._search_episode_since = time.time()   # horodatage debut d'episode (reset a l'entree fraiche AUTO/LOCATE)

        # Etat ALIGN
        self._align_since    = 0.0
        self._align_yaw_init = 0.0
        self._align_valid_n  = 0     # poses consecutives valides pendant ALIGN

        # Telemetrie consolidee (ESC, attitude, position, batterie, vitesse, statut, TOF)
        # Un seul lock pour 7 flux : reduit contention + overhead vs 7 locks separes.
        self._telem_lock = threading.Lock()
        self._telem = {
            'esc':    [0, 0, 0, 0],
            'atti':   (0.0, 0.0, 0.0),   # yaw, pitch, roll (deg)
            'pos':    (0.0, 0.0),          # x, y odometrie (m)
            'bat':    (0, 0.0, 0, 0),      # pct, temp_c, cur_ma, adc
            'vel':    (0.0, 0.0),          # vbx, vby (m/s)
            'status': {},
            'dist':   999.0,               # TOF frontal (cm)
        }

        # Carte d'obstacles (chargée si MAP_JSON_PATH existe)
        self._colmap = MapCollision(MAP_JSON_PATH, inflation_cells=MAP_INFLATION_CELLS)

        # Commande roues individuelles (mode MANUEL seulement)
        self._wheels_active = False
        self._wheels_vals   = (0, 0, 0, 0)
        self._wheels_lock   = threading.Lock()

        # Angle gimbal lu via sub_angle : (pitch, yaw_rel, pitch_ground, yaw_ground)
        # yaw_rel = angle gimbal vs chassis (signal d'alignement APPROACH)
        # yaw_ground = cap absolu de la nacelle (repere monde, stabilise inertiellement)
        self._gim_yaw_rel = 0.0
        self._gim_yaw_ground = 0.0
        self._gim_angle_lock = threading.Lock()

        # Mode : "AUTO" (recherche/approche) ou "MANUAL" (commandes ZQSD du GUI).
        # DEFAUT = MANUAL depuis le 2026-07-22 (securite BUG-058) : demarrer en AUTO
        # faisait balayer la nacelle (SEARCH) des le lancement -> risque d'accrocher un
        # cable. En MANUAL le robot reste immobile tant que l'utilisateur ne commande
        # rien. Le passage en AUTO reste explicite (bouton MODE du launcher).
        self._mode          = "MANUAL"
        self._mode_lock     = threading.Lock()
        self._man_vx        = 0.0
        self._man_wz        = 0.0
        self._man_stamp     = 0.0   # timestamp derniere commande MANUEL recue
        self._man_lock      = threading.Lock()
        # Gimbal manuel
        self._gim_pitch = 0.0
        self._gim_yaw   = 0.0
        self._gim_stamp = 0.0
        self._gim_lock  = threading.Lock()
        # LOCK BALISE (2026-07-23) : centrage periodique, OFF par defaut. Le v1
        # historique (servo continu 20Hz) a ete retire le meme jour (redondant).
        self._gimbal_lock = False
        self._gimbal_lock_period_s = GIM_LOCK_PERIOD_S_DEFAULT   # configurable en direct (/carolus/gimbal_lock_period)
        self._lock_timer = None   # rospy.Timer courant, recree quand la periode change
        # Fenetre "gimbal occupe" (2026-07-23 nuit) : tant que time.time() < cette
        # valeur, une action async gimbal (recenter) est en cours -> la boucle MANUEL
        # et le tick LOCK suspendent leurs commandes pour ne pas l'annuler.
        self._gimbal_busy_until = 0.0

        self._beacon_was_fresh = False   # detecte transition LOST->DETECTED (voyant/minimap)

        # Connexion unique au robot
        rospy.loginfo("[RM] Connexion RNDIS...")
        self.ep = robot.Robot()
        self.ep.initialize(conn_type="rndis")
        self.cam     = self.ep.camera
        self.chassis = self.ep.chassis
        self.gimbal  = self.ep.gimbal
        rospy.loginfo("[RM] Robot connecte")

        # Sortir le gimbal de l'etat veille (sinon couple coupe, moteurs mous)
        try:
            self.gimbal.resume()
            rospy.loginfo("[GIMBAL] resume OK — couple actif")
        except Exception as e:
            rospy.logwarn(f"[GIMBAL] resume failed: {e}")

        # Publishers gimbal crees AVANT sub_angle (BUG-064bis, 2026-07-23) : le SDK
        # peut invoquer _gimbal_angle_cb des l'inscription du callback, avant que le
        # reste de __init__ ne s'execute — AttributeError observe en session reelle
        # sur pub_gimbal_yaw quand la creation du Publisher restait plus bas (ligne
        # ~399 auparavant).
        self.pub_gimbal_yaw = rospy.Publisher("/carolus/gimbal_yaw_rel", Float32, queue_size=10)
        self.pub_gimbal_yaw_ground = rospy.Publisher("/carolus/gimbal_yaw_ground", Float32, queue_size=10)
        # TF dynamique base_link->camera_link (BUG-067, 2026-07-23, F3) : le gimbal
        # bouge reellement (yaw_rel/pitch_rel non nuls des que la nacelle n'est pas
        # recentree) -- une TF statique identite (testcarolus.launch) rendait le calcul
        # de pose absolue de beacon_absolute_pose.py faux des que la nacelle tournait
        # (z aberrant observe en test reel : 1.68m au lieu de ~0m). Publiee ici, pas
        # dans carolus_tf_broadcaster.py, car pitch_rel/yaw_rel sont deja en memoire
        # dans ce process (pas de topic intermediaire necessaire).
        self.tf_br_gimbal = tf2_ros.TransformBroadcaster()

        # Suivi de l'angle gimbal en temps reel. REVERT a 10 Hz le 2026-07-22 : le
        # passage a 20 Hz avait rendu sub_angle silencieux (gimbal_yaw_rel ne publiait
        # plus, retour d'angle mort) — 20 n'est probablement pas une freq valide pour ce
        # subject, ou le lien DDS sature. 10 Hz est la valeur d'origine, connue bonne.
        try:
            self.gimbal.sub_angle(freq=10, callback=self._gimbal_angle_cb)
            rospy.loginfo("[GIMBAL] sub_angle OK — suivi yaw_rel/yaw_ground actif")
        except Exception as e:
            rospy.logwarn(f"[GIMBAL] sub_angle failed: {e}")

        # Télémétrie batterie etendue (percent + temp + courant + tension ADC)
        try:
            self.battery = self.ep.battery
            _bat_subj = _FullBatSubject()
            _bat_subj.freq = 1
            self.ep.dds.add_subject_info(_bat_subj, self._battery_cb, (), {})
            rospy.loginfo("[BAT] Souscription batterie etendue OK (temp+courant+tension)")
        except Exception as e:
            rospy.logwarn(f"[BAT] indisponible: {e}")

        # Telemetrie ESC (vitesses roues) — best-effort
        try:
            self.chassis.sub_esc(freq=5, callback=self._esc_cb)
            rospy.loginfo("[ESC] sub_esc OK")
        except Exception as e:
            rospy.logwarn(f"[ESC] sub_esc failed: {e}")

        # Telemetrie attitude (pitch/roll/yaw chassis) — best-effort.
        # REVERT a 5 Hz le 2026-07-22 (valeur d'origine) : retour a un etat stable
        # connu apres que les bumps a 20 Hz aient coincide avec sub_angle silencieux.
        try:
            self.chassis.sub_attitude(freq=5, callback=self._atti_cb)
            rospy.loginfo("[ATTI] sub_attitude OK")
        except Exception as e:
            rospy.logwarn(f"[ATTI] sub_attitude failed: {e}")

        # Telemetrie position (odometrie relative au demarrage) — best-effort.
        # REVERT a 5 Hz le 2026-07-22 (valeur d'origine).
        try:
            self.chassis.sub_position(cs=0, freq=5, callback=self._pos_cb)
            rospy.loginfo("[POS] sub_position OK")
        except Exception as e:
            rospy.logwarn(f"[POS] sub_position failed: {e}")

        # Vitesse chassis en temps reel (frame corps) — best-effort.
        # REVERT a 5 Hz le 2026-07-22 (valeur d'origine).
        try:
            self.chassis.sub_velocity(freq=5, callback=self._vel_cb)
            rospy.loginfo("[VEL] sub_velocity OK")
        except Exception as e:
            rospy.logwarn(f"[VEL] sub_velocity failed: {e}")

        # Statut chassis (pickup/slip/roll/impact) — best-effort
        try:
            self.chassis.sub_status(freq=5, callback=self._status_cb)
            rospy.loginfo("[STATUS] sub_status OK")
        except Exception as e:
            rospy.logwarn(f"[STATUS] sub_status failed: {e}")

        # Distance TOF (capteur IR frontal, évitement réactif) — best-effort
        try:
            self.ep.sensor.sub_distance(freq=10, callback=self._dist_cb)
            rospy.loginfo("[TOF] sub_distance OK")
        except Exception as e:
            rospy.logwarn(f"[TOF] sub_distance failed (normal sur S1): {e}")

        # Carte d'obstacles
        if self._colmap.loaded:
            rospy.loginfo(f"[MAP] Carte obstacle chargee depuis {MAP_JSON_PATH}")
        else:
            rospy.logwarn(f"[MAP] {MAP_JSON_PATH} absent — collision map desactive")

        self.cam.start_video_stream(display=False)
        rospy.loginfo("[RM] Flux camera demarre")

        # Publishers / subscribers ROS
        self.pub_img = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
        # Odometrie chassis (F2, 2026-07-21) : republie sub_position/sub_attitude (deja
        # souscrits ci-dessus, connexion SDK unique) sur /odom pour capture propre
        # (rostopic echo > fichier) au lieu de lire les logs texte a l'oeil. Position
        # convertie en REP-103 (y_ros=-y_ep, cf. Perplexity 07 / GitHub SDK officiel :
        # EP y+=droite, ROS y+=gauche). Yaw publie tel quel (signe EP non confirme pour
        # une orientation absolue — suffisant pour une mesure de derive relative comme
        # ici ; a verifier avant reutilisation dans l'EKF robot_localization de F3).
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        # pub_gimbal_yaw / pub_gimbal_yaw_ground crees plus haut (avant sub_angle,
        # cf. commentaire ligne ~311) — pas de re-creation ici.
        rospy.Subscriber("/pose",               PoseStamped, self._pose_cb)
        rospy.Subscriber("/carolus/mode",       String,      self._mode_cb)
        rospy.Subscriber("/carolus/cmd_vel",    Twist,       self._cmdvel_cb)
        rospy.Subscriber("/carolus/gimbal_vel", Twist,       self._gimbal_cb)
        rospy.Subscriber("/carolus/wheels",     String,      self._wheels_cb)
        rospy.Subscriber("/carolus/gimbal_lock", String,     self._gimbal_lock_cb)
        rospy.Subscriber("/carolus/gimbal_lock_period", String, self._gimbal_lock_period_cb)
        rospy.Subscriber("/carolus/gimbal_recenter", String, self._gimbal_recenter_cb)
        rospy.loginfo("[ROS] Subscriptions actives (/pose, /carolus/mode, /carolus/cmd_vel, /carolus/gimbal_vel, /carolus/wheels, /carolus/gimbal_lock, /carolus/gimbal_lock_period, /carolus/gimbal_recenter)")

        # LOCK BALISE (2026-07-23) : timer independant de la boucle principale 20Hz --
        # centrage periodique, pas un servo continu. Stocke sur self._lock_timer pour
        # pouvoir le recreer avec une nouvelle periode (cf. _gimbal_lock_period_cb).
        self._lock_timer = rospy.Timer(rospy.Duration(self._gimbal_lock_period_s), self._gimbal_lock_tick)
        # Statut balise DETECTED/LOST (2026-07-23) : log periodique parse par
        # carolus_launcher.py (meme pattern que [BAT]/[ATTI]/[POS]) pour le voyant et
        # la minimap. 5Hz -- independant de l'etat ON/OFF du LOCK.
        rospy.Timer(rospy.Duration(0.2), self._beacon_status_tick)

        self.rate = rospy.Rate(TARGET_FPS)

    # ---- callbacks ROS ----

    def _pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        # Yaw de la balise (rotation autour de l'axe y camera = tilt horizontal)
        siny = 2.0 * (q.w * q.y + q.z * q.x)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        self.posebuf.update(p.x, p.y, p.z, yaw_deg)

    def _battery_cb(self, info):
        try:
            if isinstance(info, (list, tuple)) and len(info) == 4:
                pct, temp_c, cur_ma, adc = info
            else:
                pct  = info[0] if isinstance(info, (list, tuple)) else int(info)
                temp_c, cur_ma, adc = 0.0, 0, 0
            with self._telem_lock:
                self._telem['bat'] = (int(pct), float(temp_c), int(cur_ma), int(adc))
        except Exception:
            pass

    def _mode_cb(self, msg):
        new_mode = msg.data.strip().upper()
        if new_mode not in ("AUTO", "MANUAL", "LOCATE"):
            return
        with self._mode_lock:
            old_mode = self._mode
            self._mode = new_mode
        if new_mode == "MANUAL" and old_mode != "MANUAL":
            self.stop_chassis()
            try:
                self.gimbal.resume()   # re-activer le couple au cas ou drive_speed(0,0) l'aurait suspendu
            except Exception as e:
                rospy.logwarn(f"[GIMBAL] resume at MANUAL: {e}")
            rospy.loginfo("[CTRL] Mode -> MANUAL")
        elif new_mode == "LOCATE" and old_mode != "LOCATE":
            self.stop_chassis()
            self._search_since = time.time()
            self.search_edges  = 0
            self.search_steps  = 0
            self.search_axis_idx       = 0
            self._search_dist_total_m  = 0.0
            self._search_episode_since = time.time()
            rospy.loginfo("[CTRL] Mode -> LOCATE (sweep gimbal, no advance, no approach)")
        elif new_mode == "AUTO" and old_mode in ("MANUAL", "LOCATE"):
            self.stop_chassis()                # arret immediat des vitesses residuelles MANUEL
            self._search_since = time.time()   # grace period avant de recommencer a balayer
            self.search_edges  = 0             # repart sur un cycle de sweep propre
            self.search_steps  = 0             # repart sur un compteur d'avancements propre
            self.search_axis_idx       = 0             # nouvel episode -> premier axe du motif
            self._search_dist_total_m  = 0.0           # nouvel episode -> distance cumulee remise a zero
            self._search_episode_since = time.time()   # nouvel episode -> horloge globale remise a zero
            try:
                self.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            except Exception:
                pass
            with self._gim_lock:
                self._gim_pitch = 0.0
                self._gim_yaw   = 0.0
            rospy.loginfo("[CTRL] Mode -> AUTO (grace period active)")

    def _cmdvel_cb(self, msg):
        with self._man_lock:
            self._man_vx    = msg.linear.x
            self._man_wz    = msg.angular.z
            self._man_stamp = time.time()

    def _gimbal_cb(self, msg):
        with self._gim_lock:
            self._gim_pitch = clamp(msg.angular.y, -GIMBAL_PITCH_MAX, GIMBAL_PITCH_MAX)
            self._gim_yaw   = clamp(msg.angular.z, -GIMBAL_YAW_MAX,   GIMBAL_YAW_MAX)
            self._gim_stamp = time.time()

    def _gimbal_lock_cb(self, msg):
        self._gimbal_lock = (msg.data.strip().upper() == "ON")
        rospy.loginfo(f"[LOCK] Lock balise (centrage periodique) -> {'ON' if self._gimbal_lock else 'OFF'}")

    def _gimbal_lock_period_cb(self, msg):
        """Periode de centrage configurable en direct (2026-07-23), en SECONDES
        uniquement. Repli silencieux sur GIM_LOCK_PERIOD_S_DEFAULT si la valeur recue
        n'est pas un nombre positif (meme logique de tolerance qu'un champ web)."""
        try:
            period = float(msg.data)
            if period <= 0:
                raise ValueError
        except (ValueError, TypeError):
            period = GIM_LOCK_PERIOD_S_DEFAULT
            rospy.logwarn(f"[LOCK] periode recue invalide ({msg.data!r}) -> repli sur {period}s")
        self._gimbal_lock_period_s = period
        self._lock_timer.shutdown()
        self._lock_timer = rospy.Timer(rospy.Duration(period), self._gimbal_lock_tick)
        rospy.loginfo(f"[LOCK] periode de centrage -> {period}s")

    def _gimbal_lock_tick(self, event):
        """LOCK BALISE (2026-07-23) : toutes les self._gimbal_lock_period_s secondes,
        si actif et une pose fraiche existe, envoie UNE commande gimbal.move() relative
        qui centre la balise dans le champ. Independant du mouvement chassis.
        Yaw seulement -- pitch reste derriere GIM_PITCH_TRACK_ENABLED (BUG-058)."""
        if not self._gimbal_lock:
            return
        if self._mode != "MANUAL":
            return
        # Une action RECENTER en cours ? -> ne pas la perturber par un move() concurrent.
        if time.time() < self._gimbal_busy_until:
            return
        # NB : le numpad n'a plus la priorite ici (2026-07-23 nuit) -- tant que LOCK est
        # ON, le pilotage manuel de la nacelle est ignore (cote boucle MANUEL aussi).
        if not self.has_fresh_pose():
            return
        p = self.posebuf.get()
        if p is None or abs(p.z) < 0.05:
            return
        yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
        if abs(yaw_err_deg) > GIM_LOCK_MAX_ERR_DEG:
            rospy.logwarn(f"[LOCK] erreur trop grande ({yaw_err_deg:.1f}°), pose probablement aberrante -> ignoree")
            return
        if abs(yaw_err_deg) < GIM_LOCK_DEADBAND_DEG:
            rospy.loginfo(f"[LOCK] deja centre (err={yaw_err_deg:.1f}°) -> pas de correction")
            return
        pitch_delta = 0.0
        if GIM_PITCH_TRACK_ENABLED:
            pitch_err_deg = math.degrees(math.atan2(p.y, abs(p.z)))
            pitch_delta = GIM_PITCH_SIGN * pitch_err_deg
        try:
            self.gimbal.move(pitch=pitch_delta, yaw=GIM_YAW_SIGN * yaw_err_deg,
                              pitch_speed=GIM_PITCH_SPEED_MAX, yaw_speed=GIM_LOCK_YAW_SPEED)
            rospy.loginfo(f"[LOCK] centrage : yaw_err={yaw_err_deg:.1f}° -> move(yaw={GIM_YAW_SIGN*yaw_err_deg:.1f})")
        except Exception as e:
            rospy.logwarn(f"[LOCK] gimbal.move a echoue: {e}")

    def _beacon_status_tick(self, event):
        """Log periodique DETECTED/LOST (2026-07-23), parse par carolus_launcher.py
        (voyant, minimap)."""
        fresh = self.has_fresh_pose()
        self._beacon_was_fresh = fresh
        if fresh:
            p = self.posebuf.get()
            if p is not None and abs(p.z) > 0.05:
                yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
                pitch_err_deg = math.degrees(math.atan2(p.y, abs(p.z)))
                rospy.loginfo(f"[BEACON] status=DETECTED yaw_err={yaw_err_deg:.1f} pitch_err={pitch_err_deg:.1f}")
                return
        rospy.loginfo("[BEACON] status=LOST")

    def _gimbal_recenter_cb(self, msg):
        """RECENTRER CAM (2026-07-23) : ramene la nacelle a sa position de base
        (pitch=0, yaw=0 repere power-on du gimbal, gimbal.recenter() du SDK) --
        orientation de la CAMERA, independant de l'orientation du chassis. Meme
        scope que LOCK (mode MANUEL uniquement)."""
        if self._mode != "MANUAL":
            rospy.logwarn("[GIMBAL] RECENTER ignore : hors mode MANUEL")
            return
        # Fenetre "occupe" AVANT de lancer l'action : sinon la boucle MANUEL (20Hz)
        # ou le tick LOCK enverraient une commande concurrente qui annulerait le
        # recenter avant qu'il n'aboutisse (cause du bug "recenter ne marche pas").
        self._gimbal_busy_until = time.time() + GIMBAL_RECENTER_BUSY_S
        try:
            # recenter() plafonne a [-360,360] deg/s (different de move(), 540 max) --
            # ne pas reutiliser GIM_LOCK_YAW_SPEED (540) tel quel, hors plage ici.
            self.gimbal.recenter(pitch_speed=360.0, yaw_speed=360.0)
            rospy.loginfo("[GIMBAL] RECENTER : nacelle vers position de base (pitch=0, yaw=0)")
        except Exception as e:
            self._gimbal_busy_until = 0.0   # echec -> liberer tout de suite
            rospy.logwarn(f"[GIMBAL] RECENTER : gimbal.recenter a echoue: {e}")

    def _gimbal_angle_cb(self, info):
        if not isinstance(info, (list, tuple)) or len(info) < 4:
            return
        with self._gim_angle_lock:
            self._gim_yaw_rel    = info[1]
            self._gim_yaw_ground = info[3]
        self.pub_gimbal_yaw.publish(Float32(data=info[1]))
        self.pub_gimbal_yaw_ground.publish(Float32(data=info[3]))

        # TF dynamique base_link->camera_link (F3, 2026-07-23) — pitch_rel=info[0],
        # yaw_rel=info[1], convention SDK EP non confirmee (meme reserve deja notee
        # pour yaw_rel/yaw_ground ailleurs dans ce fichier) : signe a verifier au 1er
        # test hardware dedie (comparer TF affichee vs mouvement gimbal connu).
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        q = tft.quaternion_from_euler(0.0, math.radians(info[0]), math.radians(info[1]))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_br_gimbal.sendTransform(t)

    def _esc_cb(self, sub_info):
        try:
            speed, angle, timestamp, state = sub_info
            with self._telem_lock:
                self._telem['esc'] = list(speed)
        except Exception:
            pass

    def _atti_cb(self, sub_info):
        try:
            yaw, pitch, roll = sub_info
            with self._telem_lock:
                self._telem['atti'] = (float(yaw), float(pitch), float(roll))
        except Exception:
            pass

    def _pos_cb(self, sub_info):
        try:
            x, y, z = sub_info
            with self._telem_lock:
                self._telem['pos'] = (float(x), float(y))
        except Exception:
            pass

    def _vel_cb(self, sub_info):
        try:
            vgx, vgy, vgz, vbx, vby, vbz = sub_info
            with self._telem_lock:
                self._telem['vel'] = (float(vbx), float(vby))
        except Exception:
            pass

    def _status_cb(self, sub_info):
        try:
            static, up_hill, down_hill, on_slope, is_pickup, slip, ix, iy, iz, roll_over, hill_static = sub_info
            with self._telem_lock:
                self._telem['status'] = {
                    'is_pickup': is_pickup,
                    'slip':      slip,
                    'roll_over': roll_over,
                    'on_slope':  on_slope,
                    'impact':    (ix or iy or iz),
                }
        except Exception:
            pass

    def _dist_cb(self, sub_info):
        try:
            vals = [float(v) for v in sub_info if v is not None and float(v) > 0]
            with self._telem_lock:
                self._telem['dist'] = min(vals) if vals else 999.0
        except Exception:
            pass

    def _wheels_cb(self, msg):
        parts = msg.data.strip().split()
        with self._wheels_lock:
            if not parts or parts[0].upper() == "STOP":
                self._wheels_active = False
                self._wheels_vals   = (0, 0, 0, 0)
            elif len(parts) == 4:
                try:
                    w = tuple(int(p) for p in parts)
                    self._wheels_vals   = w
                    self._wheels_active = True
                except ValueError:
                    pass

    def _is_path_blocked(self):
        """Arbiter prioritaire : TOF < seuil OU collision map OU hors zone → (True, raison)."""
        with self._telem_lock:
            tof     = self._telem['dist']
            px, py  = self._telem['pos']
            yaw_deg = self._telem['atti'][0]
        if tof < OBSTACLE_TOF_CM:
            return True, f"TOF={tof:.0f}cm"
        if self._colmap.loaded:
            if not self._colmap.lookahead_is_free(px, py, yaw_deg, LOOKAHEAD_M):
                return True, f"MAP col={px:.2f}m yaw={yaw_deg:.0f}deg"
            if not self._colmap.point_in_zone(px, py):
                return True, f"GEOFENCE x={px:.2f} y={py:.2f}"
        return False, ""

    def get_gimbal_yaw_rel(self):
        with self._gim_angle_lock:
            return self._gim_yaw_rel

    # ---- helpers ----

    def has_fresh_pose(self):
        p = self.posebuf.get()
        return (p is not None) and (time.time() - p.stamp < POSE_TIMEOUT_S)

    def stop_chassis(self):
        try:
            self.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        except Exception:
            pass

    def _is_auto(self):
        with self._mode_lock:
            return self._mode in ("AUTO", "LOCATE")

    def _is_locate(self):
        with self._mode_lock:
            return self._mode == "LOCATE"

    def stop_gimbal(self):
        try:
            self.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        except Exception:
            pass

    def _gimbal_servo_yaw(self, p):
        """Vitesse yaw nacelle (deg/s) pour centrer la balise sur l'erreur laterale.
        Utilise par l'etat APPROACH (AUTO)."""
        yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
        return clamp(GIM_YAW_SIGN * K_GIM_YAW * yaw_err_deg, -GIM_YAW_MAX, GIM_YAW_MAX)

    def do_gimbal_sweep(self):
        """Balaye le gimbal dans le sens courant. INSENSIBLE AU SIGNE de drive_speed :
        la fin de course est detectee par |yaw_rel| > limite OU par STALL (yaw_rel
        ne progresse plus = butee mecanique atteinte). Le chassis ne bouge PAS.
        Renvoie : 'FOUND' (pose detectee) / 'EDGE' (fin de course) / 'MANUAL' (switch).
        Pilotage a l'angle lu (yaw_rel), la consigne de vitesse n'est pas fiable."""
        last_yaw = self.get_gimbal_yaw_rel()
        stall_t  = time.time()
        t_log    = 0.0
        # "armed" : on ne peut detecter une fin de course que si on est d'abord
        # revenu en zone centrale. Evite le re-declenchement immediat quand on
        # demarre la passe en ayant deja depasse la borne (overshoot du pass precedent).
        armed = abs(last_yaw) < (SEARCH_YAW_LIMIT - 15)
        while self.running and not rospy.is_shutdown():
            if self.has_fresh_pose():
                self.stop_gimbal()
                return "FOUND"
            if not self._is_auto():
                self.stop_gimbal()
                return "MANUAL"
            yaw = self.get_gimbal_yaw_rel()
            now = time.time()
            if now - t_log > 1.0:
                rospy.loginfo(f"[SEARCH] sweep dir={self.search_dir} yaw_rel={yaw:.1f} armed={armed}")
                t_log = now
            # progression : reset du timer de stall si l'angle bouge encore
            if abs(yaw - last_yaw) > 2.0:
                last_yaw = yaw
                stall_t  = now
            # armement une fois revenu en zone centrale
            if not armed and abs(yaw) < (SEARCH_YAW_LIMIT - 15):
                armed = True
            # fin de course : borne (seulement si arme) OU stall (toujours, gere le
            # signe inverse : si on pousse dans la butee mecanique, yaw ne bouge plus)
            hit_limit = armed and abs(yaw) > SEARCH_YAW_LIMIT
            hit_stall = (now - stall_t > 1.5)
            if hit_limit or hit_stall:
                self.stop_gimbal()
                reason = "borne" if hit_limit else "butee"
                rospy.loginfo(f"[SEARCH] fin de course ({reason}) yaw_rel={yaw:.1f} -> inversion")
                return "EDGE"
            try:
                self.gimbal.drive_speed(pitch_speed=0,
                                        yaw_speed=SEARCH_GIMBAL_YAW_S * self.search_dir)
            except Exception:
                pass
            time.sleep(0.05)
        self.stop_gimbal()
        return "MANUAL"

    def do_step_forward(self):
        """Sweep complet sans cible : recentre le gimbal sur le corps (yaw_rel -> 0),
        puis avance d'un pas en ligne droite. Surveille pose/mode pendant l'avance."""
        rospy.loginfo(f"[SEARCH] Cycle complet sans cible -> recenter + avance {SEARCH_STEP_M}m")
        try:
            self.gimbal.recenter(pitch_speed=0, yaw_speed=60)
        except Exception as e:
            rospy.logwarn(f"[SEARCH] recenter: {e}")
        t0 = time.time()
        while time.time() - t0 < RECENTER_TIMEOUT_S:
            if not self._is_auto() or self._is_locate():
                return
            if abs(self.get_gimbal_yaw_rel()) < 5.0:
                break
            time.sleep(0.1)
        dur = SEARCH_STEP_M / SEARCH_STEP_VX
        t = time.time()
        while time.time() - t < dur:
            if not self.running or rospy.is_shutdown():
                break
            if self.has_fresh_pose() or not self._is_auto() or self._is_locate():
                break
            blocked, reason = self._is_path_blocked()
            if blocked:
                rospy.logwarn(f"[SEARCH] avancement bloque ({reason}) — arret")
                break
            try:
                self.chassis.drive_speed(x=SEARCH_STEP_VX, y=0.0, z=0.0, timeout=1)
            except Exception:
                pass
            time.sleep(0.1)
        self.stop_chassis()

    def _rotate_body_by(self, delta_deg):
        """Tourne le chassis d'un angle fixe (valeur absolue mesuree via l'attitude EP).
        Rotation toujours dans le meme sens de commande — on vise une rotation RELATIVE
        fixe, pas un cap absolu, donc le signe exact de sub_attitude n'a pas besoin
        d'etre connu a l'avance (contrairement au servo gimbal ou au corps en ALIGN,
        qui visent une valeur cible precise).

        Le gimbal est recentre au prealable (comme dans do_step_forward) : il est
        stabilise inertiellement (repere monde), donc apres un sweep il peut etre
        proche de sa limite meca (~250deg). Tourner le chassis sous un gimbal reste
        au monde deplacerait yaw_rel d'autant — recentrer avant evite de le pousser
        au-dela de la limite."""
        try:
            self.gimbal.recenter(pitch_speed=0, yaw_speed=60)
        except Exception as e:
            rospy.logwarn(f"[SEARCH] recenter avant rotation axe: {e}")
        t0 = time.time()
        while time.time() - t0 < RECENTER_TIMEOUT_S:
            if not self._is_auto() or self._is_locate():
                return
            if abs(self.get_gimbal_yaw_rel()) < 5.0:
                break
            time.sleep(0.1)

        with self._telem_lock:
            yaw0 = self._telem['atti'][0]
        t0 = time.time()
        while time.time() - t0 < SEARCH_FAN_ROT_TIMEOUT_S:
            if not self.running or rospy.is_shutdown() or not self._is_auto() or self._is_locate():
                break
            with self._telem_lock:
                yaw_now = self._telem['atti'][0]
            if abs(_angle_diff_deg(yaw_now, yaw0)) >= delta_deg:
                break
            try:
                self.chassis.drive_speed(x=0.0, y=0.0, z=SEARCH_FAN_ROT_WZ, timeout=1)
            except Exception:
                pass
            time.sleep(0.05)
        self.stop_chassis()

    # ---- boucle de contrôle (thread séparé) ----

    def _control_loop(self):
        state = "SEARCH"
        self._search_since = time.time()
        rospy.loginfo("[CTRL] Demarrage — place la balise devant la camera")
        time.sleep(2.0)
        rospy.loginfo(f"[CTRL] State: SEARCH (grace period {SEARCH_GRACE_S:.0f}s)")

        while self.running and not rospy.is_shutdown():

            with self._mode_lock:
                mode = self._mode

            # Logging telemetrie (dashboard GUI) — throttle 1 Hz
            with self._telem_lock:
                s                    = list(self._telem['esc'])
                _yaw, _pitch, _roll  = self._telem['atti']
                _px, _py             = self._telem['pos']
                _pct, _tc, _cur, _adc = self._telem['bat']
                _vbx, _vby           = self._telem['vel']
                _st                  = dict(self._telem['status'])
                _tof                 = self._telem['dist']
            rospy.loginfo_throttle(1.0, f"[ESC] W1={s[0]} W2={s[1]} W3={s[2]} W4={s[3]}")
            rospy.loginfo_throttle(1.0, f"[ATTI] yaw={_yaw:.1f} pitch={_pitch:.1f} roll={_roll:.1f}")
            rospy.loginfo_throttle(1.0, f"[POS] x={_px:.3f} y={_py:.3f}")

            # Publication /odom (F2, 2026-07-21) — memes valeurs deja lues ci-dessus,
            # aucune souscription/connexion supplementaire. y converti REP-103 (-y_ep).
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = _px
            odom.pose.pose.position.y = -_py
            odom.pose.pose.position.z = 0.0
            yaw_rad = math.radians(_yaw)
            odom.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            odom.twist.twist.linear.x = _vbx
            odom.twist.twist.linear.y = -_vby
            odom.pose.covariance  = _ODOM_POSE_COV
            odom.twist.covariance = _ODOM_TWIST_COV
            self.pub_odom.publish(odom)
            rospy.loginfo_throttle(5.0, f"[BAT] {_pct}%  temp={_tc:.1f}C  cur={_cur}mA  adc={_adc}")
            rospy.loginfo_throttle(1.0, f"[VEL] vbx={_vbx:.3f} vby={_vby:.3f}")
            if any(_st.values()):
                rospy.loginfo_throttle(1.0,
                    f"[STATUS] pickup={_st.get('is_pickup',0)} slip={_st.get('slip',0)} "
                    f"roll={_st.get('roll_over',0)} slope={_st.get('on_slope',0)} impact={_st.get('impact',0)}")
            if _tof < 200:
                rospy.loginfo_throttle(1.0, f"[TOF] front={_tof:.0f}cm")

            # Position balise (pour map) si une pose fraiche existe
            _p = self.posebuf.get()
            if _p is not None and (time.time() - _p.stamp < POSE_TIMEOUT_S):
                rospy.loginfo_throttle(0.5,
                    f"[BEACONPOS] x={_p.x:.3f} z={_p.z:.3f} byaw={_p.yaw:.1f}")

            # ── MODE MANUEL ──────────────────────────────────────────────────
            if mode == "MANUAL":
                with self._man_lock:
                    vx    = self._man_vx
                    wz    = self._man_wz
                    stamp = self._man_stamp

                with self._wheels_lock:
                    wheels_active = self._wheels_active
                    w1, w2, w3, w4 = self._wheels_vals

                if wheels_active:
                    # Commande roues individuelles (tilt / wheelie)
                    try:
                        self.chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4, timeout=1)
                    except Exception:
                        pass
                else:
                    # Bug 4 : timeout securite — arret si pas de commande recente
                    if time.time() - stamp > MANUAL_CMDVEL_TIMEOUT:
                        vx, wz = 0.0, 0.0

                    # Garde-fou minimal en MANUEL (2026-07-22) : bloque l'AVANCE si un
                    # obstacle TOF est proche. Rotation (wz) et marche arriere (vx<0)
                    # restent autorisees pour se degager. TOF-only volontairement (pas
                    # la map/geofence, qui exigent une origine odometrique calibree —
                    # eviter les faux blocages en pilotage manuel). Inerte si le TOF est
                    # absent (S1 roote) : _telem['dist'] reste a 999.0 -> jamais declenche.
                    if vx > 0.0:
                        with self._telem_lock:
                            _tof_cm = self._telem['dist']
                        if _tof_cm < OBSTACLE_TOF_CM:
                            vx = 0.0
                            rospy.logwarn_throttle(1.0, f"[MANUAL] avance bloquee (TOF={_tof_cm:.0f}cm)")

                    # Envoi chassis a chaque tick (20 Hz) — le SDK invalide les commandes sans repetition
                    try:
                        self.chassis.drive_speed(x=vx, y=0.0, z=wz, timeout=1)
                    except Exception:
                        pass

                # Gimbal (MANUEL) : arbitrage en 3 priorites (2026-07-23 nuit) :
                #   1. Action async en cours (RECENTER) -> ne rien emettre, la laisser
                #      aboutir (sinon drive_speed(0,0) l'annulerait, cf. GIMBAL_RECENTER_BUSY_S).
                #   2. LOCK ON -> le tick _gimbal_lock_tick pilote via move() ; le numpad
                #      est IGNORE tant que le LOCK est actif (demande utilisateur).
                #   3. Sinon -> relais direct numpad (0,0 si aucune touche = maintien).
                # Le SDK invalide les commandes de vitesse apres ~0.1-0.5s sans repetition,
                # d'ou l'envoi systematique a chaque tick dans le cas 3.
                if time.time() < self._gimbal_busy_until:
                    pass
                elif self._gimbal_lock:
                    pass
                else:
                    with self._gim_lock:
                        gim_pitch = self._gim_pitch
                        gim_yaw   = self._gim_yaw
                    try:
                        self.gimbal.drive_speed(pitch_speed=gim_pitch, yaw_speed=gim_yaw)
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, f"[GIMBAL] drive_speed error: {e}")

                time.sleep(0.05)
                continue

            # ── MODE AUTO ────────────────────────────────────────────────────

            if state == "SEARCH":
                if self.has_fresh_pose():
                    if not self._is_locate():
                        self.stop_gimbal()
                        self.stop_chassis()
                        yaw_init = self.get_gimbal_yaw_rel()
                        self._align_since    = time.time()
                        self._align_yaw_init = yaw_init
                        self._align_valid_n  = 0
                        state = "ALIGN"
                        rospy.loginfo(f"[CTRL] State: ALIGN (yaw_rel_init={yaw_init:.1f}deg)")
                    else:
                        # LOCATE : balise visible, on fige la position et on logue
                        self.stop_gimbal()
                        self.stop_chassis()
                        time.sleep(0.2)
                    continue
                # Grace period : attendre avant de commencer a balayer
                if time.time() - self._search_since < SEARCH_GRACE_S:
                    time.sleep(0.1)
                    continue
                # Sweep gimbal (robot immobile)
                result = self.do_gimbal_sweep()
                if result == "FOUND":
                    if not self._is_locate():
                        self.stop_chassis()
                        self.search_steps = 0
                        yaw_init = self.get_gimbal_yaw_rel()
                        self._align_since    = time.time()
                        self._align_yaw_init = yaw_init
                        self._align_valid_n  = 0
                        state = "ALIGN"
                        rospy.loginfo(f"[CTRL] State: ALIGN (yaw_rel_init={yaw_init:.1f}deg)")
                    else:
                        rospy.loginfo_throttle(2.0, "[LOCATE] balise trouvee — position sur map mise a jour")
                elif result == "MANUAL":
                    continue
                elif result == "EDGE":
                    # Limite atteinte : inverser le sens. Deux bords = cycle complet
                    # (gimbal a couvert +/-limite) -> avancer d'un pas en ligne droite.
                    self.search_edges += 1
                    self.search_dir   = -self.search_dir
                    if self.search_edges >= 2:
                        self.search_edges = 0
                        if self._is_locate():
                            # LOCATE : ne jamais avancer, comportement voulu — rien a logger
                            pass
                        elif self.search_steps < SEARCH_MAX_STEPS:
                            # Borne globale de l'episode de recherche (temps ou distance
                            # cumulee) — Perplexity 08 : 90s ou ~13.5m plutot qu'un
                            # nombre d'axes fixe, plus robuste face a la variabilite terrain.
                            elapsed = time.time() - self._search_episode_since
                            if (elapsed >= SEARCH_TOTAL_TIMEOUT_S
                                    or self._search_dist_total_m >= SEARCH_TOTAL_DIST_M):
                                self.stop_chassis()
                                rospy.logwarn_throttle(
                                    10.0,
                                    f"[SEARCH] borne recherche atteinte ({elapsed:.0f}s, "
                                    f"{self._search_dist_total_m:.1f}m parcourus) — "
                                    "arret, sweep sur place (intervention requise)")
                            else:
                                self.do_step_forward()
                                self.search_steps += 1
                                self._search_dist_total_m += SEARCH_STEP_M
                        elif self.search_axis_idx < SEARCH_FAN_AXIS_COUNT - 1:
                            # Axe epuise (SEARCH_MAX_STEPS avancements sans cible) : motif
                            # en eventail -> tourner le chassis vers l'axe suivant plutot
                            # que d'abandonner (Perplexity 08 : increment 60deg, 6 axes).
                            self.search_axis_idx += 1
                            rospy.loginfo(
                                f"[SEARCH] axe {self.search_axis_idx + 1}/{SEARCH_FAN_AXIS_COUNT} "
                                f"epuise sans cible -> rotation {SEARCH_FAN_AXIS_INC_DEG:.0f}deg")
                            self._rotate_body_by(SEARCH_FAN_AXIS_INC_DEG)
                            self.search_steps = 0
                        else:
                            # Tous les axes du motif en eventail epuises sans detection.
                            self.stop_chassis()
                            rospy.logwarn_throttle(
                                10.0,
                                f"[SEARCH] {SEARCH_FAN_AXIS_COUNT} axes du motif en eventail "
                                "epuises — arret, sweep sur place (intervention requise)")

            elif state == "ALIGN":
                # Pose fraiche ?
                if not self.has_fresh_pose():
                    self.stop_chassis()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    self._align_valid_n = 0
                    rospy.loginfo("[CTRL] ALIGN: cible perdue -> SEARCH")
                    continue

                # Timeout dynamique : alpha * |yaw_rel_init| / ALIGN_MAX_WZ, plancher 2s, cap 12s
                timeout = max(2.0, min(
                    ALIGN_TIMEOUT_ALPHA * abs(self._align_yaw_init) / max(ALIGN_MAX_WZ, 1.0),
                    ALIGN_TIMEOUT_MAX_S))
                if time.time() - self._align_since > timeout:
                    self.stop_chassis()
                    self.posebuf.invalidate()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    self._align_valid_n = 0
                    rospy.logwarn(f"[CTRL] ALIGN: timeout ({timeout:.1f}s) -> SEARCH")
                    continue

                yaw_rel = self.get_gimbal_yaw_rel()

                # Commande rotation (deadband evite chattering autour de 0)
                if abs(yaw_rel) < ALIGN_YAW_DEADBAND:
                    wz = 0.0
                else:
                    wz = clamp(K_BODY_YAW * yaw_rel, -ALIGN_MAX_WZ, ALIGN_MAX_WZ)

                # Compter poses consecutives valides quand le yaw_rel est dans le seuil
                p = self.posebuf.get()
                if p is not None and abs(yaw_rel) < ALIGN_YAW_THRESHOLD:
                    depth = abs(p.z)
                    if 0.3 < depth < 2.5:
                        self._align_valid_n += 1
                    else:
                        self._align_valid_n = 0
                else:
                    self._align_valid_n = 0

                rospy.loginfo_throttle(1.0,
                    f"[ALIGN] yaw_rel={yaw_rel:.1f}deg wz={wz:.1f} "
                    f"valides={self._align_valid_n}/{ALIGN_VALID_POSES}")

                # Transition APPROACH apres N poses valides consecutives
                if self._align_valid_n >= ALIGN_VALID_POSES:
                    self.stop_chassis()
                    state = "APPROACH"
                    self._align_valid_n = 0
                    rospy.loginfo(
                        f"[CTRL] ALIGN -> APPROACH (yaw_rel={yaw_rel:.1f}deg, "
                        f"{ALIGN_VALID_POSES} poses valides)")
                    continue

                # Gimbal OFF pendant ALIGN — world-stabilise suffit a garder la balise
                self.stop_gimbal()

                try:
                    self.chassis.drive_speed(x=0.0, y=0.0, z=wz, timeout=1)
                except Exception as e:
                    rospy.logwarn(f"[ALIGN] drive_speed error: {e}")

                time.sleep(0.10)

            elif state == "APPROACH":
                if not self.has_fresh_pose():
                    self.stop_chassis()
                    self.stop_gimbal()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges = 0
                    self.search_steps = 0
                    rospy.loginfo("[CTRL] Lost target -> SEARCH")
                    continue

                p = self.posebuf.get()
                if p is None:
                    continue
                depth = abs(p.z)
                if depth <= 0.05:
                    time.sleep(0.05)
                    continue

                dist_err    = depth - STOP_DISTANCE_M

                if dist_err <= 0.0:
                    self.stop_chassis()
                    self.stop_gimbal()
                    state = "STOP"
                    rospy.loginfo("[CTRL] Reached target -> STOP")
                    continue

                # Gimbal : verrouille la balise au centre (servo sur l'erreur laterale).
                # Le gimbal etant stabilise monde, il garde la cible meme si le corps tourne.
                # Servo partage avec l'auto-suivi MANUEL (_gimbal_servo_yaw).
                gim_yaw = self._gimbal_servo_yaw(p)
                try:
                    self.gimbal.drive_speed(pitch_speed=0, yaw_speed=gim_yaw)
                except Exception:
                    pass

                # Chassis : tourne pour annuler yaw_rel (se realigner derriere la camera)
                # et avance proportionnellement a la distance restante.
                yaw_rel = self.get_gimbal_yaw_rel()
                vx = clamp(K_VX * dist_err, 0.0, MAX_VX)
                if vx < MIN_VX and dist_err > 0.15:
                    vx = MIN_VX
                wz = clamp(K_BODY_YAW * yaw_rel, -MAX_WZ, MAX_WZ)

                rospy.loginfo_throttle(1.0, f"[APPROACH] depth={depth:.3f}m dist_err={dist_err:.3f}m "
                                       f"yaw_rel={yaw_rel:.1f} gim_yaw={gim_yaw:.1f} vx={vx:.3f} wz={wz:.1f}")

                # Arbiter prioritaire : obstacle > servoing visuel
                blocked, reason = self._is_path_blocked()
                if blocked:
                    self.stop_chassis()
                    self.stop_gimbal()
                    self.posebuf.invalidate()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    rospy.logwarn(f"[OBSTACLE] {reason} — APPROACH suspendu -> SEARCH")
                    continue

                try:
                    self.chassis.drive_speed(x=vx, y=0.0, z=wz, timeout=1)
                except Exception as e:
                    rospy.logwarn(f"[APPROACH] drive_speed error: {e}")

                time.sleep(0.10)

            elif state == "STOP":
                self.stop_chassis()
                self.stop_gimbal()
                if not self.has_fresh_pose():
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges = 0
                    self.search_steps = 0
                    rospy.loginfo("[CTRL] Target lost -> SEARCH")
                time.sleep(0.2)

        self.stop_chassis()
        rospy.loginfo("[CTRL] Boucle de controle terminee")

    # ---- boucle caméra (thread principal) ----

    def spin(self):
        ctrl = threading.Thread(target=self._control_loop, daemon=True)
        ctrl.start()

        rospy.loginfo("[CAM] Publication sur /camera/color/image_raw")
        while not rospy.is_shutdown():
            try:
                frame = self.cam.read_cv2_image(strategy="newest")
                if frame is None:
                    self.rate.sleep()
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp    = rospy.Time.now()
                msg.header.frame_id = "camera_link"
                self.pub_img.publish(msg)
            except queue.Empty:
                pass
            except Exception as e:
                rospy.logerr_throttle(2.0, f"[CAM] Erreur : {e}")
            self.rate.sleep()

        self.running = False
        ctrl.join(timeout=2.0)
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("[RM] Arret...")
        self.running = False
        try:
            self.gimbal.unsub_angle()
        except Exception:
            pass
        try:
            self.battery.unsub_battery_info()
        except Exception:
            pass
        try:
            self.chassis.unsub_esc()
        except Exception:
            pass
        try:
            self.chassis.unsub_attitude()
        except Exception:
            pass
        try:
            self.chassis.unsub_position()
        except Exception:
            pass
        try:
            self.chassis.unsub_velocity()
        except Exception:
            pass
        try:
            self.chassis.unsub_status()
        except Exception:
            pass
        try:
            self.ep.sensor.unsub_distance()
        except Exception:
            pass
        try:
            self.cam.stop_video_stream()
        except Exception:
            pass
        try:
            self.ep.close()
        except Exception:
            pass
        rospy.loginfo("[RM] Deconnecte proprement")


if __name__ == "__main__":
    rospy.init_node("ep_camera_beacon_follower")
    node = EPCameraBeaconFollower()
    node.spin()
