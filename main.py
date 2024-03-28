from rplidar import RPLidar
import time
from rpi_hardware_pwm import HardwarePWM
import numpy as np
import math

# Paramètres de contrôle de vitesse
DIRECTION_PROP = 1  # -1 pour les variateurs inversés
PWM_STOP_PROP = 6.55
DELTA_PWM_MAX = 8.5  # PWM à laquelle on atteint la vitesse maximale
POINT_MORT_PROP = 0.13
VITESSE_MAX_HARD = 8  # Vitesse maximale que peut atteindre la voiture
VITESSE_MAX_SOFT = 2  # Vitesse maximale souhaitée

# Paramètres de contrôle de direction
DIRECTION = 1  # 1 pour angle_pwm_min à gauche, -1 pour angle_pwm_min à droite
ANGLE_PWM_MIN = 3  # Min
ANGLE_PWM_MAX = 6  # Max
ANGLE_PWM_CENTER = 4
ANGLE_DEGRE_MAX = 18  # vers la gauche

# Connexion et démarrage du LiDAR
lidar = RPLidar("/dev/ttyUSB0", baudrate=256000)
lidar.connect()
print(lidar.get_info())
lidar.start_motor()
time.sleep(1)

# Initialisation des PWM
pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
pwm_prop.start(PWM_STOP_PROP)

pwm_dir = HardwarePWM(pwm_channel=1, hz=50)
pwm_dir.start(ANGLE_PWM_CENTER)

# Création d'un tableau de 360 zéros pour les données LiDAR
tableau_lidar_mm = [0] * 360

def set_direction_degre(angle_degre) :
    global angle_pwm_min
    global angle_pwm_max
    global angle_pwm_centre
    angle_pwm = angle_pwm_centre + DIRECTION_PROP * (angle_pwm_max - angle_pwm_min) * angle_degre /(2 * ANGLE_DEGRE_MAX )
    if angle_pwm > angle_pwm_max :
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min :
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)

def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > VITESSE_MAX_SOFT :
        vitesse_m_s = VITESSE_MAX_SOFT
    elif vitesse_m_s < -VITESSE_MAX_HARD :
        vitesse_m_s = -VITESSE_MAX_HARD
    if vitesse_m_s == 0 :
        pwm_prop.change_duty_cycle(PWM_STOP_PROP)
    elif vitesse_m_s > 0 :
        vitesse = vitesse_m_s * (DELTA_PWM_MAX)/VITESSE_MAX_HARD
        pwm_prop.change_duty_cycle(PWM_STOP_PROP + DIRECTION_PROP*(POINT_MORT_PROP + vitesse ))
    elif vitesse_m_s < 0 :
        vitesse = vitesse_m_s * (DELTA_PWM_MAX)/VITESSE_MAX_HARD
        pwm_prop.change_duty_cycle(PWM_STOP_PROP - DIRECTION_PROP*(POINT_MORT_PROP - vitesse ))

try:
    for scan in lidar.iter_scans(scan_type='express'):
        # Rangement des données dans le tableau
        for angle, distance in scan:
            tableau_lidar_mm[min(359, max(0, 359 - int(angle)))] = distance

        # Détection d'obstacles et planification de trajectoire
        obstacle_frontal = min(tableau_lidar_mm[0:45] + tableau_lidar_mm[315:360]) < 150
        obstacle_droite = min(tableau_lidar_mm[45:135]) < 150
        obstacle_gauche = min(tableau_lidar_mm[225:315]) < 150

        if obstacle_frontal:
            angle_degre = -90 if obstacle_droite else 90
            set_direction_degre(angle_degre)
            set_vitesse_m_s(-VITESSE_MAX_HARD)
            time.sleep(0.5)
            set_vitesse_m_s(0)
            time.sleep(0.2)
            set_vitesse_m_s(-1)
        elif obstacle_droite:
            set_direction_degre(-ANGLE_DEGRE_MAX)
            set_vitesse_m_s(VITESSE_MAX_SOFT)
            time.sleep(0.5)
        elif obstacle_gauche:
            set_direction_degre(ANGLE_DEGRE_MAX)
            set_vitesse_m_s(VITESSE_MAX_SOFT)
            time.sleep(0.5)
        else:
            # Planification de trajectoire avec la moyenne des distances
            front_left_avg = np.mean(tableau_lidar_mm[315:345])
            front_right_avg = np.mean(tableau_lidar_mm[15:45])
            angle_degre = math.degrees(math.atan((front_left_avg - front_right_avg) / 300))

            set_direction_degre(angle_degre)
            set_vitesse_m_s(VITESSE_MAX_SOFT)

except KeyboardInterrupt:
    print("Fin des acquisitions")

# Arrêt et déconnexion du LiDAR et des moteurs
lidar.stop_motor()
lidar.stop()
time.sleep(1)
lidar.disconnect()
pwm_dir.stop()
pwm_prop.start(PWM_STOP_PROP)


def set_vitesse_m_s(vitesse_m_s):
    if vitesse_m_s > VITESSE_MAX_SOFT:
        vitesse_m_s = VITESSE_MAX_SOFT
    elif vitesse_m_s < -VITESSE_MAX_HARD:
        vitesse_m_s = -VITESSE_MAX_HARD

    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(PWM_STOP_PROP)
    elif vitesse_m_s > 0:
        vitesse = vitesse_m_s * DELTA_PWM_MAX / VITESSE_MAX_HARD
        pwm_prop.change_duty_cycle(PWM_STOP_PROP + DIRECTION_PROP * (POINT_MORT_PROP + vitesse))
    elif vitesse_m_s < 0:
        vitesse = vitesse_m_s * DELTA_PWM_MAX / VITESSE_MAX_HARD
        pwm_prop.change_duty_cycle(PWM_STOP_PROP - DIRECTION_PROP * (POINT_MORT_PROP - vitesse))


def set_direction_degre(angle_degre):
    angle_pwm = ANGLE_PWM_CENTER + DIRECTION * (ANGLE_PWM_MAX - ANGLE_PWM_MIN) * angle_degre / (2 * ANGLE_DEGRE_MAX)
    angle_pwm = min(ANGLE_PWM_MAX, max(ANGLE_PWM_MIN, angle_pwm))
    pwm_dir.change_duty_cycle(angle_pwm)
