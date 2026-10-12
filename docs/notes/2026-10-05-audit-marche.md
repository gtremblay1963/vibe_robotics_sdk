# Audit lecture seule du contrôleur de marche — 2026-10-05

Dépôt : vibe_robotics_sdk, HEAD e98d292 (« Changement du 2026-10-01 »). Analyse faite sans modifier les sources ; rejeu hors ligne dans un bac à sable (pinocchio 4.1.0, mujoco 3.15, sans manette ni moteurs).

## Architecture
- scripts/walking : generate_footsteps.py (pas), fsm.py (STAND/DSP/SSP + MPC), mpc.py (QP OSQP), interp.py (pied en vol, Hermite), robot.py (IK Pinocchio, sim MuJoCo), demo.py (robot réel).
- Générateur de trajectoire en boucle ouverte : LIPM (intégrateur de jerk), 2 QP 1D (avant, latéral), contraintes ZMP, coût terminal seulement. Aucun retour IMU ni articulaire.
- IK : moindres carrés amortis, 13 tâches (2 pieds 6D, COM 3D, lacet de base). Roulis/tangage du torse libres.
- Modèles : IK sur SundayA1_ankle_2dof_new (jambes seules, 2,711 kg) ; simulation sur SundayA1_short_new (25 actionneurs, 2,865 kg) ; robot réel via sundaya1_real_config_short.yaml.

## Constats principaux
1. Hauteur du COM : get_params renvoie une hauteur absolue (-0,002 m), puis reset_standing applique un plancher de 0,15 m. La hauteur jambes tendues du modèle est 0,2055 m au-dessus du repère du pied. L'IK ne peut pas descendre de 55 mm : pieds 18,5 mm trop bas, COM 37 mm trop haut, erreur 0,045 > tol 0,01.
2. Dans le rejeu, ik() ne renvoie jamais success=True. demo.py gèle alors les jambes (« IK unsafe, holding last pose ») ; simulate() ignore le drapeau. À confirmer sur la machine de Gilles.
3. Torse : roulis commandé ±15,6° à chaque pas (aucune tâche d'orientation du torse).
4. _get_targets divise par 2 le déplacement latéral du COM et borne l'avant à [-10, +2] mm : le ZMP du COM réellement commandé sort du pied d'appui sur 137 des 343 ticks de simple appui.
5. Vitesse nominale maximale : 0,03 m / 0,77 s = 3,9 cm/s. Mesuré en simulation : 2,9 cm/s, dérive latérale 3,6 cm en 11 s.
6. Simulation short_new : marche avant stable ; commande latérale pure : chute à 3,4 s ; lacet pur : chute à 2,8 s. Départ avec les pieds 6,8 cm dans le sol (qpos[2]=0,22 contre 0,288), le robot saute de 4 cm.
7. Actionneurs short_new : position kp=50, kd≈2,4, aucune limite de couple ni de vitesse. Couples crêtes en marche : 4,4 N·m (hip_pitch) et 4,0 N·m (ankle_roll), contre 4,9 N·m de couple de blocage STS3250.
8. Accélérations articulaires demandées jusqu'à 54 rad/s² ; réglage servo acc=30 (local) ou 50 (récepteur Pi).
9. Noms d'articulations : « hip_pitch » tourne autour de l'axe avant (abduction) ; « hip_roll » est un axe incliné de 40° (flexion + lacet).
10. Code mort ou cassé : lat_adjust, S_dcm, update_heading inutilisés ; deploy() et deploy_remote() utilisent WalkCommand qui n'existe pas ; doc walking.md en retard sur le code.

## Masses
- short_new : torse 0,919 kg, bras 0,454 kg chacun, jambes 0,519 kg chacune, tête 1e-9 kg (caméra et tête non pesées), lest 395 g dans le torse. Inerties physiquement valides.
- ankle_2dof_new : torse 1,808 kg avec rayon de giration de 23 mm (irréaliste, sans effet sur l'IK).
- COM au-dessus de la semelle : 0,2106 m (IK) contre 0,2262 m (sim).

## À vérifier sur le robot
- Masse totale réelle sur une balance.
- Servos des jambes : STS3250 ou STS3215.
- Message « IK unsafe, holding last pose » dans demo.py.
