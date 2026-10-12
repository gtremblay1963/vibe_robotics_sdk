# Modifications du contrôleur de marche — 2026-10-05

Suite de `audit-marche-2026-10-05.md`. Matériel confirmé par Gilles : environ 3 kg, 25 servos STS3215 12 V (2,94 N·m de couple de blocage). Rien n'est commité dans git.

## État au 2026-10-05 en fin de séance

Robot réel, selon Gilles : marche avant nettement meilleure, déplacement de côté bon, rotation « parfaite ». IP du Raspberry Pi : 192.168.1.27.

## Appliqué

Dans `scripts/walking/` :

1. `robot.py`, `get_params` : hauteur du COM = hauteur au-dessus des pieds (0,2055 m) au lieu du plancher de 0,15 m.
2. `robot.py`, `safe_leg_q` : ne bloque plus les jambes quand `success` est faux (butées et limite de 0,08 rad/tick conservées).
3. Nouveau `zmp_preview.py` : classe `WalkingFSMPreview`, suivi de ZMP par preview control à la place des deux QP de `fsm.py`. Horizon 1,5 s, 0,1 ms par tick.
4. `robot.py` : utilise `WalkingFSMPreview` ; tangage du torse tenu à zéro dans l'IK ; constantes `COM_LAT_SCALE = 0.8` et `COM_FWD_TRIM = 0.0` à la place du demi-balancement et de la borne avant.
5. Réglages par défaut : pas 0,04 m, simple appui 0,6 s, double appui 0,15 s (avant : 0,03 / 0,7 / 0,07).
6. `zmp_preview.py` : `steering_deg = 15.0` (rotation par pas à fond de stick, 5° avant).

Dans `viberobotics/configs/sundaya1_real_config_short.yaml` :

7. `sign_change` du moteur 11 (tangage de la cheville droite) passé de 1 à −1. C'était la cause de la chute vers l'arrière sur le robot réel. Modifié sur le portable par Claude, puis reporté à la main par Gilles dans la copie du Pi.

Retour à l'ancien planificateur : remettre `WalkingFSM` dans `Robot._init_walking`.

## Mesures en simulation (short_new, 3 kg, couple limité à 2,94 N·m, 12 s)

- Avant les modifications 3 à 5 : 1,6 cm/s en avant, torse basculé jusqu'à 5,2° vers l'arrière, pied levé de 11 mm.
- Après : 3,5 cm/s avant, 3,4 cm/s arrière, tangage sous 1,2°, roulis ±6°, aucune chute dans les quatre directions.
- Avec pas 0,05 m, simple appui 0,5 s, `COM_LAT_SCALE` 1,0 : 6,2 cm/s, roulis ±15°, pied levé de 40 mm.
- Rotation pure en 11 s : 46° à 5°/pas, 96° à 10°/pas, 150° à 15°/pas, sans chute ; lacet de hanche max 0,36 rad à 15°/pas (butée 0,785).

## Constats utiles

- L'ancien MPC devient insoluble (plantage) dès que le pas passe à 0,04 m ou que le simple appui descend à 0,6 s.
- Torse droit avec jambes fléchies : abandonné (genoux qui changent de sens, charge trop forte pour des STS3215).
- La boucle réelle est passée de 23–29 Hz à 33 Hz avec le nouveau planificateur : le retard venait de l'ancien MPC.
- Diagnostic du signe de cheville : en simulation, inverser un tangage de cheville donne un torse basculé de 15° vers l'arrière en marche avant et un pas de côté normal. Test réel : consigne q[11]=+0,2 et q[23]=−0,2 doit lever les deux pointes de pied ; la droite descendait.
- Le dépôt contient les deux signes pour la cheville gauche selon la config (−1 dans `short`, +1 dans `short_mini` et `half_2dof`) : vérifier les signes par ce test avant d'utiliser une autre config.

## Reste à faire

- Commit git des modifications (supprimer d'abord `.git/index.lock`, laissé par Claude le 5 octobre).
- `COM_LAT_SCALE` : baisser à 0,7 si chute vers l'extérieur du pied d'appui ; monter vers 1,0 si les pieds traînent.
- Vitesse : essayer pas 0,05 m et simple appui 0,5 s, par étapes, sur le robot réel.
- L'IK ne passe la tolérance que 54 à 77 % du temps ; non corrigé.
