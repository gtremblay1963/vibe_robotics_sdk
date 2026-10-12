# Équilibre, retour au cap, pas de rattrapage, chevilles (2026-10-08 au 2026-10-11)

## État actuel des réglages
**`scripts/walking/robot.py`**
- `TORSO_PITCH_TRIM_DEG = 5.0` : l'IK tient le torse penché de 5° vers l'avant, debout et en marche. Sans ça, le torse réel recule de 6 à 8° avec toutes les articulations à 0 (zéro des servos de tangage probablement décalé). Choix de l'utilisateur : environ −2° affiché au repos.
- La pose debout est calculée par l'IK au démarrage (`_init_stand_pose`) **sans** `COM_LAT_TRIM` ; le décalage latéral de 6 mm ne s'applique qu'en marche (corrige un penché à droite au repos introduit puis retiré le 2026-10-11).
- Inchangés : `COM_LAT_SCALE = 0.9`, `COM_LAT_TRIM = 0.006`, `ssp=0.6`, `dsp=0.3`, `step=0.04`.

**`scripts/walking/demo.py`** (tout nécessite `--imu`)
- Pose de référence (tangage, roulis, cap) prise 1,5 s après le Start, une fois le robot redressé.
- **Arrêt de sécurité** : 10° pendant la marche au stick (`SAFETY_TILT_DEG`), 25° debout et pendant les pas de rattrapage (`SAFETY_PUSH_DEG`). Arrêt = sticks ignorés jusqu'au relâchement et retour sous la moitié du seuil.
- **Bouton RB** (robot arrêté) : retour au cap du Start. Mesure l'écart à l'arrêt, fait un nombre de pas calculé (vise 80 %), apprend combien le robot tourne par pas, jusqu'à 5 essais, tolérance 3°, demi-stick au plus. Fonctionne.
- **Pas de rattrapage** (robot arrêté, sans stick) : déclenchement à 7° d'écart de tangage, anticipé avec la vitesse de bascule (`PUSH_LEAD = 0.25 s`, jamais sous 3°), confirmé en 1 cycle. 1 pas rapide (2 si plus de 11°), puis pas de fermeture. Temps rapides pendant ce mode : `dsp 0.08`, `ssp 0.3`, pas de 5 cm ; temps normaux remis à l'arrêt. Période sourde de 1,5 s après chaque rattrapage (supprimait un va-et-vient). Premier pied posé vers 0,4 s après déclenchement, plus le retard des servos. L'utilisateur s'en déclare satisfait ; sujet clos pour l'instant.

**`scripts/run_receiver_log.py`** (Pi) : option `--ankle-kp N` (et `--ankle-kd`, défaut 32) qui règle le gain des chevilles avant-arrière (servos 11 et 23) au démarrage. **Non mémorisé par les servos** : à repasser à chaque lancement. Valeur retenue par l'utilisateur : 64 (« à l'air mieux »).

## Faits mesurés
- Gains d'usine des 12 servos des jambes : P = 32, D = 32.
- Charge des chevilles avant-arrière en marche : 24 % au plus. Elles ne manquent pas de force.
- Pied qui « résiste mais plie comme un ressort » : pas de jeu à rattraper. Cause trouvée par l'utilisateur : le servo de cheville bougeait dans son logement rectangulaire ; recollé.
- Simulation : une correction d'équilibre par les chevilles à l'arrêt ne sauve aucune poussée (retard des servos 0,1 à 0,15 s, bascule en environ 0,3 s). D'où le choix du pas de rattrapage.
- Sur le Pi 4, les ports des servos sont `/dev/serial/by-id/...` (jambes : `usb-1a86_USB_Single_Serial_5B79014957-if00`), pas `/dev/sundaya1_*`.
- Incident : BNO055 ne répondant plus (`Errno 121`) le 2026-10-09. Conduite à tenir : `i2cdetect -y 1`, coupure complète d'alimentation, vérification du câble hors tension. Si récurrent, ralentir le bus I2C (incompatibilité connue BNO055 / Raspberry Pi).

## Pistes ouvertes
- Calibration du zéro à l'équerre, puis ramener `TORSO_PITCH_TRIM_DEG` et `COM_LAT_TRIM` vers 0.
- Rendre `--ankle-kp 64` permanent (valeur par défaut) si l'utilisateur le souhaite.
- Rattrapage latéral (poussée de côté) : non traité.
