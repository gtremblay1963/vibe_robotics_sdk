# Centrale inertielle et Raspberry Pi — 2026-10-07

## État

- Le Raspberry Pi 5 (8 GB) ne démarre plus depuis le premier branchement de la BNO055 : voyant rouge fixe, bouton sans effet, 5 V présent, résistance nulle entre la broche 1 (3,3 V) et la masse. Considéré comme hors service. Cause non établie.
- Remplacé par un Raspberry Pi 4 modèle B avec la même carte SD. Adresse : 192.168.1.25 (l'ancienne 192.168.1.27 n'est plus valide). Marche vérifiée par Gilles en mode distant : tout fonctionne.
- Sur le Pi, le SDK est dans `/root/vibe_robotics_sdk`, environnement `vibe`. Les ports des cartes de servos y sont désignés par numéro de série (`/dev/serial/by-id/...`), donc stables d'un Pi à l'autre.
- Prix relevés chez PiShop.ca le 2026-10-07 : Pi 5 2 GB 110,50 $ CA (en stock), 4 GB 156,85 $ CA (rupture), 8 GB 249,50 $ CA (en stock). Un 2 GB suffit pour le rôle de récepteur.

## Centrale BNO055 (carte Adafruit STEMMA QT) : validée en lecture

- Branchée en I2C direct sur le Pi 4, détectée à l'adresse 0x28.
- Câblage vérifié sur photo : rouge VIN vers broche 1 (3,3 V), noir GND vers broche 9, bleu SCL vers broche 5, jaune SDA vers broche 3. Connecteur à six positions sur la carte, positions 3V et RST vides. Les pastilles de la carte sont dans l'ordre VIN, 3V, GND, SCL, SDA, RST.
- Posée à plat sur le plancher du torse, à l'arrière. Orientation confirmée par mesure : X vers la droite du robot, Y vers l'avant, Z vers le haut.
- Script de test sur le Pi : `scripts/test_bno055.py` (bibliothèque `smbus2`, mode IMU 0x08 sans magnétomètre, lecture du vecteur gravité au registre 0x2E). Convention : tangage = atan2(-gy, gz), positif vers l'avant ; roulis = atan2(-gx, gz), positif vers la droite du robot.
- Mesures de Gilles : +18,3° penché vers l'avant, −20,2° vers l'arrière, +11,7° vers la droite du robot, +13,9° de signe opposé vers la gauche. Lecture à 72–83 Hz, zéro erreur I2C.
- Le code du dépôt (`arduino/bno055.ino` et `viberobotics/sensor/imu_arduino.py`) passe par un Arduino et ne fonctionnerait pas tel quel ; il n'est pas utilisé.

## À faire

1. Mesurer le décalage au repos, robot debout sur ses pieds (la valeur de −7,6° relevée l'était robot suspendu).
2. Faire remonter tangage et roulis du Pi vers le portable pendant la marche (le lien actuel est à sens unique, du portable vers le Pi).
3. Enregistrer ces angles pendant une marche sans correction, pour voir l'oscillation réelle du torse.
4. Seulement ensuite, ajouter la stabilisation du torse dans la marche.
