# Calibration du zéro des servos — 12 octobre 2026

## Outil
- `scripts/jog_servo.py` (à lancer sur le Pi, sans demo.py ni récepteur) : choisir un servo, `+`/`-`/`+N` (1 pas = 0,088°, max 50 par commande), `a` = jambes à 2048, `b` = bras à 2048, `p` = tableau du groupe choisi + tangage/roulis (si imu_sender tourne), `z` = position actuelle devient le zéro DANS le servo (journal `calibration_log.txt` sur le Pi).
- Cale imprimée : `docs/gabarit/cale_pieds_102_2mm.stl` (semelle 84,6 mm, entraxe 186,8 mm). Les STL 106,5 / 108,5 / 110,5 sont faux.

## Numéros des servos
- Jambes : 7–12 droite, 19–24 gauche (hanche ×3, genou 10/22, cheville avant-arrière 11/23, cheville côté 12/24).
- Bras : 1–6 droite, 13–18 gauche (épaule avant-arrière, épaule écartement, rotation bras, coude, rotation poignet, pince). Tête : 25.
- Pinces 6/18 : zéro = butée dans le modèle, ne pas pousser au-delà.

## Résultats jambes
- Servo 9 (rotation hanche D) : −8 pas (pied droit tourné vers la droite, 3 mm à la pointe).
- Servo 10 (genou D) : +65 pas (5,7°). Servo 11 (cheville D) : +36 pas (3,2°).
- Servo 20 (écartement G) : +10 pas. Servos 22 et 23 : −5 et −3 pas.
- Correction presque entièrement sur la jambe droite (défaut d'assemblage probable de ce côté). Vérifié pieds dans le vide.
- Tangage du torse au zéro : −5,5° → −2,3° (−2,7° après calibration des bras).
- 2 mm d'écartement des pieds / forme en A : laissé tel quel (sous la précision de la référence, roulis ≈ 0).

## Bras
- Référence (modèle, tout à 0) : bras pendant à la verticale, coude tendu, épaule/coude/poignet alignés de face et de profil. Tous calibrés le 12 oct.

## Code
- `robot.py` : `TORSO_PITCH_TRIM_DEG` 5.0 → 2.0 (objectif tangage ≈ −2° à l'arrêt).
- `robot.py` : `COM_LAT_TRIM` 0.006 → 0.003 → 0.0. Le décalage latéral n'avait presque plus d'effet après calibration.

## Rotation en marche avant/arrière (après calibration)
- Le robot tournait à droite en avant et à gauche en arrière, ~1–3°/pas (cap de la centrale), quel que soit `COM_LAT_TRIM` (6, 3, 0 mm).
- Journaux des jambes : la rotation de hanche suit la consigne à 0,1–0,3° près ; angle entre les pieds ±0,9° selon le sens (souplesse des servos), trop faible pour expliquer le cap. Conclusion : pivot des pieds sur le plancher de bois verni.
- Solution retenue : semelles TPU 95A collées sous les pieds (`docs/gabarit/semelle_tpu_1_2mm.stl`, 76,2 × 171,2 × 1,2 mm, coins r 15,5, chanfrein 0,6 mm côté sol ; même pièce pour les deux pieds). Résultat : « pratiquement parfait », aucune compensation dans le code.
- En rotation sur place, la rotation de hanche réelle dépasse la consigne d'environ 30 % au moment de la pose (9,7° pour 7,2°) — noté, non traité.
