# Dérive à gauche en marche avant — diagnostic et état (2026-10-07)

## État actuel (à lire en premier, mis à jour le 2026-10-08)
- **Chute et casse.** Lors d'un virage sur place à fond de stick avec `dsp_duration=0.3`, `COM_LAT_SCALE=0.9`, `COM_LAT_TRIM=0.006`, le robot a basculé vers l'arrière (tangage de 6° à 20° en 6 s) et une pièce du pied gauche a cassé. Il tournait de 3° par pas pour 15° demandés, avec un balancement à contretemps (environ 0,4 s de retard). La tenue de cap était inactive pendant le virage : la commande de rotation enregistrée est exactement celle du stick.
- **Réglages en place, à la demande de l'utilisateur** (état juste avant l'ajout de la tenue de cap) : `COM_LAT_SCALE = 0.9`, `COM_FWD_TRIM = 0.0`, `COM_LAT_TRIM = 0.006`, `ssp_duration=0.6`, `dsp_duration=0.3`, `step_length=0.04`. Le code de la tenue de cap est retiré de `demo.py` (version précédente restaurée à l'identique ; copie de la version avec tenue de cap dans le bac à sable `$HOME/work/hold/demo.py`).
- **Attention** : ces réglages sont ceux avec lesquels la chute en virage s'est produite. Validés seulement en marche avant. Ne pas faire de virage sur place à fond de stick avec eux.
- Dernier état où les quatre mouvements fonctionnaient : `COM_LAT_SCALE = 0.8`, `COM_LAT_TRIM = 0.0`, `dsp_duration=0.15` (dérive à gauche en marche avant).
- **À faire au remontage** : calibration du zéro à l'équerre ; revalider les quatre mouvements à demi-stick, robot retenu par le cordon, avant tout essai à fond et après tout changement de réglage.
- Sens confirmé : stick de rotation à droite → `cap` diminue.

## Symptôme
En marche avant, stick de rotation neutre, le robot tournait à gauche (jusqu'à 4° par pas). Le torse pivotait de 10 à 16° à chaque pas, dans un sens puis dans l'autre.

## Outils de mesure mis en place
- **Cap** : `scripts/imu_sender.py` (Pi) envoie un 7e champ, le cap cumulé (deg, + = gauche), tiré du quaternion de la BNO055. `imu_remote.py` et `demo.py` l'enregistrent (colonne `cap` de `imu_log_*.csv`). Paquet UDP `'<7d'` : les deux côtés doivent être à jour ensemble.
- **Servos des jambes** : `scripts/run_receiver_log.py` (Pi), à lancer à la place de `run_receiver.py`. Écrit `legs_log_*.csv` : consigne, position, vitesse, charge des 12 servos à chaque tick. Option `--acc` (défaut 50).
- **Analyse** (bac à sable du portable, `$HOME/work/legs/`) : `an.py` (retard, erreur, charge par servo), `tilt.py` (aligne `legs_log` et `imu_log` de même horodatage ; compare le roulis du torse déduit des angles réels des servos à celui de la centrale).
- **Tenue de cap** (retirée du dépôt, conservée dans `$HOME/work/hold/demo.py`) : stick de rotation neutre, ramène le cap moyen sur un cycle (2 pas) à sa valeur de départ. Proportionnelle, plafonnée à 0,3. Jamais essayée sur le robot.

## Faits établis
- Commande de rotation nulle, consignes symétriques, pied commandé à plat et levé de 4 cm.
- Les servos gauche et droit suivent de façon identique (amplitude 95–100 %, charge max 45 %). Retard 93 à 155 ms, identique avec `--acc 50` et `--acc 150` : l'accélération n'y est pour rien.
- **Les genoux ne plient pas** (1° d'amplitude). Le pied se lève uniquement en penchant le corps (plan : ±16° de roulis). Tibia de 5 cm : une marche genoux fléchis demanderait >300 °/s au genou ; piste abandonnée.
- Le torse ne penche réellement que de 3 à 8° alors que les servos font 12° : le pied d'appui bascule sur sa tranche de 5 à 10°, le pied « levé » frotte, le robot pivote.
- Asymétrie : le pas du pied gauche faisait +7 à +10° de rotation, le pas du pied droit −3°. Balance : 1,33 kg sous le pied gauche, 1,43 kg sous le droit. D'après les angles réels, le pied gauche part environ 4 mm plus bas. Hypothèse retenue : pose zéro légèrement de travers (calibration à l'œil).
- **Confirmation** : avec `COM_LAT_TRIM = 0.007` (décalage de 7 mm vers la droite), pas gauche +2,9°, pas droit −3,8°, dérive nette −11° puis −3°. À 0,006 : dérive −1° et −2° sur 10 à 12 pas. L'utilisateur : « il va droit », puis « encore un peu à gauche » (niveau du bruit).
- `dsp_duration` 0,15 → 0,3 : gain marginal en marche avant (rotation du pas gauche 8,9° → 7,0°), mais virage sur place dégradé jusqu'à la chute.

## Ce qui reste
- Va-et-vient du cap de 9 à 11° par cycle même quand la dérive nette est nulle : les pieds frottent toujours.
- Tangage qui recule jusqu'à 14° par moments en marche avant.
- Virage sur place à rendre sûr avec les réglages actuels, ou réglages séparés pour le virage.
- Correction de fond de l'asymétrie : calibration du zéro à l'équerre, puis réévaluer `COM_LAT_TRIM`.
- Protection proposée : arrêt de la marche si la centrale mesure plus d'une quinzaine de degrés d'inclinaison.

## Leçons
- La simulation MuJoCo, même avec le retard des servos modélisé, a prédit à tort que `dsp_duration=0.3` suffirait et n'a pas prévu la chute en virage. Ne pas s'y fier pour valider un réglage sur le robot réel.
- Un réglage changé pour la marche avant doit être revalidé dans les quatre mouvements, à demi-stick, avant tout essai à fond.
- Pistes écartées par la mesure : câble, adhérence des semelles, `COM_LAT_SCALE` seul, accélération des servos, servo faible à droite, centre de gravité décalé à gauche.
- Toujours vérifier l'alignement temporel de deux enregistrements (corrélation > 0,99) avant d'attribuer un effet à un pied.
