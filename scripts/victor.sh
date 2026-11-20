#!/usr/bin/env bash
# Lance Victor sur le Pi dans UNE seule session SSH : centrale, recepteur et ecoute vocale.
#   bash scripts/victor.sh <IP du portable> [options du recepteur, ex. --bras-couple 60]
#   VOIX=0 bash scripts/victor.sh <IP>      # sans l'ecoute vocale
#   PAROLE=0 bash scripts/victor.sh <IP>    # ecoute, mais Victor ne parle pas
#   HP=plughw:CARD=XXX,DEV=0 bash ...       # haut-parleur (defaut : la sortie USB, voir python scripts/parole.py --liste)
# Ctrl+C arrete les trois programmes. Les lignes sont prefixees [centrale], [recepteur], [voix].
# Ensuite, sur le portable : python scripts/walking/demo.py --remote --host 192.168.1.25 --imu --commandes
set -u
cd "$(dirname "$0")/.."
PORTABLE="${1:?Usage : bash scripts/victor.sh <IP du portable> [options du recepteur]}"
shift
MICRO="${MICRO:-plughw:CARD=Webcam,DEV=0}"
VOIX="${VOIX:-1}"
PAROLE="${PAROLE:-1}"
HP="${HP:-auto}"

if ! python -c "import viberobotics" 2>/dev/null; then
    echo "[victor] Python ne trouve pas le SDK : faites d'abord  conda activate vibe"
    exit 1
fi

# Anciennes copies encore en marche (sinon : « Address already in use »)
ANCIENS="python.*(imu_sender|run_receiver|ecoute|pose_server)[^ ]*[.]py"
if pkill -f "$ANCIENS"; then
    echo "[victor] arret des anciennes copies..."
    for _ in $(seq 50); do pgrep -f "$ANCIENS" > /dev/null || break; sleep 0.1; done
fi

# Ctrl+C (ou fin d'un programme) : on arrete tout le groupe
trap 'trap - INT TERM EXIT; echo; echo "[victor] arret de tous les programmes"; kill 0' INT TERM EXIT

python -u scripts/imu_sender.py 2>&1 | sed -u 's/^/[centrale] /' &
sleep 2
python -u scripts/run_receiver_log.py "$@" 2>&1 | sed -u 's/^/[recepteur] /' &
if [ "$VOIX" = "1" ]; then
    MUET=""
    [ "$PAROLE" = "1" ] || MUET="--muet"
    python -u scripts/ecoute.py --micro "$MICRO" --hp "$HP" $MUET --host "$PORTABLE" 2>&1 | sed -u 's/^/[voix] /' &
fi
echo "[victor] lance. Demarrez demo.py sur le portable. Ctrl+C ici pour tout arreter."
wait -n
echo "[victor] un des programmes s'est arrete"
