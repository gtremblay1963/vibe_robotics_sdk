"""Ecoute vocale de Victor (a lancer sur le Pi) : micro -> Vosk + Google -> page de commande du portable.

Vosk (local) guette « Victor » et « stop ». Apres « Victor », le son de la commande est enregistre
jusqu'a la fin de la phrase, puis transcrit par Google en francais canadien (fr-CA) : bien meilleur
avec l'accent. Sans Internet (ou avec --sans-google), le texte de Vosk sert de secours.

  python ecoute.py --liste                       # micros vus par ALSA
  python ecoute.py --micro plughw:2,0 --test     # affiche ce qui est entendu, n'envoie rien
  python ecoute.py --micro plughw:2,0 --host 192.168.1.XX

Regles :
  « Victor, avance de 3 pas »  -> la phrase apres « Victor » part vers POST /api/phrase du portable.
  « Victor » ... (pause) ... « avance de 3 pas » -> la phrase suivante (dans les 6 s) est la commande.
  « stop », « arrete », « halte » : arret immediat (POST /api/stop), meme sans « Victor »,
  des la reconnaissance partielle, sans attendre la fin de la phrase.
  Une pause de moins de 0,8 s au milieu de la commande ne la coupe pas.

Dependances sur le Pi : pip install vosk SpeechRecognition ; apt install flac
(Google : acces gratuit non officiel de la bibliotheque SpeechRecognition, peut cesser sans preavis.)
"""
import argparse
import collections
import json
import re
import subprocess
import sys
import threading
import time
import urllib.request

RATE = 16000
BLOC = 4000                      # octets lus a la fois (0,125 s de son)
ATTENTE = 6.0                    # s pendant lesquelles Victor ecoute apres « Victor » seul
FIN_PHRASE = 0.8                 # s de silence apres la fin d'une phrase avant d'envoyer la commande
AVANT = 1.0                      # s de son gardees avant « Victor » (le debut du mot)
MAX_COMMANDE = 12.0              # s, duree maximale d'une commande
GOOGLE_DELAI = 5.0               # s, au-dela on se rabat sur le texte de Vosk
# Variantes entendues par Vosk et Google : victor, victors, victores, viktor, vyktor, victory, victoria...
MOT = re.compile(r"\b(v[iy]c?k?tor\w*(?:'s)?|victoire)\b", re.I)
STOP = re.compile(r"\b(stop|arr[eê]te|arr[eê]tez|halte)\b", re.I)
# Arret immediat sur la reconnaissance partielle : « stop » seulement. Vosk entend « arrete » pour « droite »
# et « halte » pour « deux bras » (journal du 2026-11-01) : ces mots n'arretent que s'ils sont dits seuls,
# ou s'ils sont dans la transcription de Google.
STOP_IMMEDIAT = re.compile(r"\bstop\b", re.I)
MOTS_STOP = {'stop', 'arrête', 'arrete', 'arrêtez', 'arretez', 'halte'}


def seul_stop(texte):
    mots = texte.lower().split()
    return bool(mots) and all(m in MOTS_STOP for m in mots)

# Vocabulaire limite : Vosk ne choisit que parmi ces mots (beaucoup plus fiable que la dictee libre,
# surtout pour « Victor » et l'accent quebecois). Un mot hors liste devient [unk] (ignore).
# Ajouter ici les mots des nouvelles commandes et les noms des poses.
VOCABULAIRE = """
victor stop arrête arrêtez halte attends attendre
avance avancer avancez recule reculer reculez tourne tourner tournez marche va fais
pas mètre mètres centimètres degrés degré seconde secondes tour demi demi-tour côté
à droite gauche vers sur en de du des la le les l' un une et puis ensuite encore
deux trois quatre cinq six sept huit neuf dix onze douze quinze vingt trente quarante
cinquante soixante quatre-vingt quatre-vingts quatre-vingt-dix cent
bras main mains pince pinces coude lève levez baisse baissez ouvre ouvrez ferme fermez plie tends
ton ta tes droit les deux avant arrière devant haut bas moitié pour cent
pose accroupi accroupis penche debout salut bonjour signe
"""


# Le « de » quebecois est entendu « deux » : « avance de trois pas » -> « avance deux trois pas ».
# Un « deux » suivi d'un autre nombre est donc un « de ».
NOMBRE = (r"(?:un|une|deux|trois|quatre|cinq|six|sept|huit|neuf|dix|onze|douze|quinze|vingt|trente|quarante"
          r"|cinquante|soixante|quatre-vingts?|quatre-vingt-dix|cent)")
DE_DEUX = re.compile(rf"\bdeux (?={NOMBRE}\b)")


QUEBEC = [(re.compile(r"\btoune\b", re.I), 'tourne'), (re.compile(r"\btounes\b", re.I), 'tournes')]


def corriger(texte):
    texte = DE_DEUX.sub('de ', texte)
    for motif, mot in QUEBEC:
        texte = motif.sub(mot, texte)
    return texte


def apres_victor(texte):
    """Ce qui suit le dernier « Victor » ; tout le texte s'il n'y en a pas."""
    m = None
    for m in MOT.finditer(texte):
        pass
    return texte[m.end():].strip(' ,.!?:\'') if m else texte.strip()


def poster(base, chemin, donnees, test):
    if test:
        print(f'   (test) {chemin} {donnees}')
        return None
    req = urllib.request.Request(base + chemin, data=json.dumps(donnees).encode(),
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return json.loads(r.read() or b'{}')
    except OSError as e:
        print(f'   ERREUR : portable injoignable ({e}). demo.py --commandes tourne-t-il ? Pare-feu Windows (port 8091) ?')
        return None


def transcrire_google(son):
    """Son 16 kHz mono 16 bits -> hypotheses de Google (fr-CA), la plus probable d'abord ;
    [] si rien compris, None si Google est injoignable."""
    import speech_recognition as sr
    r = sr.Recognizer()
    r.operation_timeout = GOOGLE_DELAI
    try:
        brut = r.recognize_google(sr.AudioData(son, RATE, 2), language='fr-CA', show_all=True)
    except sr.UnknownValueError:
        return []
    except (sr.RequestError, OSError) as e:
        print(f'   Google injoignable ({e}) : texte de Vosk utilise')
        return None
    if not isinstance(brut, dict):
        return []
    return [a['transcript'] for a in brut.get('alternative', []) if a.get('transcript')]


def traiter_commande(base, son, texte_vosk, google, test):
    """Transcrit (Google, sinon Vosk), puis envoie la commande au portable. Tourne dans un fil a part."""
    t0 = time.monotonic()
    hyp = transcrire_google(son) if google else None
    source = 'google'
    if hyp is None:
        hyp, source = [texte_vosk], 'vosk'
    texte = hyp[0] if hyp else ''
    commande = corriger(apres_victor(texte))
    autres = []                                  # autres hypotheses, transmises a l'IA qui choisit
    for h in hyp[1:]:
        c = corriger(apres_victor(h))
        if c and c != commande and c not in autres:
            autres.append(c)
    print(f'[ecoute] {source} ({time.monotonic() - t0:.1f} s) : « {texte} » -> commande : « {commande} »')
    for c in autres[:4]:
        print(f'[ecoute]    autre hypothese : « {c} »')
    if not commande:
        print('[ecoute] commande vide, rien envoye')
        return
    if (STOP.search(commande) if source == 'google' else seul_stop(commande)):
        poster(base, '/api/stop', {}, test)
        return
    vosk = corriger(texte_vosk) if source == 'google' else ''    # 2e avis : Vosk ne connait que les mots du robot
    r = poster(base, '/api/phrase', {'texte': commande, 'alternatives': autres[:4], 'vosk': vosk}, test)
    if r:
        print(f"   {r.get('reponse', '')}  actions : {r.get('actions')}  refus : {r.get('refus')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--liste', action='store_true', help='lister les micros (arecord -l) et quitter')
    ap.add_argument('--micro', default='default', help='peripherique ALSA, ex. plughw:2,0 (voir --liste)')
    ap.add_argument('--modele', default='/root/vosk-model-small-fr-0.22', help='dossier du modele Vosk francais')
    ap.add_argument('--host', default=None, help='adresse IP du portable (demo.py --commandes)')
    ap.add_argument('--port', type=int, default=8091)
    ap.add_argument('--test', action='store_true', help="afficher ce qui est entendu sans rien envoyer")
    ap.add_argument('--libre', action='store_true', help='dictee libre pour Vosk (sans vocabulaire limite)')
    ap.add_argument('--sans-google', action='store_true', help='Vosk seulement (hors ligne)')
    args = ap.parse_args()

    if args.liste:
        subprocess.run(['arecord', '-l'])
        return
    if not args.test and not args.host:
        sys.exit('--host (adresse du portable) est requis, ou --test')
    base = f'http://{args.host}:{args.port}'

    from vosk import Model, KaldiRecognizer, SetLogLevel
    SetLogLevel(-1)
    if args.libre:
        rec = KaldiRecognizer(Model(args.modele), RATE)
    else:
        mots = sorted(set(VOCABULAIRE.split())) + ['[unk]']
        rec = KaldiRecognizer(Model(args.modele), RATE, json.dumps(mots, ensure_ascii=False))
    # arecord convertit le son du micro en 16 kHz mono 16 bits (plughw:)
    son = subprocess.Popen(['arecord', '-q', '-D', args.micro, '-f', 'S16_LE', '-r', str(RATE), '-c', '1', '-t', 'raw'],
                           stdout=subprocess.PIPE)
    print(f"[ecoute] micro {args.micro}, modele {args.modele}" + (' (TEST)' if args.test else f', vers {base}'))
    print("[ecoute] dites « Victor, ... »  (Ctrl+C pour quitter)")

    bloc_s = BLOC / (2.0 * RATE)                  # duree d'un bloc (s)
    avant = collections.deque(maxlen=int(AVANT / bloc_s))
    t = 0.0                      # temps du son (s)
    cap = None                   # commande en cours d'enregistrement : {'son', 'textes', 'debut', 'fin'}
    stop_envoye = False          # un seul STOP par phrase

    def commencer(par_partiel):
        print("[ecoute] Victor entendu, j'ecoute...")
        # « We! » joue par le portable ; dans un fil a part pour ne pas retarder l'ecoute
        threading.Thread(target=poster, args=(base, '/api/eveil', {}, args.test), daemon=True).start()
        return {'son': list(avant), 'textes': [], 'debut': t, 'fin': None, 'partiel': par_partiel}

    def reste(c):
        """Texte de la commande (sans le mot d'eveil). Si l'ecoute a demarre sur un « victor » partiel que Vosk
        a ensuite ecrit autrement (« ta », « les ta »...), ce premier morceau EST le mot d'eveil : on l'ignore."""
        tout = ' '.join(c['textes'])
        if MOT.search(tout):
            return apres_victor(tout)
        return ' '.join(c['textes'][1:]) if c['partiel'] else tout

    try:
        while True:
            bloc = son.stdout.read(BLOC)
            if not bloc:
                sys.exit("[ecoute] le micro ne donne plus de son (arecord arrete). Verifiez --micro.")
            t += bloc_s
            avant.append(bloc)
            if cap is not None:
                cap['son'].append(bloc)
            final = rec.AcceptWaveform(bloc)
            if not final:
                partiel = json.loads(rec.PartialResult()).get('partial', '')
                if partiel and not stop_envoye and STOP_IMMEDIAT.search(partiel):
                    print(f'[ecoute] STOP entendu (« {partiel} »)')
                    poster(base, '/api/stop', {}, args.test)
                    stop_envoye, cap = True, None
                elif partiel and cap is None and MOT.search(partiel):
                    cap = commencer(True)
                elif partiel and cap is not None:
                    cap['fin'] = None                 # on parle encore
            else:
                texte = ' '.join(w for w in json.loads(rec.Result()).get('text', '').split() if w != '[unk]')
                deja_stop, stop_envoye = stop_envoye, False
                if texte:
                    print(f'[ecoute] vosk : « {texte} »')
                    if seul_stop(texte):                 # « stop », « arrete », « halte » dits seuls
                        if not deja_stop:
                            print(f'[ecoute] STOP entendu (« {texte} »)')
                            poster(base, '/api/stop', {}, args.test)
                        cap = None
                        continue
                    if cap is None and MOT.search(texte):
                        cap = commencer(False)
                    if cap is not None:
                        cap['textes'].append(texte)
                        cap['fin'] = t + (FIN_PHRASE if reste(cap) else ATTENTE)   # « Victor » seul : on attend la suite
            if cap is not None and ((cap['fin'] is not None and t >= cap['fin']) or t - cap['debut'] > MAX_COMMANDE):
                if reste(cap) or not args.sans_google:
                    threading.Thread(target=traiter_commande, daemon=True,
                                     args=(base, b''.join(cap['son']), reste(cap),
                                           not args.sans_google, args.test)).start()
                cap = None
    except KeyboardInterrupt:
        pass
    finally:
        son.terminate()


if __name__ == '__main__':
    main()
