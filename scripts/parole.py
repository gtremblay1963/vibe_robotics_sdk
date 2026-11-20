"""Parole de Victor (a lancer sur le Pi) : synthese vocale en francais dans le haut-parleur du robot.

Voix quebecoise neuronale de Microsoft (edge-tts : acces gratuit NON officiel, Internet requis, ~1 s).
Chaque phrase dite est gardee en cache (~/.victor/voix) : « Oui ? » et les phrases deja dites partent tout de suite.
Sans Internet : voix locale Piper (accent de France) si elle est installee, sinon espeak-ng, sinon rien.

  python scripts/parole.py --liste                    # sorties son vues par ALSA
  python scripts/parole.py --essai                    # chaque voix d'homme fr-CA dit la meme phrase
  python scripts/parole.py "Bonjour, je suis Victor"  # dire une phrase
  python scripts/parole.py --voix fr-CA-JeanNeural --hp plughw:CARD=XXX,DEV=0 "Allo"

Dependances sur le Pi : pip install edge-tts ; apt install mpg123
Secours hors ligne (facultatif) : pip install piper-tts, et le modele fr_FR-tom-medium (.onnx et .onnx.json)
dans /root/piper/
"""
import argparse
import asyncio
import hashlib
import os
import queue
import re
import shutil
import subprocess
import threading
import time
import wave

VOIX = 'fr-CA-AntoineNeural'
VOIX_HOMMES = ['fr-CA-AntoineNeural', 'fr-CA-JeanNeural', 'fr-CA-ThierryNeural']
CACHE = os.path.expanduser('~/.victor/voix')
PIPER = '/root/piper/fr_FR-tom-medium.onnx'
DELAI_NUAGE = 6.0        # s, au-dela on passe a la voix locale
HORS_LIGNE = 60.0        # s sans reessayer le nuage apres un echec
ECHO = 0.4               # s pendant lesquelles le micro reste sourd apres la fin d'une phrase


def trouver_hp():
    """Premiere sortie son USB vue par « aplay -l » -> 'plughw:CARD=<nom>,DEV=0' ; sinon 'default'."""
    try:
        sortie = subprocess.run(['aplay', '-l'], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return 'default'
    for ligne in sortie.splitlines():
        m = re.match(r'card \d+: (\S+) \[', ligne)
        if m and 'usb' in ligne.lower():
            return f'plughw:CARD={m.group(1)},DEV=0'
    return 'default'


class Parole:
    """dire() met une phrase dans la file ; un fil la synthetise et la joue. Ne bloque jamais l'appelant."""

    def __init__(self, hp='auto', voix=VOIX):
        self.hp = trouver_hp() if hp == 'auto' else hp
        self.voix = voix
        self.file = queue.Queue()
        self._verrou = threading.Lock()
        self._sourds = 0             # phrases en attente ou en cours pendant lesquelles le micro est sourd
        self._fin_echo = 0.0
        self._hors_ligne = 0.0
        self._piper = None
        os.makedirs(CACHE, exist_ok=True)
        manque = '' if shutil.which('mpg123') else '  (mpg123 absent : apt install mpg123)'
        print(f'[parole] haut-parleur {self.hp}, voix {voix}{manque}')
        threading.Thread(target=self._boucle, daemon=True).start()

    def dire(self, texte, sourd=True, si_libre=False):
        """sourd : le micro n'ecoute pas pendant la phrase (Victor ne s'entend pas lui-meme).
        si_libre : ne rien dire si Victor parle deja."""
        texte = ' '.join((texte or '').split())
        if not texte or (si_libre and self.parle()):
            return
        with self._verrou:
            self._sourds += sourd
        self.file.put((texte, sourd))

    def parle(self):
        return self.file.unfinished_tasks > 0

    def sourd(self):
        """Vrai pendant une phrase « sourde » (et un court moment apres, pour l'echo)."""
        return self._sourds > 0 or time.monotonic() < self._fin_echo

    def prechauffer(self, *textes):
        """Prepare des phrases en cache, en arriere-plan, pour qu'elles partent sans delai."""
        threading.Thread(target=lambda: [self._fichier(t) for t in textes], daemon=True).start()

    def attendre(self):
        self.file.join()

    def _boucle(self):
        while True:
            texte, sourd = self.file.get()
            try:
                print(f'[parole] « {texte} »')
                f = self._fichier(texte)
                if f:
                    self._jouer(f)
            except Exception as e:                     # la parole ne doit jamais arreter l'ecoute
                print(f'[parole] erreur ({type(e).__name__}: {e})')
            finally:
                with self._verrou:
                    if sourd:
                        self._sourds -= 1
                        self._fin_echo = time.monotonic() + ECHO
                self.file.task_done()

    def _fichier(self, texte):
        cle = hashlib.sha1(f'{self.voix}|{texte}'.encode()).hexdigest()[:20]
        mp3 = os.path.join(CACHE, cle + '.mp3')
        if os.path.exists(mp3) and os.path.getsize(mp3) > 0:
            return mp3
        if time.monotonic() >= self._hors_ligne:
            tmp = f'{mp3}.{threading.get_ident()}.tmp'
            try:
                import edge_tts
                asyncio.run(asyncio.wait_for(edge_tts.Communicate(texte, self.voix).save(tmp), DELAI_NUAGE))
                if os.path.getsize(tmp) > 0:
                    os.replace(tmp, mp3)
                    return mp3
            except Exception as e:
                self._hors_ligne = time.monotonic() + HORS_LIGNE
                print(f'[parole] voix du nuage indisponible ({type(e).__name__}) : voix locale')
            finally:
                if os.path.exists(tmp):
                    os.remove(tmp)
        return self._local(texte, cle)

    def _local(self, texte, cle):
        wav = os.path.join(CACHE, f'local_{cle}.wav')
        if os.path.exists(wav) and os.path.getsize(wav) > 44:
            return wav
        try:
            if os.path.exists(PIPER):
                from piper import PiperVoice
                if self._piper is None:
                    self._piper = PiperVoice.load(PIPER)
                with wave.open(wav, 'wb') as w:
                    if hasattr(self._piper, 'synthesize_wav'):     # piper-tts 1.3 et plus
                        self._piper.synthesize_wav(texte, w)
                    else:
                        self._piper.synthesize(texte, w)
                return wav
            if shutil.which('espeak-ng'):
                subprocess.run(['espeak-ng', '-v', 'fr', '-w', wav, texte], timeout=15, check=True)
                return wav
        except Exception as e:
            print(f'[parole] voix locale en erreur ({type(e).__name__}: {e})')
        if os.path.exists(wav):
            os.remove(wav)
        print('[parole] aucune voix disponible (Internet coupe, et ni Piper ni espeak-ng)')
        return None

    def _jouer(self, f):
        if f.endswith('.mp3'):
            cmd = ['mpg123', '-q', '-o', 'alsa', '-a', self.hp, f]
        else:
            cmd = ['aplay', '-q', '-D', self.hp, f]
        r = subprocess.run(cmd, timeout=60, stderr=subprocess.PIPE, text=True)
        if r.returncode:
            print(f'[parole] lecture impossible sur {self.hp} : {r.stderr.strip()[:200]}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('texte', nargs='*', help='phrase a dire')
    ap.add_argument('--hp', default='auto', help="sortie ALSA, ex. plughw:CARD=XXX,DEV=0 (defaut : la sortie USB)")
    ap.add_argument('--voix', default=VOIX, help=f'voix edge-tts (defaut {VOIX})')
    ap.add_argument('--liste', action='store_true', help='lister les sorties son (aplay -l) et quitter')
    ap.add_argument('--essai', action='store_true', help="faire dire une phrase a chaque voix d'homme fr-CA")
    args = ap.parse_args()
    if args.liste:
        subprocess.run(['aplay', '-l'])
        print(f'Sortie choisie automatiquement : {trouver_hp()}')
        return
    if args.essai:
        for v in VOIX_HOMMES:
            p = Parole(args.hp, v)
            p.dire(f"Salut, moi c'est Victor ! Ici la voix {v.split('-')[2][:-6]}. Je vais bien, merci. "
                   "J'avance de trois pas.")
            p.attendre()
            time.sleep(0.8)
        return
    p = Parole(args.hp, args.voix)
    t0 = time.monotonic()
    p.dire(' '.join(args.texte) or 'Oui ?')
    p.attendre()
    print(f'[parole] {time.monotonic() - t0:.1f} s')


if __name__ == '__main__':
    main()
