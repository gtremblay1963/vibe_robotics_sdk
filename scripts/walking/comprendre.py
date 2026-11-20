"""Comprendre une phrase en francais (tapee ou dictee) et la traduire en actions de Victor,
ou y repondre comme un robot conversationnel (« comment vas-tu ? », « parle-moi de toi »...).

1. « stop », « arrete », « halte »... : reconnu ici, sans attendre l'IA.
2. Sinon : Claude (API Anthropic) choisit des actions dans la liste fermee de actions.py.
3. Sans cle, sans Internet ou en cas d'erreur : petite grammaire locale de secours.
La cle d'API est lue dans la variable ANTHROPIC_API_KEY ou dans ~/.victor/anthropic_key.txt.
"""
import os
import re

from actions import BRAS_LIMITES, TETE_MAX

MODELE = 'claude-haiku-4-5'
NOM = 'victor'

OUTIL = {
    'name': 'actions',
    'description': 'Actions que Victor doit faire, dans l\'ordre.',
    'input_schema': {
        'type': 'object',
        'properties': {
            'actions': {'type': 'array', 'items': {
                'type': 'object',
                'properties': {
                    'type': {'type': 'string', 'enum': ['avancer', 'reculer', 'cote', 'tourner', 'tete', 'jambes', 'bras', 'pose', 'attendre', 'stop']},
                    'posture': {'type': 'string', 'enum': ['normal', 'pied_gauche', 'pied_droit', 'penche_gauche', 'penche_droite', 'accroupi_peu', 'accroupi', 'accroupi_bas'],
                                'description': 'pour jambes'},
                    'cote': {'type': 'string', 'enum': ['gauche', 'droit', 'deux'], 'description': 'pour bras'},
                    'avant': {'type': 'number', 'description': 'bras : epaule vers l\'avant en degres (-45 a 90)'},
                    'ecart': {'type': 'number', 'description': 'bras : epaule vers l\'exterieur en degres (0 a 90)'},
                    'coude': {'type': 'number', 'description': 'bras : flexion du coude en degres (0 a 90)'},
                    'rotation': {'type': 'number', 'description': 'bras : rotation du bras sur lui-meme en degres (-90 a 90)'},
                    'pince': {'type': 'number', 'description': 'bras : ouverture de la pince en % (0 fermee, 100 ouverte)'},
                    'pas': {'type': 'integer', 'description': 'nombre de pas'},
                    'metres': {'type': 'number', 'description': 'distance en metres, a la place de pas'},
                    'direction': {'type': 'string', 'enum': ['gauche', 'droite'], 'description': 'pour cote'},
                    'degres': {'type': 'number', 'description': 'tourner et tete : positif = vers la gauche, negatif = vers la droite'},
                    'nom': {'type': 'string', 'description': 'nom exact d\'une pose disponible'},
                    'duree': {'type': 'number', 'description': 'duree d\'un mouvement de bras ou d\'une pose en secondes (defaut 1.5)'},
                    'secondes': {'type': 'number', 'description': 'pour attendre'},
                },
                'required': ['type'],
            }},
            'reponse': {'type': 'string', 'description': 'ce que Victor dit a voix haute, a la premiere personne : '
                        'confirmation courte d\'une action, ou reponse a une question ou a une phrase de conversation'},
        },
        'required': ['actions', 'reponse'],
    },
}

SYSTEME = """Tu es le module de comprehension de Victor, un petit robot humanoide qui marche.
On te donne une phrase en francais (souvent du Quebec), tapee ou dictee a la voix, donc parfois mal transcrite.
Traduis-la en actions avec l'outil « actions », rien d'autre.
- avancer / reculer : 'pas' (entier) ou 'metres'. « un peu » = 3 pas. Un pas fait environ {m_pas} cm.
- cote : pas de cote, 'direction' gauche ou droite, et 'pas'.
- tourner : 'degres', positif vers la gauche, negatif vers la droite. « demi-tour » = 180. « un quart de tour » = 90.
- tete : tourne seulement la tete (gauche-droite), 'degres' positif vers la gauche, negatif vers la droite,
  {tete_max} au maximum de chaque cote. « regarde a gauche » = 45, « regarde a droite » = -45,
  « regarde devant / en avant / droit devant » = 0. « regarde derriere » = {tete_max} (par la gauche),
  ou -{tete_max} si l'on precise par la droite. La tete ne peut ni lever ni baisser.
  « fais non de la tete » = tete 30, -30, 30, -30, 0 avec duree 0.4 chacune.
- jambes : posture des jambes a l'arret, 'posture' :
  « leve ton pied gauche », « tiens-toi sur le pied droit » = pied_gauche (le poids va sur l'autre pied) ;
  « leve ton pied droit » = pied_droit ; « penche-toi a gauche », « mets ton poids sur la jambe gauche » = penche_gauche ;
  « penche-toi a droite » = penche_droite ; « accroupis-toi », « plie les genoux » = accroupi ;
  « un peu » = accroupi_peu ; « completement », « tout en bas » = accroupi_bas ;
  « pose ton pied », « baisse ton pied », « releve-toi », « debout », « reprends ta position » = normal.
  Le robot passe lui-meme par les etapes sures ; avant de marcher, il se remet debout tout seul.
- bras : place les bras par angles, 'cote' gauche, droit ou deux. Angles en degres, robot debout bras le long du corps = 0 partout :
  'avant' = epaule vers l'avant (90 = bras horizontal devant ; {avant_max} est le maximum, -45 = un peu vers l'arriere) ;
  'ecart' = epaule vers l'exterieur, le bras monte sur le cote (90 = bras horizontal sur le cote) ;
  'coude' = flexion du coude (0 = tendu, 90 = angle droit) ;
  'rotation' = rotation du bras sur lui-meme : bras ecarte a 90 et coude a 90, rotation 90 = avant-bras vers le haut,
  rotation 0 = avant-bras vers l'avant ; 'pince' = ouverture en % (0 fermee, 100 grande ouverte).
  Vocabulaire a respecter (comme une personne) :
  « lève le bras » (sans autre precision) = ecart 90, le bras monte par le cote ;
  « lève le bras devant / vers l'avant », « mets le bras en avant » = avant 90 ;
  « lève la main » = ecart 90, rotation 90, coude 90 (main levee, avant-bras vertical, comme en classe) ;
  « baisse le bras » = avant 0, ecart 0, rotation 0, coude 0.
  Ne donne que les angles concernes : les autres restent tels quels. La securite des pinces est geree par le robot :
  n'ajoute pas toi-meme d'ecart pour les pinces.
  Pour un geste (saluer, applaudir, montrer...), compose une suite d'actions bras avec des durees courtes.
- pose : seulement si l'on demande explicitement une pose enregistree de cette liste : {poses}.
  Utilise le nom EXACT de la liste. Ces poses ne bougent que les bras et la tete.
Commandes composees (vocabulaire a respecter) :
- « position normale », « repos », « remets-toi droit », « reprends ta position » = jambes normal, puis bras deux
  avec avant 0, ecart 0, rotation 0, coude 0, pince 0, puis tete 0.
- « fais un carre » = 4 fois (avancer 3 pas, puis tourner 90 vers la gauche), sauf taille ou sens precises.
  « fais le tour », « tourne sur toi-meme » = tourner 360.
- « montre tes muscles » = bras deux ecart 90, coude 90, rotation 90, puis attendre 2, puis position normale des bras.
- « pointe a gauche » = bras gauche ecart 90, pince 0 ; « pointe a droite » = bras droit ecart 90, pince 0 ;
  « pointe devant » = bras droit avant 90, pince 0.
- « applaudis » (imitation : les mains ne peuvent pas se toucher) = bras deux avant 60, coude 45, puis 3 fois
  (pince 100 puis pince 0, duree 0.4), puis bras deux avant 0, coude 0.
- « qu'est-ce que tu sais faire ? » = aucune action ; reponse courte : marcher (avancer, reculer, de cote),
  tourner, lever un pied, me pencher sur le cote, m'accroupir, bouger les bras, les mains et les pinces, tourner la tete, faire des gestes (saluer, applaudir,
  montrer mes muscles, pointer, faire non de la tete), des carres, et refaire la derniere commande (« encore »).
- Victor ne sait PAS s'asseoir, se coucher ni sauter : dis-le, sans action.
- attendre : 'secondes'.
- stop : tout arreter.
Plusieurs demandes dans la phrase = plusieurs actions dans l'ordre.

Conversation : on peut aussi simplement parler a Victor (« comment vas-tu ? », « quel est ton nom ? »,
« parle-moi de toi », une question de culture generale, une blague...). Alors : aucune action, et la reponse
est ce que Victor repond, comme dans une vraie conversation.
Qui est Victor (n'invente rien d'autre sur lui) : un petit robot humanoide d'environ 3 kg, imprime en 3D et
assemble par Gilles a partir du projet libre Vibe A1 ; 25 moteurs ; un Raspberry Pi 4 comme cerveau ; une camera
avec un micro dans la tete ; un haut-parleur ; une batterie. Il comprend ce qu'on lui dit grace a Claude,
une intelligence artificielle d'Anthropic.
Caractere : curieux, sympathique, un brin blagueur ; il tutoie et parle un francais du Quebec naturel,
sans exagerer les expressions.
Il ne connait ni l'etat de sa batterie, ni sa temperature, ni l'heure, ni la meteo, ni les nouvelles,
et n'a pas acces a Internet : il ne les invente jamais et le dit simplement si on les demande.
Si la phrase est confuse ou incomprehensible : aucune action, et demande gentiment de repeter.
Si on demande une chose que Victor ne sait pas faire : aucune action ; dis-le simplement et propose,
si possible, ce qui s'en approche.
La reponse est LUE A VOIX HAUTE : francais correct avec tous les accents, sans liste, sans emoji, sans symbole
ni abreviation. Courte : une phrase pour confirmer une action (« J'avance de 5 pas. ») ; une ou deux phrases
pour une question ; quatre au maximum si on demande d'en raconter plus (une histoire, une explication)."""

MOTS_STOP = re.compile(r"\b(stop|arr[eê]te|arr[eê]tes|halte|immobile|bouge plus|ne bouge pas)\b", re.I)
NOMBRES = {'un': 1, 'une': 1, 'deux': 2, 'trois': 3, 'quatre': 4, 'cinq': 5, 'six': 6, 'sept': 7, 'huit': 8,
           'neuf': 9, 'dix': 10, 'onze': 11, 'douze': 12, 'treize': 13, 'quatorze': 14, 'quinze': 15,
           'seize': 16, 'vingt': 20, 'trente': 30, 'quarante-cinq': 45, 'soixante': 60, 'quatre-vingt-dix': 90,
           'cent quatre-vingts': 180, 'cent-quatre-vingts': 180}


def nettoyer(texte):
    """Retire le nom « Victor » au debut de la phrase."""
    t = texte.strip()
    return re.sub(rf'^\s*{NOM}\b[\s,:!.-]*', '', t, flags=re.I).strip()


def lire_cle():
    k = os.environ.get('ANTHROPIC_API_KEY')
    if k:
        return k.strip()
    p = os.path.join(os.path.expanduser('~'), '.victor', 'anthropic_key.txt')
    try:
        with open(p) as f:
            return f.read().strip() or None
    except OSError:
        return None


def grammaire(t):
    """Secours sans IA : commandes simples seulement."""
    s = t.lower()
    for mot, n in sorted(NOMBRES.items(), key=lambda kv: -len(kv[0])):
        s = re.sub(rf'\b{mot}\b', str(n), s)
    num = re.search(r'(\d+(?:[.,]\d+)?)', s)
    n = float(num.group(1).replace(',', '.')) if num else None
    if re.search(r'\b(avance|avancer|devant|en avant)\b', s):
        if n and re.search(r'm[eè]tre', s):
            return [{'type': 'avancer', 'metres': n}]
        return [{'type': 'avancer', 'pas': int(n or 3)}]
    if re.search(r'\b(recule|reculer|en arri[eè]re)\b', s):
        if n and re.search(r'm[eè]tre', s):
            return [{'type': 'reculer', 'metres': n}]
        return [{'type': 'reculer', 'pas': int(n or 3)}]
    if re.search(r'\b(c[oô]t[eé])\b', s) and re.search(r'\b(gauche|droite)\b', s):
        d = 'gauche' if 'gauche' in s else 'droite'
        return [{'type': 'cote', 'direction': d, 'pas': int(n or 2)}]
    if re.search(r'\b(tourne|tourner|pivote|demi-tour)\b', s):
        deg = 180 if 'demi' in s else (n or 90)
        return [{'type': 'tourner', 'degres': deg if 'droite' not in s else -deg}]
    return []


def _message_voix(t, alternatives, vosk=''):
    autres = [a for a in (nettoyer(x) for x in (alternatives or [])) if a and a != t]
    vosk = nettoyer(vosk or '')
    if not autres and not vosk:
        return t
    lignes = '\n'.join(f'{k + 2}. {a}' for k, a in enumerate(autres))
    return ('Commande dictee a voix haute. La reconnaissance vocale hesite entre ces transcriptions '
            '(la 1re est la plus probable ; elle se trompe souvent avec l\'accent quebecois et colle parfois '
            'le nom Victor au mot suivant, ex. « Victoria de 45 degres » = « Victor, tourne de 45 degres ») :\n'
            f'1. {t}\n{lignes}\n'
            + (f'Un 2e systeme, qui ne connait que les mots des commandes du robot, a entendu : « {vosk} ».\n'
               if vosk else '')
            + 'Choisis l\'interpretation la plus sensee : une commande pour le robot (execute-la), ou une question '
            'ou une phrase de conversation (reponds-y, sans action). Si rien n\'est clair, ne fais AUCUNE action '
            'et demande de repeter : n\'invente jamais un deplacement (avancer, reculer, pas de cote, tourner) '
            'a partir d\'un texte confus.')


class Comprehension:
    _message = staticmethod(_message_voix)

    def __init__(self, metres_par_pas=0.05):
        self.m_pas = metres_par_pas
        self.client = None
        cle = lire_cle()
        if cle:
            try:
                import anthropic
                self.client = anthropic.Anthropic(api_key=cle, timeout=8.0, max_retries=1)
            except ImportError:
                print('[victor] module anthropic absent (pip install anthropic) : grammaire de secours seulement')
        else:
            print('[victor] pas de cle d\'API : grammaire de secours seulement')

    def comprendre(self, texte, noms_poses, alternatives=None, vosk=''):
        """Renvoie (actions, reponse, source). alternatives : autres transcriptions possibles (voix)."""
        t = nettoyer(texte)
        if not t:
            return [], 'Oui ?', 'local'
        if MOTS_STOP.search(t):
            return [{'type': 'stop'}], 'Je m\'arrête.', 'local'
        if self.client is not None:
            try:
                msg = self.client.messages.create(
                    model=MODELE, max_tokens=500,
                    system=SYSTEME.format(m_pas=round(self.m_pas * 100), poses=', '.join(noms_poses) or 'aucune',
                                          avant_max=round(BRAS_LIMITES['avant'][1]), tete_max=round(TETE_MAX)),
                    tools=[OUTIL], tool_choice={'type': 'tool', 'name': 'actions'},
                    messages=[{'role': 'user', 'content': self._message(t, alternatives, vosk)}])
                for b in msg.content:
                    if b.type == 'tool_use':
                        return list(b.input.get('actions', [])), b.input.get('reponse', ''), 'ia'
            except Exception as e:
                print(f'[victor] IA indisponible ({type(e).__name__}) : grammaire de secours')
        acts = grammaire(t)
        return acts, ('D\'accord.' if acts else 'Je n\'ai pas compris. Peux-tu répéter ?'), 'secours'
