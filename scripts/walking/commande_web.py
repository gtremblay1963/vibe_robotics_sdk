"""Page web de commande de Victor : http://localhost:8091 (ou l'adresse du portable).
On y tape une phrase ; la meme entree servira a la voix (POST /api/phrase depuis le Pi)."""
import json
import os
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from actions import valider, decrire
from comprendre import nettoyer

# « encore », « refais-le », « recommence »... dits seuls : la derniere commande est refaite, sans passer par l'IA.
ENCORE = re.compile(r"(encore|refais|refais[- ]le|refais[- ]ca|recommence|r[eé]p[eè]te|encore une fois|"
                    r"une autre fois|une fois de plus)( une fois| encore)?[ .!]*", re.I)

# Son joue quand le Pi entend « Victor » (cree une fois avec la synthese vocale de Windows).
SON_EVEIL = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'eveil.wav')


def jouer_eveil():
    """Joue eveil.wav sans bloquer (Windows) ; sinon un bip, ou rien hors Windows."""
    try:
        import winsound
    except ImportError:
        print('[victor] (eveil)')
        return
    if os.path.exists(SON_EVEIL):
        winsound.PlaySound(SON_EVEIL, winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
    else:
        winsound.MessageBeep()

PAGE = """<!doctype html><html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Victor</title>
<style>
 :root{--bg:#f4f5f7;--card:#fff;--txt:#1d2330;--mut:#6b7280;--line:#e3e6eb;--acc:#2563eb;--bad:#dc2626}
 @media (prefers-color-scheme:dark){:root{--bg:#14171c;--card:#1d2128;--txt:#e8eaee;--mut:#9aa1ad;--line:#2c323b;--acc:#5b8def;--bad:#f87171}}
 body{margin:0;font:16px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--txt)}
 main{max-width:900px;margin:0 auto;padding:16px}
 .row{display:flex;gap:8px;flex-wrap:wrap}
 input{flex:1 1 300px;font:inherit;padding:12px;border-radius:10px;border:1px solid var(--line);background:var(--card);color:var(--txt)}
 button{font:inherit;padding:12px 18px;border-radius:10px;border:0;background:var(--acc);color:#fff;cursor:pointer}
 #stop{background:var(--bad);font-weight:700}
 section{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;margin-top:14px}
 .mut{color:var(--mut);font-size:14px} pre{white-space:pre-wrap;margin:0;font:13px ui-monospace,monospace}
</style></head><body><main>
<h1 style="font-size:20px">Victor</h1>
<div class="row"><input id="t" placeholder="Ex. : avance de 5 pas, puis tourne à droite" autofocus>
<button onclick="envoyer()">Envoyer</button><button id="stop" onclick="stop()">STOP</button></div>
<section><b>Réponse</b><div id="rep" class="mut">—</div></section>
<section><b>En cours</b> <span id="cur" class="mut">rien</span><div id="file" class="mut"></div></section>
<section><b>Journal</b><pre id="log"></pre></section>
<p class="mut">Le stick de la manette ou STOP annule tout. Échap = STOP.</p>
</main><script>
const $=i=>document.getElementById(i);
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});return r.json();}
async function envoyer(){const t=$('t').value.trim(); if(!t) return; $('rep').textContent='…';
 const j=await post('/api/phrase',{texte:t}); $('t').value='';
 $('rep').textContent=(j.reponse||'')+(j.refus&&j.refus.length?'  (refusé : '+j.refus.join(' ; ')+')':'')+'  ['+j.source+']';}
function stop(){post('/api/stop');}
$('t').addEventListener('keydown',e=>{if(e.key==='Enter')envoyer();});
document.addEventListener('keydown',e=>{if(e.key==='Escape')stop();});
setInterval(async()=>{try{const j=await (await fetch('/api/etat')).json();
 $('cur').textContent=j.en_cours||'rien'; $('file').textContent=j.file.length?'ensuite : '+j.file.join(' → '):'';
 $('log').textContent=j.journal.slice().reverse().join('\\n');}catch(e){}},500);
</script></body></html>"""


def demarrer(executeur, comprehension, port=8091):
    derniere = {'actions': [], 'reponse': ''}       # derniere commande acceptee, pour « encore »

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def reply(self, body, ctype='application/json'):
            data = body.encode() if isinstance(body, str) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/api/etat':
                self.reply(executeur.etat())
            else:
                self.reply(PAGE, 'text/html')

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            try:
                p = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                p = {}
            if self.path == '/api/eveil':
                jouer_eveil()
                return self.reply({'ok': True})
            if self.path == '/api/stop':
                executeur.arreter('bouton STOP')
                return self.reply({'ok': True})
            if self.path == '/api/phrase':
                texte = str(p.get('texte', ''))[:300]
                executeur.log(f'phrase : « {texte} »')
                autres = [str(x)[:300] for x in (p.get('alternatives') or [])][:4]   # hypotheses de la voix
                vosk = str(p.get('vosk') or '')[:300]
                if autres:
                    executeur.log('  autres hypotheses : ' + ' | '.join(autres))
                if ENCORE.fullmatch(nettoyer(texte)):
                    if not derniere['actions']:
                        return self.reply({'actions': [], 'refus': [], 'reponse': 'Je n\'ai rien a refaire.',
                                           'source': 'local'})
                    executeur.ajouter([dict(a) for a in derniere['actions']])
                    return self.reply({'actions': [decrire(a) for a in derniere['actions']], 'refus': [],
                                       'reponse': 'Encore une fois !', 'source': 'local'})
                acts, rep, src = comprehension.comprendre(texte, sorted(executeur.poses), autres, vosk)
                ok, refus = [], []
                for a in acts:
                    v, err = valider(a, executeur.poses)
                    (ok.append(v) if v else refus.append(err))
                if ok:
                    executeur.ajouter(ok)
                    derniere['actions'] = [dict(a) for a in ok]
                return self.reply({'actions': [decrire(a) for a in ok], 'refus': refus, 'reponse': rep, 'source': src})
            self.reply({'ok': False})

    srv = ThreadingHTTPServer(('0.0.0.0', port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'[victor] commandes : http://localhost:{port}')
    return srv
