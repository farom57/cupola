# TODO

## À vérifier sur le matériel

- [ ] Vérifier si le firmware actuellement flashé sur l'Arduino répond à la commande `k` (et `p`). Ouvrir un moniteur série à 1 000 000 bauds et envoyer `?`, `k`, `k1`, `k0`, `p`. Si oui, le bon firmware n'est pas celui du dépôt : le retrouver et le commiter.
- [ ] Mesurer le nombre réel de pas du codeur sur un tour complet de coupole (48 ? 692 ? 4320 ?).

## Incohérences firmware ↔ client Python

- [ ] Commandes `k` / `k0` / `k1` (flag de suivi) et `p` (sorties) envoyées par `python/serial_com.py` mais absentes de `cupola/cupola.cpp`. Le bouton « Suivi » ne peut donc jamais recevoir de réponse.
- [ ] Nombre de pas par tour : `TOTAL_STEPS = 48` dans le firmware (`cupola.cpp:24`) contre `STEPS_PER_TURN = 692` côté Python (`serial_com.py:6`). Une cible au-delà de 47 n'est jamais atteinte.
- [ ] Commande `t<N>` : le firmware répond d'abord `delta:<d>`, puis la cible. `Cupola.goto()` ne lit que la première ligne et reçoit donc une réponse non numérique.
- [ ] `t0` est traité comme un stop par le firmware (`tmp > 0`). La position 0 ne peut donc pas être demandée comme cible.
- [ ] Commande `i` : la variable `int i` n'est pas initialisée (`cupola.cpp:218`), donc le masque renvoyé contient des valeurs indéterminées.

## Améliorations

- [ ] Port série fixé en dur (`port = 'COM6'` dans `python/main.py`). Le rendre configurable ou le détecter automatiquement.
