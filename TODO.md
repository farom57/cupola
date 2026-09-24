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
- [x] Le client Python ouvrait le port à 9600 bauds (`self.baudrate` n'était jamais appliqué à `self.ser`) au lieu de 1 000 000. Corrigé dans `Cupola.connect()`.

## Cimier : contacts tournants et ouverture en un clic

Les 6 contacts entre la partie fixe et la partie tournante ne se touchent que dans une position précise de la coupole. Seuls les contacts 1-2 sont câblés aujourd'hui (+12 V pour ouvrir, -12 V pour fermer), car on a la version motorisée et non robotisée. Voir `../Doc/Uniwersal_origineel_schema.pdf` : p3 pour le brochage des contacts (« HOME sensor contacts »), p4 pour la logique des fins de course SW1 à SW4.

- [ ] Câbler les contacts 3 à 6 sur la partie tournante (matériel).
- [ ] Contacts 5-6 : poser un strap pour détecter la position de contact et s'en servir comme position de référence (home) de l'azimut.
- [ ] Contacts 3-4 : relier les fins de course du cimier pour détecter s'il est ouvert ou fermé. Le firmware déclare déjà `IN_OPEN` (A4) et `IN_CLOSED` (A5), mais ne s'en sert pas. L'état n'est lisible qu'en position de contact.
- [ ] Ouverture et fermeture en un clic, au lieu de devoir placer la coupole à la main puis garder le bouton appuyé. Enchaînement : rotation jusqu'à la position de contact, commande ouvrir ou fermer jusqu'au fin de course (avec un timeout de sécurité, et arrêt sur n'importe quel bouton), puis éventuellement retour à l'azimut de suivi.

## Améliorations

- [x] Port série fixé en dur (`port = 'COM6'` dans `python/main.py`) : désormais configurable via le bouton « Config » et sauvegardé dans `%APPDATA%\cupola\config.json`.
- [ ] Flèches ↑ ← → ↓ des boutons presque invisibles sous Linux (police de remplacement d'Arial trop fine). Vérifier sous Windows ; augmenter la taille ou changer de police pour ces boutons.
- [ ] Console inondée quand rien n'est connecté (« Connexion impossible » et « [PWI4] Erreur » chaque seconde). N'afficher un message qu'au changement d'état (perte / rétablissement de la connexion).
