# Résolution transports en commun + marche (mode 1)

Pour chaque requête d'une instance `instances/lehavre_rural/`, calcule les meilleurs trajets en transports en
commun (tram et bus LiA, horaires GTFS réels) et à pied, puis donne des statistiques.

```bash
# environnement de creation_instance_le_havre (osmnx, scipy, pandas)
PY=../creation_instance_le_havre/.venv/bin/python
$PY resoudre_tc.py -t scolaire -i 30_30_0        # une instance
$PY resoudre_tc.py -t samedi                     # toutes les instances du dossier
```

`-t` : `scolaire` (lundi 16/11/2026), `vacances` (lundi 26/10/2026), `samedi` (samedi 14/11/2026).

Il faut avoir lancé `creation_instance_le_havre/02_preparation.py` au préalable, pour disposer du réseau piéton.
Au premier lancement, les correspondances à pied entre arrêts sont calculées puis mises en cache dans `cache/`.

## Méthode

- McRAPTOR ([mcraptor.py](mcraptor.py)) sur les horaires du jour ([reseau.py](reseau.py)). Il calcule
  l'ensemble de Pareto des trajets selon quatre critères : heure de départ du domicile (la plus tardive),
  heure d'arrivée, temps de marche et nombre de véhicules.
- La fenêtre de temps est respectée et le MRT n'est pas imposé :
  - fenêtre de départ : la personne quitte son origine dans [h, h+15] ;
  - fenêtre d'arrivée : la personne arrive dans [h−15, h]. Si elle arrive plus tôt, elle attend, et
    l'attente compte dans le temps de trajet.
- On retient trois trajets par requête :
  - `temps` : temps porte à porte minimal, puis le moins de correspondances, puis le moins de marche ;
  - `marche` : marche minimale, puis temps, puis correspondances ;
  - `transferts` : correspondances minimales, puis temps, puis marche.

Les paramètres sont dans [config_tc.py](config_tc.py) :
- 30 min de marche maximum entre l'origine (ou la destination) et un arrêt ;
- 10 min de marche maximum pour une correspondance ;
- 1 min de correspondance minimum ;
- 5 véhicules maximum ;
- 3 h de trajet maximum.

## Résultats (`results/lehavre_rural_tc/<type>/`)

- `itineraires_<instance>.csv` : pour chaque requête et chaque critère, l'heure de départ, l'heure
  d'arrivée, le temps de trajet, le ratio au trajet direct en voiture, la marche, les correspondances et
  le détail de l'itinéraire.
- `statistiques_<…>.csv` : moyenne, médiane, minimum et maximum par critère, sur les requêtes faisables.
