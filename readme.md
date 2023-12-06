# Démonstrations vidéos

Toutes les démonstrations vidéos des différentes fonctionnalités (simulation/robot-physique) sont disponibles au [lien suivant sur Google Drive](https://drive.google.com/drive/folders/1FRc0I4VLxESOACXQvV_2FOeH5X6hW15U?usp=sharing).

# Application Web

Le code de l'application Web se trouve dans le dossier "webApp" et est organisé en deux parties principales :

## Front-end
Le front-end est codé en TypeScript en utilisant le framework Angular. Les pratiques de codage suivantes sont respectées :
- Utilisation de la convention camelCase pour les noms de variables et de fonctions.
- Utilisation de chemins absolus pour les importations.
- Utilisation de directives et de composants Angular pour une structure modulaire et réutilisable.

## Back-end
L'ajout de tout nouveau package nécessaire au fonctionnement du serveur Flask existe dans le fichier `/back-end/requirements.txt`. Ce fichier répertorie toutes les dépendances essentielles pour exécuter le serveur. Maintenir cette liste à jour garantit l'installation correcte de toutes les dépendances requises lors du déploiement du serveur Flask, assurant ainsi la cohérence de l'environnement d'exécution.

Le back-end est codé en Python en utilisant le framework Flask. Il est organisé en dossiers de routes, de services et de contrôleurs. Les pratiques de codage suivantes sont appliquées :
- Utilisation de la convention snake_case pour les noms de variables et de fonctions.
- Gestion appropriée des exceptions et des erreurs pour des réponses HTTP claires.

## Robot-physique

Tous les fichiers nécessaires au fonctionnement des robots sont situés dans le répertoire suivant :
```bash
./INF3995-104/embedded/agilex_ws
```

Pour démarrer le robot, exécutez la commande suivante au moment du démarrage :
```bash
./INF3995-104/start_script.sh
```

## Fonctionnement
L'interface utilisateur affiche quatre options principales :
- Lancer un robot physique
- Lancer une simulation
- Identifier un robot
- Consulter l'historique des missions
- Mettre à jour le code des robots physiques

À chaque fois qu'un opérateur sélectionne une option, la demande est redirigée vers le serveur, soit sous la forme d'une requête HTTP, soit par le biais d'une communication par socket. Le serveur prend ensuite en charge la demande de manière appropriée. Selon que la simulation a été choisie dans l'interface utilisateur, le serveur redirige les demandes vers les robots physiques ou vers la simulation dans Gazebo.

## Tests unitaires
Des tests unitaires ont été rédigés pour chaque fonctionnalité du back-end et du front-end. Pour les exécuter, des commandes spécifiques doivent être utilisées :

- Côté serveur : Des tests ont été élaborés pour les fichiers principaux .py, en excluant les fichiers de configuration.

```bash
cd webApp/back-end
```
```bash
python -m pytest 
```

- front-end side: 

```bash
cd webApp/front-end
```
```bash
npm run test
```
ou pour avoir le coverage:
```bash
npm run coverage
```

# Docker : Tout lancer en une commande

Exécutez `docker compose up` à l'intérieur de ce répertoire. Cela créera le serveur frontend sur le port 4100 disponible sur le réseaux, le serveur backend sur le port 8000 en localhost, et la simulation gazebo localhost, port 9090.

Pour avoir accès a l'application depuis d'autres appareils sur le réseaux, veuillez a modifier le fichier `/front-end/src/environments/environment.prod.ts` et mettre l'adresse IP local de la station au sol.
