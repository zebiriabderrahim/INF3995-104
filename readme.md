# Application Web

Le code de l'application web se trouve dans le dossier "webApp" et est organisé en deux parties principales :

## Front-end
Le front-end est codé en TypeScript en utilisant le framework Angular. Les pratiques de codage suivantes sont respectées :
- Utilisation de la convention camelCase pour les noms de variables et de fonctions.
- Utilisation de chemins absolus pour les importations.
- Utilisation de directives et de composants Angular pour une structure modulaire et réutilisable.

## Back-end
Le back-end est codé en Python en utilisant le framework Flask. Il est organisé en dossiers de routes, de services et de contrôleurs. Les pratiques de codage suivantes sont appliquées :
- Utilisation de la convention snake_case pour les noms de variables et de fonctions.
- Gestion appropriée des exceptions et des erreurs pour des réponses HTTP claires.

## Fonctionnement
L'interface utilisateur affiche quatre options principales :
- Lancer un robot
- Identifier un robot
- Consulter l'historique des missions
- Lancer une simulation
À chaque fois qu'un opérateur sélectionne une option, la demande est redirigée vers le serveur, soit sous la forme d'une requête HTTP, soit par le biais d'une communication par socket. Le serveur prend ensuite en charge la demande de manière appropriée. Selon que la simulation a été choisie dans l'interface utilisateur, le serveur redirige les demandes vers les robots physiques ou vers la simulation dans Gazebo.

# Guide d'utilisation de la Simulation Limo Gazebo pour le PDR
Le code de la simulation se trouve dans le dossier "embedded".
Ce guide vous explique comment utiliser la simulation Limo Gazebo dans votre espace de travail Catkin (`gz_sim_ws`). Suivez ces étapes pour lancer la simulation.

## Étape 1 : Accéder au Répertoire de la Simulation

```bash
cd /INF3995-104/embedded/gz_sim_ws
```
## Étape 2 : Compiler la Simulation
```bash
catkin_make
```
## Étape 3 : Sourcer le Setup
```
source devel/setup.bash
```
## Étape 4 : Lancer la Simulation et le serveur rosbridge
```
roslaunch limo_gazebo_sim sim_rosbrige_ser.launch
```
