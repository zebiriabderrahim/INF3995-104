# Autonomous SLAM System for AgileX LIMO Rover

## Project Overview

This project presents an Autonomous SLAM (Simultaneous Localization and Mapping) System designed for the AgileX LIMO rover. The system leverages the capabilities of ROS (Robot Operating System) to control and manage the operations of the LIMO rover, providing it with the autonomy to navigate and map its environment in real-time. 

The simulation component of the project is realized through Gazebo, a powerful robot simulation environment that allows for the testing and development of the SLAM algorithms in a controlled and safe virtual environment. This enables extensive experimentation and tuning before deployment on the physical rover.

On the software side, the project is structured into a backend and a frontend. The backend is developed in Python, utilizing the Flask framework to handle server operations, communication with ROS, and processing of SLAM data. The frontend is built using Angular, offering a user-friendly interface for real-time visualization of the mapping process, rover control, and system configuration.

## Key Features

- **ROS Integration**: Utilizes ROS to facilitate communication, control, and data exchange between the various components of the SLAM system and the AgileX LIMO rover.
- **Gazebo Simulation**: Employs Gazebo for realistic simulations of the rover's operations and SLAM algorithms, allowing for safe and efficient development cycles.
- **Python Backend**: The backend is powered by Python and Flask, ensuring robust server-side operations, including SLAM data processing and API management.
- **Angular Frontend**: An Angular-based frontend provides an interactive and intuitive interface for system monitoring, rover control, and live SLAM visualization.
- **Modular Design**: The system architecture is modular, promoting ease of updates, scalability, and integration with additional sensors or modules for enhanced SLAM capabilities.

## Getting Started

Refer to the subsequent sections for detailed instructions on setting up the simulation environment, running the SLAM system, and interacting with the user interface.

# Video Demonstrations

All the video demonstrations of the various features (simulation/physical robot) are available at the [following link on Google Drive](https://drive.google.com/drive/folders/1FRc0I4VLxESOACXQvV_2FOeH5X6hW15U?usp=sharing).

# Web Application

The web application code is located in the "webApp" folder and is organized into two main parts:

## Front-end

The front-end is coded in TypeScript using the Angular framework. The following coding practices are adhered to:

- Use of the camelCase convention for variable and function names.
- Use of absolute paths for imports.
- Use of Angular directives and components for a modular and reusable structure.

## Back-end

The addition of any new package necessary for the Flask server's operation is listed in the file `/back-end/requirements.txt`. This file enumerates all the essential dependencies required to run the server. Keeping this list up-to-date ensures the correct installation of all necessary dependencies when deploying the Flask server, thus maintaining the consistency of the runtime environment.

The back-end is coded in Python using the Flask framework and is organized into folders for routes, services, and controllers. The following coding practices are applied:

- Use of the snake_case convention for variable and function names.
- Proper management of exceptions and errors for clear HTTP responses.

## Physical Robot

All files necessary for the operation of the robots are located in the following directory:
```bash
./INF3995-104/embedded/agilex_ws
```
To start the robot, execute the following command at startup:
```bash
./INF3995-104/start_script.sh
```
In case of problems when executing `./start_script.sh`, run the following command:
```bash
./INF3995-104/kill.sh
```

## Operation

The user interface displays four main options:

- Launch a physical robot
- Start a simulation
- Identify a robot
- View the mission history
- Update the code of physical robots

Each time an operator selects an option, the request is redirected to the server, either in the form of an HTTP request or through socket communication. The server then appropriately handles the request. Depending on whether the simulation has been chosen in the user interface, the server redirects the requests to the physical robots or to the simulation in Gazebo.

## Unit Tests

Unit tests have been written for each feature of the back-end and front-end. Specific commands must be used to execute them:

For the server side, tests have been developed for the main `.py` files, excluding configuration files:
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
Or to get coverage:

```bash
npm run coverage
```

# Docker: Launch Everything with One Command

Execute `docker compose up` inside this directory. This will set up the frontend server on port 4100 available on the network, the backend server on port 8000 on localhost, and the Gazebo simulation on localhost, port 9090.

To access the application from other devices on the network, please modify the file `/front-end/src/environments/environment.prod.ts` and set the local IP address of the ground station.


# FR
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

En cas de problème lors de l'exécution du ./start_script.sh , exécutez la commande suivant:
```bash
./INF3995-104/kill.sh
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
