# Système de Suivi de Lignes pour Véhicule Autonome

Ce projet implémente un système de détection et de suivi de lignes pour un véhicule autonome, utilisant une Raspberry Pi, une caméra et des algorithmes de vision par ordinateur.

## Description

Le système utilise la vision par ordinateur pour détecter les lignes de route, calculer la position relative du véhicule, et ajuster sa direction en conséquence. Il est conçu pour être robuste face aux conditions variables d'éclairage et aux détections partielles de lignes.

### Caractéristiques principales

- Détection en temps réel des lignes de route
- Support pour configuration mono ou double caméra
- Ajustement interactif des paramètres de détection
- Mécanismes de résilience pour maintenir le suivi en cas de détection partielle
- Contrôle proportionnel du moteur de direction

## Architecture du système

Le système est organisé en plusieurs modules :

1. **Module Caméra** : Acquisition des images via la caméra Raspberry Pi
2. **Module Détection** : Traitement d'image et détection des lignes
3. **Module Décision** : Analyse des lignes pour déterminer la position du véhicule
4. **Module Moteur** : Contrôle du moteur de direction basé sur les décisions

## Installation

### Prérequis

- Raspberry Pi 3 ou plus récent
- Camera Pi ou caméra USB compatible
- Moteur et circuit de contrôle appropriés
- Python 3.7+
- OpenCV 4.x

### Dépendances

Installez les dépendances requises :

```bash
pip install opencv-python numpy picamera2
```

### Configuration matérielle

1. Connectez la caméra à la Raspberry Pi
2. Branchez le moteur aux GPIO selon la configuration indiquée :
   - EN sur GPIO 18
   - IN1 sur GPIO 20
   - IN2 sur GPIO 16

## Utilisation

### Configuration de base

1. Cloner le dépôt :
   ```bash
   git clone https://gitlab-df.imt-atlantique.fr/pronto/code-suivi-de-2-lignes.git
   cd code-suivi-de-2-lignes
   ```

2. Exécuter le programme principal :
   ```bash
   python main.py
   ```

### Ajustement des paramètres

Pour ajuster les paramètres de détection de lignes en temps réel :

```bash
python main.py --show_visuals --adjust_parameters
```

Les paramètres optimisés seront enregistrés dans `single_camera_config.json` et réutilisés automatiquement lors des prochaines exécutions.

### Configuration avancée

Différentes options sont disponibles :

- Mode double caméra : `python main.py --dual_camera`
- Visualisation des détections : `python main.py --show_visuals`
- Mode de débogage : `python main.py --debug`

## Structure des fichiers

```markdown
.
├── camera_module.py         # Module pour l'acquisition d'images
├── detection_module.py       # Module pour la détection de lignes
├── decision_module.py        # Module pour la prise de décision
├── motor_module.py           # Module pour le contrôle du moteur
├── config/                   # Dossier pour les fichiers de configuration
│   ├── single_camera_config.json  # Configuration pour la caméra unique
│   └── dual_camera_config.json    # Configuration pour la double caméra
├── logs/                     # Dossier pour les fichiers journaux
└── main.py                   # Point d'entrée principal du programme
```

## Journal des modifications

Consultez le fichier `CHANGELOG.md` pour un historique complet des modifications.

## Auteurs et reconnaissance

- Loïc Fournier (chef de projet, électronique)
- Antoine Jourdain (électronique)
- Daniélou Titouan (informatique)
- Dode Nicolas (informatique)
- Fontan Julien (informatique)

Remerciements spéciaux au professeur Christophe Lohr du Département Informatique d’IMT Atlantique pour nous avoir accompagnés dans ce projet.

## Licence

Ce projet est sous licence MIT.