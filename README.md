# 🚀 Projet STM32 – Murgia / Prudhomme / Quivront

Ce dépôt contient le code source du projet réalisé dans le cadre du cours de microcontrôleur (STM32). Il est basé sur la carte **NUCLEO-L152RE**, le capteur **HTS221**, et l’utilisation de **NanoEdge AI Studio** pour la classification. Le projet respecte les contraintes imposées par le sujet (ADC, GPIO, PWM, SPI, UART, interruptions...).

---

## 🗂️ Organisation du dépôt

Le dépôt est divisé en **deux répertoires principaux** correspondant aux grandes étapes du projet :

### 📁 `datalogger/`

> 🧪 Cette première partie permet de **récupérer et afficher les valeurs brutes du capteur HTS221** via UART et écran SPI.

* Affichage des valeurs sur TeraTerm
* Écriture propre avec interruptions, SPI, ADC...
* Permet de valider l’acquisition et la configuration matérielle

### 📁 `classification/`

> 🧠 Partie finale du projet avec **intégration de NanoEdge AI** pour classifier les données (humid / dry / ambient) et déclencher des actions.

* IA embarquée avec 3 classes
* Affichage des classes sur écran SPI
* Moteur activé si la classe détectée dépasse un seuil
* Utilisation des boutons en interruptions
* Anti-rebond, PWM moteur, ajustement dynamique des seuils


💡 Nous avons également inclus dans le dépôt le **fichier binaire généré avec NanoEdge AI Studio pour la carte NUCLEO-L476RG**, utilisé lors de la phase de prototypage initial avant le portage définitif sur la carte L152RE.

---

## 🛠️ Pré-requis

* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* Git
* TeraTerm (ou équivalent)
* Carte NUCLEO-L152RE + capteur HTS221 (I2C) + écran SPI + moteur (PWM)
* Dossiers générés par NanoEdge AI exportés dans `/NanoEdgeAI`

---

## 📦 Installation

### 1. Cloner le dépôt Git

> 📍 Placez-vous d'abord dans le **dossier de destination** souhaité sur votre PC, puis entrez :

```bash
git clone https://github.com/KillahNeo/STM32_PROJET_MURGIA_PRUDHOMME_QUIVRONT.git --recurse-submodules
```
