# 🚀 Projet STM32 – Murgia / Prudhomme / Quivront

Ce dépôt contient le code source du projet réalisé dans le cadre du cours de microcontrôleur (STM32). Il est basé sur la carte **NUCLEO-L152RE**, le capteur **HTS221**, et l’utilisation de **NanoEdge AI Studio** pour la classification. Le projet respecte les contraintes imposées par le sujet (ADC, GPIO, PWM, SPI, UART, interruptions...).

---

## 🗂️ Organisation du dépôt

Le dépôt est divisé en **deux répertoires principaux** correspondant aux grandes étapes du projet :

### 📁 `datalogger/`

> 🧪 Cette première partie permet de **récupérer et afficher les valeurs brutes du capteur HTS221** via UART.

* Affichage des valeurs sur TeraTerm
* Permet de valider l’acquisition et la configuration matérielle

### 📁 `classification/`

> 🧠 Partie finale du projet avec **intégration de NanoEdge AI** pour classifier les données et déclencher des actions.

* IA embarquée avec 3 classes : [Dry, Ambiant, Humid]
* Affichage des classes sur écran SPI et 
* Affichage des classes et de la valeur via l'UART
* Utilisation des boutons en interruptions pour permettre de déverrouiller la carte via une séquence précise [Sequence : 1xB1 - 2xBTN1 - 1xB1]
* Prise en compte de la valeur de RV2, Déclenchement du buzzer en interruption si classe "Humid" et valeur de RV2 supérieur a 2V.


💡 Nous avons également inclus dans le dépôt le **fichier binaire généré avec NanoEdge AI Studio pour la carte NUCLEO-L476RG**, utilisé lors de la phase de prototypage initial avant le portage définitif sur la carte L152RE.

---

## 🛠️ Matériel et logiciel utilisé

Logiciel :

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Git
- TeraTerm (ou équivalent)
- NanoedgeAI

Materiel :

- Carte **NUCLEO-L152RE**
- Carte **ISEN32**
- Carte d’extension capteurs : **X-NUCLEO-IKS01A3**
- Capteur utilisé : **HTS221** (humidité relative + température)
- Câble pour debug ST-Link



---

## 📦 Installation

### 1. Cloner le dépôt Git

> 📍 Placez-vous d'abord dans le **dossier de destination** souhaité sur votre PC, puis entrez :

```bash
git clone https://github.com/KillahNeo/STM32_PROJET_MURGIA_PRUDHOMME_QUIVRONT.git --recurse-submodules
```
