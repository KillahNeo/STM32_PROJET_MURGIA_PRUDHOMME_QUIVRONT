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

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) – IDE officiel pour les cartes STM32
- [NanoEdge AI Studio](https://www.st.com/en/development-tools/nanoedge-ai-studio.html) – outil d'IA embarquée pour STM32
- [Git + Git Bash](https://git-scm.com/downloads) – pour cloner le dépôt localement et exécuter les commandes en ligne
- [TeraTerm](https://osdn.net/projects/ttssh2/releases/) (ou équivalent) – terminal série pour afficher les données UART

Matériel :

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
### 2. Ouvrir un projet dans STM32CubeIDE

    Vous pouvez ouvrir un projet à la fois (datalogger ou classification) comme suit :

    Ouvrir STM32CubeIDE

    Aller dans File > Open Projects from File System

    Cliquer sur Directory... et naviguer vers :

        le dossier "NUCLEO-L152RE_HTS221_DATA_LOGGER"

        le dossier "NUCLEO-L152RE_HTS221_CLASSIFICATION" 

    Cliquer sur Finish

    Le projet s’importe automatiquement dans CubeIDE

✅ Vous pouvez ensuite le compiler (Project > Build All) et le flasher (Run > Debug) sur la carte.

### 3. Instructions d'utilisation

#### ✅ Pour le projet `NUCLEO-L152RE_HTS221_DATA_LOGGER`

1. Brancher la carte **NUCLEO-L152RE** via ST-Link USB  
2. Ouvrir **TeraTerm** (ou un terminal série) à **115200 bauds**
3. Compiler et flasher le projet dans STM32CubeIDE
4. Observer dans TeraTerm les **valeurs brutes** d’humidité mesurées par le capteur HTS221 envoyées via **UART**
5. Aucune interaction utilisateur nécessaire — mode continu  
6. 🔁 **Les données récupérées avec ce datalogger ont été utilisées pour entraîner les classes dans NanoEdge AI Studio.**

---

#### ✅ Pour le projet `NUCLEO-L152RE_HTS221_CLASSIFICATION`

1. Brancher la carte **NUCLEO-L152RE**  
2. Ouvrir **TeraTerm** à **115200 bauds**
3. Compiler et flasher le projet
4. Effectuer la **séquence de démarrage** pour activer le système :
   ```
   Séquence attendue :
   - 1 appui sur B1 (PC13)
   - 2 appuis successifs sur BTN1 (PA11)
   - 1 appui sur B1 de nouveau
   ```
   Si la séquence est correcte, un message "Sequence complete OK" s’affiche et le système démarre.
5. L’écran SPI affiche la classe détectée (`Dry`, `Ambient`, ou `Humid`) et TeraTerm affiche aussi la classe + humidité en %.
6. Si la classe détectée est **"Humid"** **et** que le potentiomètre (RV2) > 2V :
   - Le **buzzer** s’active automatiquement (via interruption ADC watchdog)
