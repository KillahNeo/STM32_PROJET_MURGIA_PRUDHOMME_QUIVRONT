# 🚀 Projet STM32 – Murgia / Prudhomme / Quivront

Ce dépôt contient le code source du projet réalisé dans le cadre du cours de microcontrôleur (STM32). Il est basé sur la carte **NUCLEO-L152RE**, le capteur **HTS221**, et l’utilisation de **NanoEdge AI Studio** pour la classification.

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
* Affichage des classes sur écran SPI 
* Affichage des classes et de la valeur en % via l'UART
* Utilisation des boutons en interruptions pour permettre de déverrouiller la carte via une séquence précise [Sequence : 1xB1 - 2xBTN1 - 1xB1]
* Prise en compte de la valeur de RV2
* Déclenchement du buzzer en interruption si classe "Humid" et valeur de RV2 supérieur a 2V ou si classe "Dry" et valeur RV2 inférieur à 2V 


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

    - Ouvrir STM32CubeIDE

    - Aller dans File > Open Projects from File System

    - Cliquer sur Directory... et naviguer vers :

        le dossier "NUCLEO-L152RE_HTS221_DATA_LOGGER"

        le dossier "NUCLEO-L152RE_HTS221_CLASSIFICATION" 

    - Cliquer sur Finish

 ✅ Le projet s’importe automatiquement dans CubeIDE

  🔧 Configuration de l’afficheur MAX7219 : 
  Pour que l’écran SPI fonctionne correctement avec l’afficheur 7 segments MAX7219, il est nécessaire d'ajouter manuellement ce dossier aux chemins d'inclusion du compilateur dans STM32CubeIDE.

    - Clic droit sur le projet → Properties
    - Allez dans C/C++ General > Paths and Symbols
    - Onglet Includes
    - Add 
    - File System
    - Sélectionner le dossier "display" disponible dans le dossier "Drivers" du projet en cours.
    - Cliquer sur OK

✅ Vous pouvez ensuite le compiler (Project > Build All) et le téléverser (Run > Debug) sur la carte.

### 3. 🎮 Instructions d'utilisation

#### ✅ Pour le projet `NUCLEO-L152RE_HTS221_DATA_LOGGER`

1. Connecter le **shield capteur X-NUCLEO-IKS01A3** sur la carte **NUCLEO-L152RE**, et s'assurer que la carte est bien **enfichée sur la carte mère ISEN32**.
2. Brancher la carte **NUCLEO-L152RE** via le câble **ST-Link USB**
3. Ouvrir **TeraTerm** (ou un terminal série) à **115200 bauds** et sélectionner le port COM associé au ST-Link
4. Compiler et téléverser le projet dans **STM32CubeIDE**
5. Observer dans TeraTerm les **valeurs brutes** d’humidité mesurées par le capteur HTS221 envoyées via **UART**
6. Aucune interaction utilisateur nécessaire — mode continu  
7. 🔁 **Les données récupérées avec ce datalogger ont été utilisées pour entraîner les classes dans NanoEdge AI Studio.**

---

#### ✅ Pour le projet `NUCLEO-L152RE_HTS221_CLASSIFICATION`

1. Connecter le **shield capteur X-NUCLEO-IKS01A3** sur la carte **NUCLEO-L152RE**, et s'assurer que la carte est bien **enfichée sur la carte mère ISEN**.
2. Brancher la carte **NUCLEO-L152RE** via le câble **ST-Link USB**
3. Ouvrir **TeraTerm** à **115200 bauds** et sélectionner le port COM associé au ST-Link
4. Compiler et téléverser le projet dans **STM32CubeIDE**
5. Effectuer la **séquence de démarrage** pour activer le système :
   ```
   Séquence attendue :
   - 1 appui sur B1 (PC13)
   - 2 appuis successifs sur BTN1 (PA11)
   - 1 appui sur B1 de nouveau (PC13)
   ```
   Si la séquence est correcte, un message "Sequence complete OK" s’affiche via UART et un affichage "INIT" est visible sur l'écran SPI. Le système est alors actif.
   💡- Si Teraterm n'affiche aucune valeur à la suite de la séquence débrancher et rebrancher la carte **NUCLEO-L152RE** via le câble **ST-Link USB**
6. L’écran SPI affiche la classe détectée : `SEC` si Dry, `AMB` si Ambient, `HUM` si Humid. TeraTerm affiche également la classe + l’humidité en %.
7. Si la classe détectée est **"Humid"** ET que la valeur du **potentiomètre (RV2)** est > 2V :
   - Le buzzer s’active automatiquement (déclenché via interruption ADC watchdog).
   Si la classe détectée est **"Dry"** ET que la valeur du **potentiomètre (RV2)** est < 2V :
   - Le buzzer s’active automatiquement (déclenché via interruption ADC watchdog).