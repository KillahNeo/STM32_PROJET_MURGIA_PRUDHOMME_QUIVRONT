# STM32_PROJET_MURGIA_PRUDHOMME_QUIVRONT
PROJET STM 32 : DATALOGGER + CLASSIFICATION
# 🚀 Projet STM32 – Murgia / Prudhomme / Quivront

Ce dépôt contient le code source du projet réalisé dans le cadre du cours de microcontrôleur. Il repose sur la carte **NUCLEO-L152RE**, le capteur **HTS221**, et l’utilisation de **NanoEdge AI Studio** pour la classification. Le projet intègre également des périphériques STM32 classiques (ADC, GPIO, PWM, SPI, UART...) et déclenche des actions physiques (moteur) en fonction de la classe détectée.

## 📁 Structure du projet

- `/Core` : Code principal (main.c, traitement des interruptions, logique du moteur...)
- `/Drivers` : Bibliothèques de périphériques et display
- `*.ioc` : Fichier de configuration STM32CubeMX
- `/NanoEdgeAI` : Fichiers générés par NanoEdge AI Studio (knowledge.h, model.h...)
- `README.md` : Ce fichier

---

## 🛠️ Pré-requis

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Git
- TeraTerm ou autre terminal série
- Carte NUCLEO-L152RE + capteur HTS221 (via I2C) + écran SPI + moteur (PWM)
- Dossier NanoEdgeAI exporté dans `/NanoEdgeAI`

---

## 📦 Installation

### 1. Cloner le dépôt Git

> ⚠️ Avant de lancer la commande, placez-vous dans le **dossier où vous voulez stocker** le projet (ex: `Documents/ISEN/STM32/`) puis ouvrez un terminal (PowerShell ou Git Bash) :

```bash
git clone https://github.com/KillahNeo/STM32_PROJET_MURGIA_PRUDHOMME_QUIVRONT.git --recurse-submodules


