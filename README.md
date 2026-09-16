# Robotic Arm Controller (Arduino & Dynamixel)

Firmware Arduino pour le contrôle cinématique d'un bras robotique articulé multi-degrés de liberté (multi-DoF), combinant des moteurs pas-à-pas pour les axes principaux et un servomoteur intelligent Dynamixel pour l'organe terminal.

---

## Table des Matières

1. [Aperçu du Projet](#aperçu-du-projet)  
2. [Caractéristiques Principales](#caractéristiques-principales)  
3. [Architecture Matérielle & Câblage](#architecture-matérielle--câblage)  
4. [Modèle Cinématique](#modèle-cinématique)  
5. [Interface Série & Commandes](#interface-série--commandes)  
6. [Dépendances & Installation](#dépendances--installation)  
7. [Mise en Route](#mise-en-route)  
8. [Perspectives d'Amélioration](#perspectives-damélioration)

---

## Aperçu du Projet

Ce dépôt contient le code source (`code_final_V1.ino`) implémentant :

- Le pilotage en pas-à-pas de 3 axes (Base, Épaule/Bras, Coude/Poignet) avec génération d'impulsions (STEP/DIR) et gestion des micro-pas.  
- L'asservissement en position d'un servomoteur Dynamixel via bus série demi-duplex haute vitesse (1 MBauds).  
- La résolution de la **cinématique inverse (IK)** analytique permettant de convertir des coordonnées cartésiennes $(X, Y, Z)$ en angles articulaires $(\\theta\_0, \\theta\_1, \\theta\_2, \\theta\_3, \\theta\_4)$.  
- Une procédure d'origine machine (**Homing**) automatique grâce à des capteurs de fin de course (*endstops*).  
- Un interpréteur de commandes série gérant les déplacements en repère **absolu** ou **relatif**.

---

## Caractéristiques Principales

- **Contrôle Hybride :** Moteurs pas-à-pas à fort couple pour la structure porteuse \+ servo Dynamixel pour la dextérité terminale.  
- **Cinématique Inverse Intégrée :** Calcul trigonométrique 3D en temps réel sur microcontrôleur.  
- **Microstepping :** Configuration par défaut à 1/8 de pas (1600 micropas/tour pour moteurs 200 pas/tour).  
- **Sécurité & Calibrage :** Séquence de mise à zéro séquentielle sur butées mécaniques / optiques.

---

## Architecture Matérielle & Câblage

### 1\. Broches Microcontrôleur (Arduino)

| Composant / Axe | STEP | DIR | ENABLE | Signal / Endstop |
| :---- | :---: | :---: | :---: | :---: |
| **Moteur Base (Axe 0\)** | Pin 8 | Pin 9 | Pin 10 | Pin 12 (`ENDSTOP_PIN_1`) |
| **Moteur Bras / Épaule (Axe 1\)** | Pin 2 | Pin 3 | Pin 4 | Pin 13 (`ENDSTOP_PIN_2`) |
| **Moteur Avant-bras / Coude (Axe 4\)** | Pin 5 | Pin 6 | Pin 7 | Pin 14 (`ENDSTOP_PIN_3`) |
| **Dynamixel (Série demi-duplex)** | \- | \- | \- | Pin 2 (Contrôle Tx/Rx) |
| **Contrôle auxiliaire** | \- | \- | \- | Pin 11 (`PIN_CONTROL`) |

### 2\. Spécifications Actionneurs & Bus

- **Vitesse Liaison Série USB :** 9600 bauds  
- **Bus Dynamixel :** 1 000 000 bauds (ID configuré : `12`)  
- **Microstepping :** $200 \\times 8 \= 1600\\text{ pas/tour}$

---

## Modèle Cinématique

Le bras robotique est modélisé par des segments articulés successifs :

- $l\_1 \= 0.24\\text{ m}$ (Premier membre / Bras)  
- $l\_2 \= 0.21\\text{ m}$ (Second membre / Avant-bras)  
- $l\_3 \= 0.14\\text{ m}$ (Poignet)  
- $l\_4 \= 50\\text{ mm}$ (Organe terminal / Pince)

### Calculs Articulaires

1. **Rotation Base ($\\theta\_0$) :** $$\\theta\_0 \= \\text{atan2}(\\Delta y, \\Delta x)$$  
2. **Angle d'Épaule ($\\theta\_1$) & Coude ($\\theta\_2$) :** Résolution géométrique planaire par la loi des cosinus sur la distance radiale $r \= \\sqrt{\\Delta x^2 \+ \\Delta y^2 \+ \\Delta z^2}$.  
3. **Orientation Terminale ($\\theta\_3, \\theta\_4$) :** Compensation d'assiette pour maintenir l'orientation requise de l'effecteur.

---

## Interface Série & Commandes

Le firmware écoute sur le port série à **9600 bauds** :

| Commande | Description | Paramètres / Comportement |
| :---- | :---- | :---- |
| `i` | **Initialisation / Homing** | Déplace les axes vers les endstops séquentiellement pour fixer l'origine. |
| `M` | **Déplacement Cartésien** | Prise en compte du mode absolu (`mode_flag = 0`) ou relatif (`mode_flag = 1`). |
| `?` | **Aide** | Affiche le menu des commandes disponibles. |

---

## Dépendances & Installation

### Matériel Requis

- Carte Arduino (ex: Mega 2560 ou Uno)  
- 3 drivers de moteurs pas-à-pas (A4988, DRV8825, TMC2208/2209 ou équivalents)  
- 3 moteurs pas-à-pas (NEMA 17 / NEMA 23\)  
- 3 contacteurs de fin de course (normalement ouverts ou fermés)  
- 1 servomoteur Dynamixel (série AX ou MX) avec circuit tampon demi-duplex (74LVC2G241 / MAX485 ou Shield Dynamixel)  
- Alimentation stabilisée 12V/24V adaptée aux moteurs

### Dépendances Logicielles

- **Arduino IDE** (version 1.8.x ou 2.x)  
- Bibliothèque **`DynamixelSerial2`**

---

## Mise en Route

1. **Cloner le dépôt :**  
   git clone https://github.com/\<votre-utilisateur\>/\<nom-du-repo\>.git  
   cd \<nom-du-repo\>  
2. **Installer la bibliothèque :** Téléchargez et décompressez `DynamixelSerial2` dans votre dossier `Arduino/libraries/`.  
3. **Téléverser le programme :**  
   - Ouvrez `code_final_V1.ino` dans l'Arduino IDE.  
   - Sélectionnez votre carte et le port COM approprié.  
   - Cliquez sur **Téléverser**.  
4. **Utilisation :**  
   - Ouvrez le moniteur série (9600 bauds).  
   - Envoyez la commande `i` pour calibrer les origines machine.  
   - Envoyez des consignes de trajectoire.

---

## Structure du Dépôt

.

├── code\_final\_V1.ino     \# Firmware Arduino principal

└── README.md             \# Documentation du projet

---

## Auteur & Licence

- **Projet réalisé par :** Mohamed AMRI  


&nbsp;
