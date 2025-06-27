# NordSTM
# NanoEdge AI Classification Project (STM32)

Ce projet vise à implémenter une classification basée sur la bibliothèque **NanoEdge AI** sur une carte STM32, en exploitant un modèle généré depuis **NanoEdge AI Studio**.

---

## 🚧 Statut du projet

❌ **Non fonctionnel**  
Le projet est actuellement en cours de développement. L'exécution échoue lors de l'appel à `neai_classification_init()` en raison d'une erreur liée au **buffer knowledge** :  

> `NEAI_KNOWLEDGE_BUFFER_ERROR`

---

## 🔧 Problème actuel

L'erreur semble provenir d'une **mauvaise initialisation ou d'une mauvaise taille du buffer** utilisé pour le modèle de classification NanoEdge AI.

Points suspects :
- Utilisation d'une valeur fixe pour `NEAI_KNOWLEDGE_BUFFER_SIZE` (ex. `#define NEAI_KNOWLEDGE_BUFFER_SIZE 8192`)
- Mauvais alignement mémoire du buffer
- Inclusion incorrecte ou absente du fichier généré (`neai_inference.h`)

---

## 📁 Structure du projet

- `Core/Src/main.c` – Code principal de la classification
- `Inc/neai_inference.h` – (à générer depuis NanoEdge AI Studio)
- `Middlewares/NanoEdgeAI/` – Librairie binaire générée par NanoEdge AI Studio

---

## ✅ Étapes pour corriger

1. Regénérer la librairie avec NanoEdge AI Studio
2. Inclure correctement le fichier `neai_inference.h`
3. Initialiser le buffer avec la bonne taille, comme suit :

```c
#include "neai_inference.h"

__attribute__((aligned(4))) 
const float knowledge_buffer[NEAI_KNOWLEDGE_BUFFER_SIZE];

ret = neai_classification_init(knowledge_buffer);
