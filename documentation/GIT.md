# Git - Récap Avancé pour Collaboration

## 1. Les Concepts Fondamentaux

### Commit
Un **snapshot** (instantané) de ton code à un moment précis.
```bash
git add fichier.cpp
git commit -m "Message descriptif"
```

**Bonne pratique :** Commits petits et fréquents avec des messages clairs.

### Branche
Une **ligne de développement indépendante**. Comme une copie parallèle de ton code où tu peux expérimenter sans risque.

```bash
git branch              # Liste les branches
git checkout -b nom     # Créer et basculer sur nouvelle branche
git checkout master     # Revenir à master
git branch -d nom       # Supprimer une branche
```

**Cas d'usage :**
- `master/main` : Code stable, production
- `feature/nouvelle-fonctionnalite` : Développement d'une feature
- `bugfix/correction-calcul` : Correction d'un bug
- `experiment/test-algo` : Expérimentation

---

## 2. Merge vs Rebase

Ces deux commandes intègrent les changements d'une branche dans une autre, mais **très différemment**.

### Merge (Fusion)

**Ce que ça fait :** Crée un nouveau commit qui combine deux branches.

```bash
git checkout master
git merge feature/ma-feature
```

**Historique résultant :**
```
       C---D  (feature/ma-feature)
      /     \
A---B-------E  (master, commit de merge)
```

**Avantages :**
- ✅ Préserve l'historique complet (on voit clairement quand la feature a été développée)
- ✅ Simple et sûr
- ✅ Standard pour la collaboration

**Inconvénients :**
- ❌ Crée des commits de merge (historique peut devenir "sale")
- ❌ Graphe non-linéaire

**Quand l'utiliser :**
- Intégrer une feature terminée dans master
- Travailler en équipe (c'est le standard)
- Tu veux garder l'historique complet

---

### Rebase (Réécriture d'historique)

**Ce que ça fait :** Rejoue tes commits par-dessus une autre branche.

```bash
git checkout feature/ma-feature
git rebase master
```

**Historique résultant :**
```
A---B---C'---D'  (feature/ma-feature, commits "rejoués")
     \
      (master)
```

**Avantages :**
- ✅ Historique linéaire et propre
- ✅ Pas de commits de merge
- ✅ Plus facile à lire

**Inconvénients :**
- ❌ Réécrit l'historique (les commits C' et D' sont différents de C et D)
- ❌ **DANGEREUX** si déjà pushé et partagé avec d'autres
- ❌ Conflits peuvent être plus compliqués à résoudre

**Règle d'or du rebase :**
> **JAMAIS rebase une branche publique** (déjà pushée et utilisée par d'autres). Seulement pour nettoyer ton historique local avant de push.

**Quand l'utiliser :**
- Nettoyer tes commits locaux avant de les partager
- Mettre à jour ta branche feature avec les derniers changements de master
- Tu veux un historique propre et linéaire

---

### Merge vs Rebase : Exemple Concret

**Situation :** Tu travailles sur `feature/controleur` et pendant ce temps, quelqu'un a pushé sur `master`.

**Option 1 : Merge**
```bash
git checkout feature/controleur
git merge master  # Intègre les changements de master
```
→ Crée un commit de merge. Historique préservé.

**Option 2 : Rebase**
```bash
git checkout feature/controleur
git rebase master  # Rejoue tes commits par-dessus master
```
→ Tes commits sont "déplacés". Historique linéaire.

---

## 3. Workflow de Collaboration Typique

### Workflow Feature Branch (le plus courant)

```bash
# 1. Récupérer les derniers changements
git checkout master
git pull origin master

# 2. Créer une branche pour ta feature
git checkout -b feature/nouveau-controleur

# 3. Travailler et commiter
# ... modifications ...
git add src/controleurs/pid.cpp
git commit -m "Ajouter contrôleur PID"

# 4. Mettre à jour avec master (si quelqu'un a pushé entre temps)
git fetch origin
git rebase origin/master  # OU git merge origin/master

# 5. Résoudre les conflits si nécessaire
# ... résolution ...
git add fichier-en-conflit
git rebase --continue  # ou git commit si tu as fait un merge

# 6. Push ta branche
git push origin feature/nouveau-controleur

# 7. Créer une Pull Request sur GitHub pour review
# (via l'interface web)

# 8. Après validation, merger dans master
git checkout master
git merge feature/nouveau-controleur
git push origin master

# 9. Supprimer la branche
git branch -d feature/nouveau-controleur
git push origin --delete feature/nouveau-controleur
```

---

## 4. Résolution de Conflits

**Un conflit arrive quand :** Deux personnes modifient la même partie du même fichier.

### Étapes de résolution

```bash
# Après un merge ou rebase avec conflit
git status  # Voir les fichiers en conflit
```

Ouvre le fichier en conflit. Tu verras :
```cpp
<<<<<<< HEAD
// Ta version
int vitesse = 100;
=======
// Leur version
int vitesse = 200;
>>>>>>> feature/autre-branche
```

**Résoudre :**
1. Édite le fichier pour garder ce que tu veux
2. Supprime les marqueurs `<<<<<<<`, `=======`, `>>>>>>>`
3. Sauvegarde

```bash
git add fichier-resolu.cpp
git commit  # ou git rebase --continue si tu faisais un rebase
```

---

## 5. Commandes Utiles Avancées

### Voir l'historique proprement
```bash
git log --oneline --graph --all
```

### Annuler le dernier commit (mais garder les changements)
```bash
git reset --soft HEAD~1
```

### Modifier le dernier commit (message ou contenu)
```bash
git commit --amend
```

### Stash : Mettre de côté des changements temporairement
```bash
git stash          # Mettre de côté
git stash pop      # Récupérer
git stash list     # Voir ce qui est stashé
```

### Cherry-pick : Appliquer un commit spécifique d'une autre branche
```bash
git cherry-pick <commit-hash>
```

### Voir les différences
```bash
git diff                    # Changements non stagés
git diff --staged           # Changements stagés
git diff master..feature    # Différence entre deux branches
```

---

## 6. Les Pièges à Éviter

### ❌ Force Push sur une branche partagée
```bash
git push --force origin master  # JAMAIS FAIRE ÇA !
```
→ Tu écrases l'historique des autres. Catastrophe garantie.

**Exception :** `git push --force-with-lease` est plus sûr (vérifie que personne n'a pushé avant toi).

### ❌ Rebase une branche publique
```bash
# Si ta branche est déjà pushée et utilisée par d'autres
git rebase master  # DANGEREUX
```

### ❌ Commits géants avec plein de changements
```bash
git add .
git commit -m "update"  # Horrible
```
→ Fais des petits commits logiques.

### ❌ Travailler directement sur master
→ Utilise toujours des branches pour les features.

---

## 7. Stratégies de Merge pour Équipes

### Fast-Forward Merge (par défaut)
Si ta branche est directement devant master, git avance juste le pointeur.
```bash
git merge feature  # Pas de commit de merge si fast-forward possible
```

### No-Fast-Forward (--no-ff)
Force la création d'un commit de merge même si fast-forward possible.
```bash
git merge --no-ff feature
```
**Avantage :** Garde trace qu'une feature a été mergée (meilleur pour l'historique).

### Squash Merge
Combine tous les commits de la branche en un seul.
```bash
git merge --squash feature
```
**Avantage :** Historique très propre.
**Inconvénient :** Perd le détail des commits intermédiaires.

---

## 8. Règles d'Or

1. **Commit souvent, push régulièrement**
2. **Branche = 1 feature = plusieurs commits**
3. **Messages de commit clairs et descriptifs**
4. **Ne jamais rebase des branches publiques**
5. **Toujours pull avant de push**
6. **Résoudre les conflits avec attention**
7. **Review le code avant de merger dans master**

---

## 9. Commandes de Secours

### J'ai tout cassé, je veux revenir en arrière
```bash
git reflog  # Voir TOUT ce que tu as fait
git reset --hard <commit-hash>  # Revenir à ce commit
```

### J'ai committé sur master par erreur
```bash
git branch feature/oups  # Créer une branche avec ces commits
git reset --hard origin/master  # Remettre master comme il était
git checkout feature/oups  # Continuer sur la nouvelle branche
```

### J'ai pushé un secret (mot de passe, clé API)
```bash
# Supprimer le fichier de l'historique (avancé, fais gaffe)
git filter-branch --tree-filter 'rm -f secret.txt' HEAD
git push --force
# Puis CHANGE le mot de passe/clé immédiatement !
```

---

## Résumé Visuel

```
Merge : Combine l'historique
A---B---C (master)
     \   \
      D---E---F (feature)
           \   \
            G (commit de merge)

Rebase : Réécrit l'historique  
A---B---C (master)
         \
          D'---E'---F' (feature, commits rejoués)
```

**En équipe : Merge**
**Seul ou avant de partager : Rebase**
