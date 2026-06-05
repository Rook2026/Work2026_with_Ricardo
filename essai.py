"""
TRAVAIL PRATIQUE : Réseau sémantique pour la prise de décision d'un robot
Auteur : [Votre nom]
Date : [Date]
"""

import pygame
import numpy as np
import random
import math
from collections import defaultdict

# ============================================================
# ÉTAPE 1 : Configuration Pygame
# ============================================================

# Initialisation de Pygame
pygame.init()

# Dimensions de la fenêtre (800x600 points)
LARGEUR = 800
HAUTEUR = 600
FENETRE = pygame.display.set_mode((LARGEUR, HAUTEUR))
pygame.display.set_caption("Réseau Sémantique - Robot Decision Making")

# Couleurs
BLANC = (255, 255, 255)
NOIR = (0, 0, 0)
ROUGE = (255, 0, 0)
VERT = (0, 255, 0)
BLEU = (0, 0, 255)
JAUNE = (255, 255, 0)
ORANGE = (255, 165, 0)
GRIS = (128, 128, 128)
VIOLET = (128, 0, 128)

# Paramètres d'affichage
FPS = 60
HORLOGE = pygame.time.Clock()

# ============================================================
# ÉTAPE 2 : États possibles du robot
# ============================================================

ETATS_ROBOT = [
    "AVANCER",      # Mouvement en avant
    "GAUCHE",       # Mouvement à gauche
    "DROITE",       # Mouvement à droite
    "RECULER",      # Mouvement en arrière
    "CAPTEUR",      # Sraactivation du capteur
    "COLLISION"     # Collision
]

# État actuel du robot
ETAT_ACTUEL = "AVANCER"

# ============================================================
# ÉTAPE 3 : Classe Node (Ung du réseau sémantique)
# ============================================================

class Node:
    """
    Ung d'un réseau sémantique
    
    Attributs:
        nom : nom de l'état
        activation : degré d'activation (0 à 1)
        arcs_entrants : liste des ungs qui mènent à celui-ci
        arcs_sortants : liste des ungs vers lesquels il mène
        probabilites : probabilités de transition vers chaque ung
    """
    
    def __init__(self, nom):
        self.nom = nom
        self.activation = 0.0               # Degré d'activation (0-1)
        self.arcs_entrants = []             # Unqs entrants
        self.arcs_sortants = []             # Unqs sortants
        self.probabilites = {}              # {node_dest: probabilité}
        self.position = None                # Position pour l'affichage
        
    def ajouter_arc_sortant(self, node_dest, probabilite=0.1):
        """Ajoute un arc dirigé vers un autre ung"""
        if node_dest not in self.arcs_sortants:
            self.arcs_sortants.append(node_dest)
            self.probabilites[node_dest] = probabilite
            node_dest.arcs_entrants.append(self)
            
    def normaliser_probabilites(self):
        """Normalise les probabilités pour qu'elles somment à 1"""
        total = sum(self.probabilites.values())
        if total > 0:
            for dest in self.probabilites:
                self.probabilites[dest] /= total
                
    def choisir_prochain_etat(self):
        """Choisit le prochain état selon les probabilités"""
        if not self.probabilites:
            return None
        etats = list(self.probabilites.keys())
        probs = list(self.probabilites.values())
        return np.random.choice(etats, p=probs)
    
    def __repr__(self):
        return f"Node({self.nom}, activation={self.activation:.2f})"


# ============================================================
# CLASSE SemanticMap (Carte sémantique)
# ============================================================

class SemanticMap:
    """
    Réseau sémantique complet
    
    Attributs:
        noeuds : dictionnaire {nom: Node}
        historique : liste des états précédents
    """
    
    def __init__(self):
        self.noeuds = {}
        self.historique = []
        self.initialiser_reseau()
        
    def initialiser_reseau(self):
        """Crée tous les unqs du réseau"""
        # Création des unqs
        for etat in ETATS_ROBOT:
            self.noeuds[etat] = Node(etat)
            
        # Connexions initiales (structure de base)
        # AVANCER peut mener à CAPTEUR ou COLLISION
        self.noeuds["AVANCER"].ajouter_arc_sortant(self.noeuds["CAPTEUR"], 0.6)
        self.noeuds["AVANCER"].ajouter_arc_sortant(self.noeuds["COLLISION"], 0.4)
        
        # GAUCHE peut mener à CAPTEUR ou COLLISION
        self.noeuds["GAUCHE"].ajouter_arc_sortant(self.noeuds["CAPTEUR"], 0.7)
        self.noeuds["GAUCHE"].ajouter_arc_sortant(self.noeuds["COLLISION"], 0.3)
        
        # DROITE peut mener à CAPTEUR ou COLLISION
        self.noeuds["DROITE"].ajouter_arc_sortant(self.noeuds["CAPTEUR"], 0.7)
        self.noeuds["DROITE"].ajouter_arc_sortant(self.noeuds["COLLISION"], 0.3)
        
        # RECULER a moins de risque de collision
        self.noeuds["RECULER"].ajouter_arc_sortant(self.noeuds["CAPTEUR"], 0.8)
        self.noeuds["RECULER"].ajouter_arc_sortant(self.noeuds["COLLISION"], 0.2)
        
        # CAPTEUR peut mener à différentes actions
        self.noeuds["CAPTEUR"].ajouter_arc_sortant(self.noeuds["GAUCHE"], 0.3)
        self.noeuds["CAPTEUR"].ajouter_arc_sortant(self.noeuds["DROITE"], 0.3)
        self.noeuds["CAPTEUR"].ajouter_arc_sortant(self.noeuds["RECULER"], 0.3)
        self.noeuds["CAPTEUR"].ajouter_arc_sortant(self.noeuds["AVANCER"], 0.1)
        
        # COLLISION (état absorbant)
        self.noeuds["COLLISION"].ajouter_arc_sortant(self.noeuds["RECULER"], 0.5)
        self.noeuds["COLLISION"].ajouter_arc_sortant(self.noeuds["GAUCHE"], 0.5)
        
        # Normalisation des probabilités
        for noeud in self.noeuds.values():
            noeud.normaliser_probabilites()
            
        # Positions pour l'affichage (cercle trigonométrique)
        self.definir_positions()
        
    def definir_positions(self):
        """Définit les positions des unqs pour l'affichage"""
        centre_x = LARGEUR // 2
        centre_y = HAUTEUR // 2
        rayon = 200
        
        angles = {
            "AVANCER": 270,    # Nord
            "GAUCHE": 180,     # Ouest
            "DROITE": 0,       # Est
            "RECULER": 90,     # Sud
            "CAPTEUR": 315,    # Nord-Est
            "COLLISION": 135   # Sud-Ouest
        }
        
        for nom, noeud in self.noeuds.items():
            angle_rad = math.radians(angles[nom])
            x = centre_x + rayon * math.cos(angle_rad)
            y = centre_y + rayon * math.sin(angle_rad)
            noeud.position = (x, y)
            
    # ============================================================
    # ÉTAPE 4 : Fonction d'activation probabiliste
    # ============================================================
    
    def mettre_a_jour_activations(self, etat_actuel, activation_entree=1.0):
        """
        Met à jour les degrés d'activation des unqs
        
        Principe :
            - L'état actuel reçoit l'activation d'entrée
            - L'activation se propage aux unqs voisins
            - Décroissance exponentielle
        """
        # Réinitialiser les activations
        for noeud in self.noeuds.values():
            noeud.activation = 0.0
            
        # Activer l'état courant
        noeud_actuel = self.noeuds[etat_actuel]
        noeud_actuel.activation = activation_entree
        
        # Propagation aux voisins directs
        for noeud in self.noeuds.values():
            if noeud.activation > 0:
                for voisin in noeud.arcs_sortants:
                    # Propagation avec facteur d'atténuation
                    gain = noeud.probabilites.get(voisin, 0.1)
                    voisin.activation += noeud.activation * gain * 0.5
                    
        # Limiter les activations entre 0 et 1
        for noeud in self.noeuds.values():
            noeud.activation = min(1.0, noeud.activation)
            
    # ============================================================
    # ÉTAPE 5 : Apprentissage automatique (historique)
    # ============================================================
    
    def apprendre_depuis_historique(self, historique):
        """
        Ajuste les probabilités à partir de l'historique des états
        
        Principe : Statistique des transitions observées
        """
        # Compter les transitions
        transitions = defaultdict(lambda: defaultdict(int))
        
        for i in range(len(historique) - 1):
            etat_courant = historique[i]
            etat_suivant = historique[i + 1]
            transitions[etat_courant][etat_suivant] += 1
            
        # Mettre à jour les probabilités
        for etat, dest_comptes in transitions.items():
            if etat in self.noeuds:
                total = sum(dest_comptes.values())
                noeud = self.noeuds[etat]
                for dest, compte in dest_comptes.items():
                    if dest in self.noeuds:
                        noeud.probabilites[self.noeuds[dest]] = compte / total
                noeud.normaliser_probabilites()
                
    def ajouter_a_historique(self, etat):
        """Ajoute un état à l'historique"""
        self.historique.append(etat)
        if len(self.historique) > 100:
            self.historique.pop(0)
            
    # ============================================================
    # ÉTAPE 8 : Décision anti-collision
    # ============================================================
    
    def prendre_decision(self, capteur_active=False):
        """
        Prend une décision pour éviter les collisions
        
        Règles :
            - Si capteur actif, privilégier GAUCHE/DROITE/RECULER
            - Si collision récente, choisir action sûre
        """
        noeud_actuel = self.noeuds[ETAT_ACTUEL]
        
        # Vérifier si le capteur est activé (danger détecté)
        if capteur_active:
            print("[CAPTEUR] Obstacle détecté! Évitement...")
            # Choisir parmi les actions sûres (sauf AVANCER si danger)
            actions_sures = ["GAUCHE", "DROITE", "RECULER"]
            # Pondérer selon les probabilités apprises
            probs = []
            for action in actions_sures:
                prob = noeud_actuel.probabilites.get(self.noeuds[action], 0.2)
                probs.append(prob)
            total = sum(probs)
            if total > 0:
                probs = [p/total for p in probs]
                decision = np.random.choice(actions_sures, p=probs)
            else:
                decision = random.choice(actions_sures)
            return decision
            
        # Sinon, comportement normal selon le réseau
        prochain = noeud_actuel.choisir_prochain_etat()
        if prochain:
            return prochain.nom
        return "AVANCER"
        
    # ============================================================
    # ÉTAPE 6 & 7 : Visualisation
    # ============================================================
    
    def dessiner(self, fenetre):
        """Dessine le réseau sémantique"""
        
        # Dessiner les arcs (liens entre unqs)
        for noeud in self.noeuds.values():
            for voisin in noeud.arcs_sortants:
                debut = noeud.position
                fin = voisin.position
                
                # Couleur selon la probabilité (plus la proba est grande, plus la ligne est épaisse)
                proba = noeud.probabilites.get(voisin, 0.1)
                epaisseur = max(1, int(proba * 5))
                
                # Dégradé de couleur (vert pour forte proba, rouge pour faible)
                couleur = (int(255 * (1 - proba)), int(255 * proba), 0)
                
                pygame.draw.line(fenetre, couleur, debut, fin, epaisseur)
                
                # Ajouter une flèche pour indiquer la direction
                self.dessiner_fleche(fenetre, debut, fin, couleur)
                
        # Dessiner les unqs (cercles)
        for noeud in self.noeuds.values():
            x, y = noeud.position
            
            # Couleur selon l'activation (plus l'activation est forte, plus c'est vert)
            intensite = int(255 * noeud.activation)
            if noeud.activation > 0.7:
                couleur_noeud = VERT
            elif noeud.activation > 0.3:
                couleur_noeud = JAUNE
            else:
                couleur_noeud = GRIS
                
            # Cercle extérieur
            pygame.draw.circle(fenetre, NOIR, (int(x), int(y)), 35, 2)
            pygame.draw.circle(fenetre, couleur_noeud, (int(x), int(y)), 33)
            
            # Texte
            font = pygame.font.Font(None, 24)
            texte = font.render(noeud.nom, True, NOIR)
            rect_texte = texte.get_rect(center=(int(x), int(y)))
            fenetre.blit(texte, rect_texte)
            
            # Afficher la probabilité d'activation (petit texte)
            font_petit = pygame.font.Font(None, 18)
            prob_texte = font_petit.render(f"{noeud.activation:.2f}", True, BLEU)
            rect_prob = prob_texte.get_rect(center=(int(x), int(y) + 45))
            fenetre.blit(prob_texte, rect_prob)
            
    def dessiner_fleche(self, fenetre, debut, fin, couleur):
        """Dessine une flèche entre deux points"""
        # Calculer l'angle
        dx = fin[0] - debut[0]
        dy = fin[1] - debut[1]
        angle = math.atan2(dy, dx)
        
        # Position de la pointe (à 80% du segment)
        t = 0.8
        pointe_x = debut[0] + t * dx
        pointe_y = debut[1] + t * dy
        
        # Taille de la flèche
        taille_fleche = 10
        
        # Points de la flèche
        p1_x = pointe_x - taille_fleche * math.cos(angle - math.pi/6)
        p1_y = pointe_y - taille_fleche * math.sin(angle - math.pi/6)
        p2_x = pointe_x - taille_fleche * math.cos(angle + math.pi/6)
        p2_y = pointe_y - taille_fleche * math.sin(angle + math.pi/6)
        
        # Dessiner la flèche
        pygame.draw.polygon(fenetre, couleur, 
                           [(pointe_x, pointe_y), (p1_x, p1_y), (p2_x, p2_y)])


# ============================================================
# SIMULATION DU ROBOT
# ============================================================

class RobotSimulator:
    """Simulateur de robot avec capteurs"""
    
    def __init__(self):
        self.x = LARGEUR // 2 - 200
        self.y = HAUTEUR // 2
        self.angle = 0  # 0 = droite, 90 = bas, 180 = gauche, 270 = haut
        self.vitesse = 3
        self.capteur_active = False
        self.historique_capteurs = []
        
    def deplacer(self, action):
        """Déplace le robot selon l'action choisie"""
        global ETAT_ACTUEL
        
        ETAT_ACTUEL = action
        
        # Sauvegarder la position précédente pour détecter collision
        ancien_x, ancien_y = self.x, self.y
        
        if action == "AVANCER":
            self.x += self.vitesse * math.cos(math.radians(self.angle))
            self.y += self.vitesse * math.sin(math.radians(self.angle))
        elif action == "RECULER":
            self.x -= self.vitesse * math.cos(math.radians(self.angle))
            self.y -= self.vitesse * math.sin(math.radians(self.angle))
        elif action == "GAUCHE":
            self.angle = (self.angle - 20) % 360
        elif action == "DROITE":
            self.angle = (self.angle + 20) % 360
            
        # Détection de collision avec les bords
        if self.x < 50 or self.x > LARGEUR - 50 or self.y < 50 or self.y > HAUTEUR - 50:
            self.x, self.y = ancien_x, ancien_y
            return "COLLISION"
            
        # Simulation du capteur (détection d'obstacle)
        self.capteur_active = self.simuler_capteur()
        
        if self.capteur_active:
            return "CAPTEUR"
        return action
        
    def simuler_capteur(self):
        """Simule un capteur de proximité (murs virtuels)"""
        # Rayon de détection
        rayon_detection = 80
        
        # Détection des bords
        if (self.x < rayon_detection or 
            self.x > LARGEUR - rayon_detection or 
            self.y < rayon_detection or 
            self.y > HAUTEUR - rayon_detection):
            return True
            
        # Probabililté aléatoire pour simuler des obstacles irréguliers
        return random.random() < 0.05
        
    def dessiner(self, fenetre):
        """Dessine le robot"""
        # Corps du robot (carré)
        rect_robot = pygame.Rect(self.x - 15, self.y - 15, 30, 30)
        pygame.draw.rect(fenetre, BLEU, rect_robot)
        
        # Indicateur de direction
        fin_x = self.x + 20 * math.cos(math.radians(self.angle))
        fin_y = self.y + 20 * math.sin(math.radians(self.angle))
        pygame.draw.line(fenetre, ROUGE, (self.x, self.y), (fin_x, fin_y), 3)
        
        # Champ du capteur (si actif)
        if self.capteur_active:
            cercle_capteur = pygame.draw.circle(fenetre, ORANGE, (int(self.x), int(self.y)), 40, 2)
            
        # Affichage de l'état
        font = pygame.font.Font(None, 36)
        texte_etat = font.render(f"Etat: {ETAT_ACTUEL}", True, NOIR)
        fenetre.blit(texte_etat, (10, 10))


# ============================================================
# PROGRAMME PRINCIPAL
# ============================================================

def main():
    """Fonction principale - Simulation du réseau sémantique"""
    
    # Création des objets
    reseau = SemanticMap()
    robot = RobotSimulator()
    
    # ============================================================
    # ÉTAPE 10 : Données historiques d'entrée
    # ============================================================
    
    # Historique d'apprentissage (séquences d'états)
    historique_apprentissage = [
        "AVANCER", "CAPTEUR", "GAUCHE", "AVANCER",
        "AVANCER", "CAPTEUR", "DROITE", "AVANCER",
        "AVANCER", "COLLISION", "RECULER", "GAUCHE", "AVANCER",
        "AVANCER", "CAPTEUR", "RECULER", "DROITE", "AVANCER",
        "GAUCHE", "AVANCER", "CAPTEUR", "GAUCHE", "AVANCER"
    ]
    
    print("=== APPRENTISSAGE DU RÉSEAU SÉMANTIQUE ===")
    print("Historique d'entrée:")
    print(historique_apprentissage)
    print()
    
    # Apprentissage à partir de l'historique
    reseau.apprendre_depuis_historique(historique_apprentissage)
    
    print("=== DÉMARRAGE DE LA SIMULATION ===")
    print("Le robot navigue et évite les obstacles...")
    print("- Capteur orange = obstacle détecté")
    print("- Unqs verts = haute activation")
    print()
    
    # Variables de simulation
    en_cours = True
    compteur = 0
    historique_simulation = []
    
    # Boucle principale
    while en_cours and compteur < 500:
        
        # Gestion des événements
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                en_cours = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    en_cours = False
                    
        # ============================================================
        # ÉTAPE 4 & 8 : Prise de décision et activation
        # ============================================================
        
        # Prendre une décision en fonction du capteur
        decision = reseau.prendre_decision(robot.capteur_active)
        
        # Déplacer le robot selon la décision
        nouvel_etat = robot.deplacer(decision)
        
        # Enregistrer dans l'historique
        historique_simulation.append(nouvel_etat)
        
        # Mettre à jour les activations du réseau
        reseau.mettre_a_jour_activations(nouvel_etat)
        
        # ============================================================
        # AFFICHAGE
        # ============================================================
        
        # Effacer l'écran
        FENETRE.fill(BLANC)
        
        # Afficher le titre
        font_titre = pygame.font.Font(None, 48)
        titre = font_titre.render("Reseau Semantique pour Robot", True, NOIR)
        FENETRE.blit(titre, (LARGEUR//2 - titre.get_width()//2, 10))
        
        # Dessiner le réseau sémantique
        reseau.dessiner(FENETRE)
        
        # Dessiner le robot
        robot.dessiner(FENETRE)
        
        # Afficher les statistiques
        font_stats = pygame.font.Font(None, 24)
        stats_y = HAUTEUR - 80
        
        stats1 = font_stats.render(f"Etape: {compteur}", True, NOIR)
        stats2 = font_stats.render(f"Decision: {decision}", True, NOIR)
        stats3 = font_stats.render(f"Capteur: {'ACTIF' if robot.capteur_active else 'INACTIF'}", 
                                   True, ORANGE if robot.capteur_active else GRIS)
        
        FENETRE.blit(stats1, (10, stats_y))
        FENETRE.blit(stats2, (10, stats_y + 25))
        FENETRE.blit(stats3, (10, stats_y + 50))
        
        # Légende
        legende_y = HAUTEUR - 150
        legende1 = font_stats.render("Lignes vertes: forte probabilite", True, VERT)
        legende2 = font_stats.render("Lignes rouges: faible probabilite", True, ROUGE)
        legende3 = font_stats.render("Cercle vert: activation elevee", True, VERT)
        legende4 = font_stats.render("Cercle gris: activation faible", True, GRIS)
        
        FENETRE.blit(legende1, (LARGEUR - 250, legende_y))
        FENETRE.blit(legende2, (LARGEUR - 250, legende_y + 25))
        FENETRE.blit(legende3, (LARGEUR - 250, legende_y + 50))
        FENETRE.blit(legende4, (LARGEUR - 250, legende_y + 75))
        
        # Mettre à jour l'affichage
        pygame.display.flip()
        
        # Contrôle de la vitesse
        HORLOGE.tick(FPS)
        
        compteur += 1
        
    # ============================================================
    # RAPPORT FINAL
    # ============================================================
    
    print("\n=== SIMULATION TERMINÉE ===")
    print(f"Nombre d'étapes: {compteur}")
    
    # Statistiques des collisions
    nb_collisions = historique_simulation.count("COLLISION")
    print(f"Nombre de collisions évitées/corrigées: {nb_collisions}")
    
    print("\nHistorique des états:")
    for i, etat in enumerate(historique_simulation[-20:]):
        print(f"  Etape {i}: {etat}")
        
    # Attendre avant de fermer
    pygame.time.wait(3000)
    pygame.quit()
    
    return reseau, historique_simulation


# Exécution du programme
if __name__ == "__main__":
    reseau, historique = main()