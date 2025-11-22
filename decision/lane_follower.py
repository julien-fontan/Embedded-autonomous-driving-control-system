import numpy as np

class LaneFollower:
    def __init__(self, dual_camera=False):
        self.dual_camera = dual_camera
        self.last_offset = 0  # Pour garder la dernière direction connue
        self.offset_history = []  # Historique pour le filtre temporel
        self.max_history = 5  # Taille maximale de l'historique

    def compute_offset(self, lines, frame_shape):
        """
        Calcule l'offset par rapport au centre de la voie.
        Amélioré pour gérer plus de cas complexes.
        """
        frame_center = frame_shape[1] // 2
        
        if len(lines) == 0:
            # Si aucune ligne n'est détectée, utiliser la dernière valeur connue
            return self.last_offset
            
        # Cas avec deux lignes détectées (idéal)
        if len(lines) == 2:
            # Trier les lignes par position x pour identifier gauche et droite
            lines_sorted = sorted(lines, key=lambda line: (line[0] + line[2])/2)
            left_line = lines_sorted[0]
            right_line = lines_sorted[1]
            
            # Calculer le centre de la voie
            left_x2 = left_line[2]
            right_x2 = right_line[2]
            lane_center = (left_x2 + right_x2) // 2
            offset = lane_center - frame_center
        
        # Cas avec une seule ligne détectée
        elif len(lines) == 1:
            line = lines[0]
            x1, y1, x2, y2 = line
            
            # Calculer le milieu de la ligne
            line_middle_x = (x1 + x2) / 2
            
            # Estimer si c'est une ligne gauche ou droite par rapport au centre
            if line_middle_x < frame_center:
                # C'est probablement une ligne gauche
                estimated_lane_width = 200  # Largeur de voie estimée en pixels
                offset = (line_middle_x + estimated_lane_width/2) - frame_center
            else:
                # C'est probablement une ligne droite
                estimated_lane_width = 200  # Largeur de voie estimée en pixels
                offset = (line_middle_x - estimated_lane_width/2) - frame_center
        else:
            # Si configuration non prévue, garder la dernière valeur
            offset = self.last_offset
        
        # Appliquer un filtre temporel pour lisser l'offset
        self.offset_history.append(offset)
        if len(self.offset_history) > self.max_history:
            self.offset_history.pop(0)
            
        smoothed_offset = sum(self.offset_history) / len(self.offset_history)
        self.last_offset = smoothed_offset
        return smoothed_offset

    def decide_action(self, lines, frame_shape):
        # Pour compatibilité, garder l'ancienne logique
        offset = self.compute_offset(lines, frame_shape)
        if offset < -30:
            return "left"
        elif offset > 30:
            return "right"
        else:
            return "stop"

    def get_offset(self, lines, frame_shape):
        # Nouvelle méthode pour obtenir l'offset directement
        """Compute the normalized horizontal offset (-1 to 1) of the vehicle
        relative to the center of the lane.
        """
        return self.compute_offset(lines, frame_shape)