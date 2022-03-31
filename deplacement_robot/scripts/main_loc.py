import cv2
import numpy as np
from matUtils import R_to_bryant, bryant_to_R
from localisation_exception import UntrustworthyLocalisationError, MatchingError
from hole_detection import hough
from pnp_solving import process_pnp, calcule_erreur
from matUtils import R_from_vect
from extractHoles import getAllCircles
import sys

from matchPoints import generatePointLines, removeNonSimilarLines, formatPointsForPnP

def main_localisation(  type_plaque, 
                        chemin_modele, 
                        image,
                        matrice_homogene_3D_outils,
                        matrice_passage_outils_cam,
                        matrice_intrinseque,
                        coefficients_de_distortion):
    """main_localisation does blah blah blah.

    :type_plaque: "Tole cintree" ou "Tole plate" ou Tole epaisse"
    :chemin_modele: 
    :photo: 
    :matrice_homogene_3D_outils:
    :matrice_passage_outils_cam:
    :matrice_intrinseque:
    :coefficients_de_distortion:
    :return: [x, y, z, alpha, beta, gamma], matrice_extrinseque dans le repere monde OU None si la plaque n'est pas detectee"""
    
    #================================ Detection des trous ================================


    image_points = hough(image)
    
    #================================ Recuperation des positions des trous dans le repere de l'objet ================================

    try:
        with open(chemin_modele) as f:
            file = f.readlines()
        object_points = getAllCircles(file)
        object_points = np.delete(object_points, 3, axis=1)
    except Exception as e:
        print(e)
    
    #================================ Matching des points ================================

    try:
        lines3D, lines2D = generatePointLines(image,  image_points, object_points)
        lines2D,lines3D = removeNonSimilarLines(lines2D, lines3D)
        readyForPnP_2D, readyForPnP_3D = formatPointsForPnP(lines2D,lines3D)

    except Exception as e:
        print("================= Matching error ===============")
        #raise
        raise MatchingError(str(e))

    # S'assurer que les objects points image points sont de taille (n, 3) et (n, 2) avec n >= 4

    #print(readyForPnP_2D.shape)
    #print(readyForPnP_3D.shape)

    #================================ Pnp ransac ================================

    try:
        rotation_vector, translation_vector, inliers = process_pnp(readyForPnP_3D, readyForPnP_2D, matrice_intrinseque, coefficients_de_distortion)

    except Exception as e:
        print(e)

    #================================ Construction de la matrice extrinseque ================================

    try:
        matrice_extrinseque = R_from_vect(np.concatenate([rotation_vector, translation_vector]))

    except Exception as e:
        print(e)

    erreur = -1.

    #================================ Calcul de l'erreur ================================

    try:
        erreur = calcule_erreur(readyForPnP_3D, readyForPnP_2D, inliers, matrice_extrinseque, matrice_intrinseque)
        
    except Exception as e:
        print(e)

    if erreur == -1.:
        return None

    elif erreur > 3:
        raise UntrustworthyLocalisationError(erreur)
    
    
    

    # ================================ Transformation dans le repere monde ================================
	
    
    # Rc * Mco --> Ro * Mom --> Rm 
    matrice_extrinseque[:3, 3] = matrice_extrinseque[:3, 3]/1000.
    #assert matrice_extrinseque.shape==(4,4)

    #extrinseque_monde = np.dot(matrice_homogene_3D_outils,matrice_extrinseque)
    #extrinseque_monde = np.dot(matrice_extrinseque, matrice_passage_outils_cam)
    extrinseque_monde = np.dot(matrice_homogene_3D_outils,matrice_passage_outils_cam)
    extrinseque_monde = np.dot(extrinseque_monde, matrice_extrinseque)
    #print("extrinseque_pnp :\n{}\nmonde-outil :\n{}".format(matrice_extrinseque,matrice_homogene_3D_outils))
    

    # Calcul des angles de Brillant    

    try : 
        bryant = R_to_bryant(extrinseque_monde[:3, :3])
    except Exception as e:
        print(e)

    print(erreur)
    return translation_vector, rotation_vector, extrinseque_monde, bryant
    
	


if __name__ == "__main__":

    distortion_coefs = np.array([   1.55284357e-01,
                                    -3.07067931e+00,  
                                    5.16274059e-03, 
                                    -4.78075223e-03,
                                    1.80663250e+01])


    intrinsic_mat = np.array([  [4.78103205e+03, 0.00000000e+00, 1.20113948e+03],
                                [0.00000000e+00, 4.77222528e+03, 1.14533714e+03],
                                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])


    print(main_localisation(
        "Tole plate", 
        "Data/Plaque1/Model/Plaque_1.stp", 
        cv2.imread("HoleDetection/ShittyDataset/1.bmp"), 
        None, 
        None, 
        intrinsic_mat, 
        distortion_coefs))
