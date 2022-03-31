import numpy as np
import cv2
from localisation_exception import *
from matUtils import transform_point_with_matrix, perspective_projection

def process_pnp(object_points, image_points, matrice_intrinseque, dist_coefs):

    rotation_guess = np.array([-3.14, 0., 0.])
    #rotation_guess,_ = cv2.Rodrigues(np.array([[-1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]]))
    translation_guess = np.array([0., 0., 1000.])

    success, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        matrice_intrinseque,
        dist_coefs,
        rvec=rotation_guess,
        tvec=translation_guess,
        useExtrinsicGuess=True,
        flags=0)

    return rotation_vector, translation_vector, inliers

def calcule_erreur(object_points, image_points, inliers, matrice_extrinseque, matrice_intrinseque):

    if inliers is None:
        return -1.

    if object_points.shape[0] == 0  or inliers.shape[0] == 0:
        return -1.
    

    object_points_inliers = []
    image_points_inliers = []
    for inl in inliers:
        object_points_inliers.append(object_points[inl[0]])
        image_points_inliers.append(image_points[inl[0]])

    object_points = np.array(object_points)
    image_points = np.array(image_points)


    P_cam = transform_point_with_matrix(matrice_extrinseque, object_points)
    u, v  = perspective_projection(matrice_intrinseque, P_cam)
    reprojected_image_points = np.array([u, v]).T

    error = np.sum(np.sqrt(
        np.dot(reprojected_image_points[:,0] - image_points[:,0], reprojected_image_points[:,0] - image_points[:,0]) + 
        np.dot(reprojected_image_points[:,1] - image_points[:,1], reprojected_image_points[:,1] - image_points[:,1])
        ))/object_points.shape[0]

    
    return error
