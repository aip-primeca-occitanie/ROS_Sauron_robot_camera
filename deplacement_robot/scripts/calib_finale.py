#!/usr/bin/python2.6
# -*-coding:Latin-1 -*

import cv2
import numpy as np
import glob
from PIL import Image 

# Calibre la caméra avec les images fournit en entrée
# in : Chemin vers le dossier contenant les images de la mire
# out : matrice des paramètres intrinsèques, coefficient de distortion, vecteur de rotation, vecteur de translation

def calibration(Path):
    CHECKERBOARD = (16,23)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    threedpoints = []
    twodpoints = []

    objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None

    images = glob.glob(Path)

    for filename in images:

        img = Image.open(filename)
        img = img.convert("RGB")
    
        d = img.getdata()
        new_image = []
        
        # Applique une couleur noire sur les pixels blanc de la dalle lumineuse
        treshold_white = 100
        for item in d:
            if item[0] >= 255-treshold_white and item[1] >= 255-treshold_white and item[2] >= 255-treshold_white:
                new_image.append((0, 0, 0))
            else:
                new_image.append(item)
        
        img.putdata(new_image)
        image = np.array(img)

        grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(grayColor, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH> + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            threedpoints.append(objectp3d)
            corners2 = cv2.cornerSubPix(
                grayColor, corners, (11, 11), (-1, -1), criteria)

            twodpoints.append(corners2)
            image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)


    h, w = image.shape[:2]

    if ret == True:
        ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(threedpoints, twodpoints, grayColor.shape[::-1], None, None)
        return matrix, distortion, r_vecs, t_vecs
    else:
        print("Chessboard not detected")



if __name__ == '__main__':
    Path = 'CALIBRATION/calibration/data_1/*.bmp'
    matrix, distortion, r_vecs, t_vecs = calibration(Path)

    print(matrix)
    print(distortion)