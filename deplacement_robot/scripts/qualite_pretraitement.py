import os
from tkinter import image_types
import cv2
import numpy as np


class filtrage:
    @staticmethod
    def get_circularite(contour):
        aire = cv2.contourArea(contour)
        perimetre = cv2.arcLength(contour, True)
        return (4 * np.pi * aire) / (perimetre ** 2)

    @staticmethod
    def preprocess(image_raw, fast_algo):

        image_filtered = cv2.cvtColor(image_raw, cv2.COLOR_BGR2GRAY)
        image_filtered = cv2.medianBlur(image_filtered, 25)
        image_filtered = cv2.bitwise_not(image_filtered)

        p1 = 1501
        if fast_algo:
            p1 = 501
        image_filtered = cv2.adaptiveThreshold(
            image_filtered, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, p1, 10
        )  # 1501

        des = cv2.bitwise_not(image_filtered)
        contours, _ = cv2.findContours(des, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)[
            -2:
        ]

        if not contours:
            print("EMPTY IMAGE (NO HOLES FOUND)")
            return None, None, True

        closestX = None
        closestY = None
        closestH = None
        closestW = None

        if len(contours) != 1:
            print("multiple shapes detected, croping")
            print("detected ", len(contours))

            # finding contour that look like a cricle
            liste_cercles = []
            threeshold = 0.90
            while len(liste_cercles) == 0:
                for c in contours:
                    circularite = filtrage.get_circularite(c)
                    if circularite > threeshold:
                        liste_cercles.append(c)
                threeshold = threeshold - 0.05

            if len(liste_cercles) > 1:
                # finding centered contour
                imgW, imgH = image_filtered.shape
                centerImg = np.array([imgW / 2, imgH / 2])

                closest_contour = liste_cercles[0]
                (x, y, w, h) = cv2.boundingRect(closest_contour)
                centerM = np.array([int(x + (w / 2)), int(y + (h / 2))])
                closest_dist = np.linalg.norm(centerImg - centerM)

                closestX = x
                closestY = y
                closestH = h
                closestW = w

                for c in liste_cercles[1:]:
                    (x, y, w, h) = cv2.boundingRect(c)
                    centerM = np.array([int(x + (w / 2)), int(y + (h / 2))])
                    d = np.linalg.norm(centerImg - centerM)
                    if d < closest_dist:
                        closest_dist = d
                        closest_contour = c
                        closestX = x
                        closestY = y
                        closestH = h
                        closestW = w
            else:
                contour = liste_cercles[0]
                (x, y, w, h) = cv2.boundingRect(contour)
                closestX = x
                closestY = y
                closestH = h
                closestW = w

        else:
            print("ONE SHAPE DETECTED")
            contour = contours[0]
            # croping around contour to save process time:
            (x, y, w, h) = cv2.boundingRect(contour)
            closestX = x
            closestY = y
            closestH = h
            closestW = w

        border = 100
        if fast_algo:
            border = 20
        imH, imW = image_filtered.shape
        while not (
            closestY - border > 0
            and closestY + closestH + border < imH
            and closestX - border > 0
            and closestX + closestW + border < imW
        ):
            print("image border too big, trying ",border - 1)
            border = border - 1

        image_filtered = image_filtered[
            closestY - border : closestY + closestH + border, closestX - border : closestX + closestW + border
        ]
        offset = (closestX - border, closestY - border)

        assert image_filtered is not None, "empty image"
        return cv2.bitwise_not(image_filtered), offset, False
