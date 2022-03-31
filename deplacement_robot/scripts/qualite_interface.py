from qualite_analyse_contour import * 
from qualite_pretraitement import * 
import cv2
import time
from matplotlib import pyplot as plt
from datetime import datetime

def fonction_qualite(rayon_attendu,image_raw,debug,image_globale,dic_points_3d_to_2d,curent_3d_point_xyz_tuple,showResult = True,fast_algo= True):
	
	try:
		key_found = False
		(x_img_2d,y_img_2d) = None,None
		try:
			(x_img_2d,y_img_2d) = dic_points_3d_to_2d[curent_3d_point_xyz_tuple]
			key_found = True
		except:
			print("================================================================")
			print("==================| QUALITE CRITICAL FAILURE |==================")
			print("================================================================")
			print("Key not found in the dictonary")
			print("Key :",curent_3d_point_xyz_tuple)
			print("dic_points_3d_to_2d :",dic_points_3d_to_2d)
			time.sleep(5)
			
		
		#calcul a la main pas fiable.
		start_time = time.time()

		px_to_mm = 0.04624277456
		mm_to_px = 21
		rayon_attendu_px = int(rayon_attendu*mm_to_px*2)  #ici on bosse en px
		if(fast_algo):	        
			#if fast we scale down the img
			height, width, _ = image_raw.shape 
			scale = 5
			rayon_attendu_px=int(rayon_attendu_px/scale)
			dsize = (int(width/scale), int(height/scale))
			image_raw = cv2.resize(image_raw, dsize)


		if(debug):
			cv2.imshow('RAW',cv2.resize(image_raw,(520,388)))
			cv2.waitKey(0)


		image_processed,offset_,nohole=filtrage.preprocess(image_raw,fast_algo)

		if(nohole):
			resized_image = cv2.resize(image_raw,(520,388))
			cv2.circle(resized_image,(int(520/2),int(388/2)),50,(0,0,255),5) #dessine cercle
			if(debug or showResult):
				plt.imshow(resized_image)
				plt.draw()
				plt.pause(0.001)        
			return False,"Trou non detecte",resized_image


		if(debug):
			cv2.imshow('FILTERED',image_processed)
			cv2.waitKey(0)



		isdefective, defect, contours = analyseContour.caracterization(cv2.cvtColor(image_processed,cv2.COLOR_GRAY2RGB),rayon_attendu_px,px_to_mm,affichage=False)

		px_size = 15
		if(fast_algo):
			px_size = 5

		assert(contours is not None)

		print("image raw")
		color_ = (0,255,0)
		if(isdefective):
			color_ = (0,0,255)
		
		image_result = cv2.drawContours(image_raw, contours, -1, color_ , px_size,offset=offset_)


			
		if(key_found):
			rectangle_width_height = 80
			image_globale = cv2.rectangle(image_globale, (x_img_2d-rectangle_width_height,y_img_2d-rectangle_width_height), (x_img_2d+rectangle_width_height,y_img_2d+rectangle_width_height),color_,8) 

			
			
		if(showResult):
			print("["+str((time.time() - start_time))+" seconds]")
			plt.imshow(cv2.resize(image_result,(520,388)))
			plt.imshow(cv2.resize(image_globale,(520,388)))
			plt.draw()
			plt.pause(0.001)

		return not isdefective, defect, image_result
	except:
		print("================================================================")
		print("==================| QUALITE CRITICAL FAILURE |==================")
		print("================================================================")
		now = datetime.now()
		date_time = now.strftime("ERROR_%m_%d_%Y__%H_%M_%S")
		cv2.imwrite("./"+date_time+".png", image_raw)
		return False,"Erreur qualite",image_raw

######################################

import numpy as np
from random import randint

"""
Parametres :
	liste_points_px = coordonnees des trous dans l'image (x,y)
	image = image globale vue du dessus (celle de l"identification)
	resultats = liste de booleen contenant le resultat de conformite du controle qualite
Objectif : Afficher le resultat de conformite sur l'image globale
"""
def run_qualite_image_globale(liste_points_px,image,resultats):
	rectangle_width_height = 80
	image = cv2.imread(image)
	index_result = 0
	for p in liste_points_px:
		x = p[0]
		y = p[1]        
		if(resultats[index_result]):
			image = cv2.rectangle(image, (x-rectangle_width_height,y-rectangle_width_height), (x+rectangle_width_height,y+rectangle_width_height),(0,255,0),8) 
		else:
			image = cv2.rectangle(image, (x-rectangle_width_height,y-rectangle_width_height), (x+rectangle_width_height,y+rectangle_width_height),(0,0,255),8) 

		index_result +=1 

	cv2.imshow('image',cv2.resize(image, (960, 540)))
	cv2.waitKey(0)

#### MAIN pour test simulation ####
if __name__ == "__main__":
	#simulation creation de coordonnees de points
	startX = 457
	startY = 261
	rectangle_width_heightX = 235
	rectangle_width_heightY = 330
	liste_points = []
	curentX = startX
	curentY = startY

	#Simulation resultat qualite
	resultats_qualite = []

	for y in range(5):
		curentY = startY+y*rectangle_width_heightY
		for x in range(8):
			curentX = startX+x*rectangle_width_heightX
			liste_points.append([curentX,curentY])
			value = randint(0, 1)
			resultats_qualite.append(value == 1)
		curentX = startX


		
	image = "./img.jpg"
	run_qualite_image_globale(liste_points,image,resultats_qualite)   
