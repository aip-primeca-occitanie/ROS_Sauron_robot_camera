#!/usr/bin/python
import sys
import numpy as np
import cv2
assert sys.version_info.major == 2
assert sys.version_info.minor == 7
assert sys.version_info.micro == 17

class Contour:

	def __init__(self,im_bgr):
		"""
		Constructeur
		Parametre : image BGR d'une unique forme, binarisee (noir = hors forme, blanc = forme)
		"""
		assert im_bgr is not None, "empty image"
	
		assert (len(im_bgr.shape) == 3 and im_bgr.shape[2] == 3),"probleme dimensions, image BGR ?"
		self.__im_bgr = im_bgr
		self.__im_grey = cv2.cvtColor(self.__im_bgr,cv2.COLOR_BGR2GRAY)
		ret,im_thresh = cv2.threshold(self.__im_grey,127,255,cv2.THRESH_BINARY)
		self.__im_thresh = im_thresh
		
		#selon documentation : moments sur contours, ou sur image gris... faire un choix
		#self.__moments = cv2.moments(self.getContour())
		self.__moments = cv2.moments(self.__im_thresh,binaryImage=True)

		contours,hierarchy = cv2.findContours(self.__im_thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)[-2:]


		self.__contour = contours



		

	def getContour(self):
		return self.__contour

	def getImGrey(self):
		return self.__im_grey

	def getImThresh(self):
		return self.__im_thresh

	def getMoments(self):
		return self.__moments

	def getAire(self):
		"""aire de la surface definie par le contour, en pixels"""
		return np.round(self.getMoments()['m00']).astype("int")

	def getCentre(self):
		"""tuple (ligne, colonne) de la position du centre de la surface definie par le contour, en pixels"""
		return (self.__getLigneCentre(),self.__getColonneCentre())

	def getLigneCentre(self):
		return np.round((self.getMoments()['m10'])/(self.getMoments()['m00'])).astype("int")

	def getColonneCentre(self):
		return np.round((self.getMoments()['m01'])/(self.getMoments()['m00'])).astype("int")

class Cercle(Contour):

	def __init__(self,im_bgr):
		Contour.__init__(self,im_bgr)
		self.__x_circle,self.__y_circle,self.__r_circle = self.findCircle()

	def findCircle(self):
		"""
		out : [x,y,rayon] cercle obtenu par transformation de hough
		"""
		(x,y),radius = cv2.minEnclosingCircle(self.getContour()[0])
		return int(np.round(x)),int(np.round(y)),int(np.round(radius))
		
	def getXCircle(self):
		return self.__x_circle

	def getYCircle(self):
		return self.__y_circle

	def getRCircle(self):
		return self.__r_circle

	def getCircle(self):
		return [self.__x_circle,self.__y_circle,self.__r_circle]

class analyseContour:

	@staticmethod
	def __contour(im_bgr):
		"""
		in : image RGB de la forme 0/255
		out : liste des points formant le contour, abscisse centre contour, ordonnee centre contour, aire du contour
		"""
		contour = Contour(im_bgr)
		
		#print("moments : {}".format(contour.getMoments()))
		return contour.getContour(),contour.getLigneCentre(),contour.getColonneCentre(),contour.getAire(),contour.getMoments()

	@staticmethod
	def __cercle(im_bgr):
		"""
		in : image rgb binaire
		out : [x,y,rayon] cercle obtenu par hough
		"""
		cercle = Cercle(im_bgr)
		return cercle.getCircle()
	
	@staticmethod
	def caracterization(image,rayon,px_to_mm,affichage=False):
		"""
		fonction caracterization
		in :
		-image (numpy array order 3) binaire 0/255 du contour UNIQUE du trou (trou a 1 valeur, reste a une autre valeur)
		-rayon suppose du trou dans image
		-affichage : booleen pour creer fenetre affichage des differentes etapes
		out :
		- isdefective : booleen (vrai si defaut present)
		- defect : string contenant une forme/defaut reconnue
		- contours : ensemble des pixels formant le contour
		"""
		isdefective = False
		defect = ""
		circle = analyseContour.__cercle(image)
		contours,xbar,ybar,aire,moments = analyseContour.__contour(image)
		#caracterisation :
		minprct =  0.85
		maxprct =  1.15
		#entre hough et les contours (la forme est-elle obstruee ou fissuree ?)
		isDeforme = False
		if (aire < minprct*np.pi*((circle[2])**2)):
			print(aire," vs ",minprct*np.pi*((circle[2])**2))
			isdefective = True
			defect += "Obstruction\n"
		if (aire > maxprct*np.pi*((circle[2])**2)):
			print(aire," vs ",maxprct*np.pi*((circle[2])**2))
			isdefective=True			
			defect += "Fissure\n"
		
		if(moments['nu20']< minprct*1/(4*np.pi) or moments['nu20']>1.04*1/(4*np.pi)):
			isdefective = True
			isDeforme = True
			

		if(moments['nu02']< minprct*1/(4*np.pi) or moments['nu02']>1.04*1/(4*np.pi)):
			isdefective = True
			isDeforme = True   

		if(abs(moments['nu11'])>10**-2):
			isdefective=True
			isDeforme = True

		if(abs(moments['nu21'])>10**-3):
			isdefective=True
			isDeforme = True

		if(abs(moments['nu12'])>10**-3):
			isdefective=True
			isDeforme = True

		if(abs(moments['nu30'])>10**-3):
			isdefective=True
			isDeforme = True

		if(abs(moments['nu03'])>10**-3):
			isdefective=True
			isDeforme = True

		if(isDeforme):
			defect += "Deformation\n"
		#entre hough et le parametre de la fonction (le rayon en parametre est-il correct ?)
		minprct = 0.85
		maxprct = 1.15

		if (circle[2]*minprct>rayon):
			isdefective=True
			defect += "Rayon trop petit"
		elif(rayon>circle[2]*maxprct):
			isdefective=True
			defect += "Rayon trop grand"
		

		if affichage:
			cv2.imshow('original image',image)
			cv2.waitKey(0)
			output = image.copy()
			cv2.circle(output,(circle[0],circle[1]),circle[2],(0,0,255),1) #dessine cercle
			cv2.imshow('transfo hough',output)
			cv2.waitKey(0)
			output = np.zeros(image.shape)
			output = cv2.drawContours(image=output,contours=Contour(image).getContour(),contourIdx=-1,color=(0,0,255))
			cv2.imshow('contours',output)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
		return isdefective, defect, contours

if __name__ == "__main__":
	image = cv2.imread('analyse_contour/81radius.jpg')
	defect,resultat = analyseContour.caracterization(image,81,True)
	print(defect)
	print(resultat)
