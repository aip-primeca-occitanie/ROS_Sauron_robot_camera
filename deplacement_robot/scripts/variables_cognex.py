from communication.msg import Image, Liste_points
from cv_bridge import CvBridge

class Variables:

	def __init__(self):
		# Publisher to be used
		self.pub_ok = None
		self.bridge = CvBridge()
		
		# Data structures to be used
		self.originale = Image()
		self.annotee = Image()
		self.points = Liste_points()

		# Telnet
		self.tn = None
		# FTP
		self.ftp = None
		# Camera IP
		self.ip = "192.168.1.100"
		# User credentials
		self.user = 'admin'
		self.password = ''

		# Process parameters
		self.identification = True
		self.opencv = True
		self.render = False
		self.plaque = "Plate" # Plate, Courbee, Lourde, False

		# Circle identification parameters
		self.blur = 5
		self.dp = 1
		self.minDist = 150
		self.p1 = 100
		self.p2 = 19
		self.minR = 0
		self.maxR = 60
		self.gain = 0

		# (min, max) of each circle type
		self.types = [(9, 13), (14, 19), (25, 31), (40, 47)]
		# Associated colors
		self.colors = [(0,255,0), (255,0,0), (255,255,0), (255,0,255)]
		# Associated sizes
		self.sizes = [5, 7, 12, 18]

		# Resize factor used to display the image in the window
		self.scale_percent = 0.4

	# Return type of the circle depending on the radius
	def get_type(self, rayon):
		i = 1
		for dim in self.types:
			if(rayon>=dim[0] and rayon<=dim[1]):
				return i
			i += 1
		return -1
	
	# Return color of the circle depending on the radius
	def get_color(self, rayon):
		if(self.get_type(rayon) != -1):
			color = self.colors[self.get_type(rayon)-1]
		else:
			color = (0,0,255)

		return color
	
	# Return type of the circle depending on the radius
	def get_size(self, rayon):
		i = 0
		for dim in self.types:
			if(rayon>=dim[0] and rayon<=dim[1]):
				return self.sizes[i]
			i += 1
		return -1
