import cv2

p1 = 100
p2 = 20
blur = 5
dp = 1
minDist = 150
minR = 0
maxR = 100

def hough(image):

    global p1, p2, blur, dp, minDist, minR, maxR

    try :
        img = image.copy()

        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        imgGray = cv2.medianBlur(imgGray, blur)

        circles = cv2.HoughCircles(imgGray, cv2.HOUGH_GRADIENT, dp, minDist,
                                    param1=p1,param2=p2,minRadius=minR,maxRadius=maxR)
        if circles is None:
            return None

        return circles[0,:,:2]
    
    except Exception as e:
        print(e)

