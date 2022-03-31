# needed libraries
from linecache import getline
from random import random
import cv2 as cv
from cv2 import SORT_DESCENDING
from cv2 import sort
from cv2 import rotate
from matplotlib import lines
import numpy as np
import math 
import matplotlib.pyplot as plt
import extractHoles


def rotate2dPoints(points, alpha):
    # angle of rotation 10 degrees
    alpha = -alpha*math.pi/180
    # calculating "center of gravity" = rotation point 
    mean = np.mean(points,axis=0)
    X_mean = mean[0]
    Y_mean = mean[1]
    # subtracting mean from original coordinates and saving result to X_new and Y_new 
    X_new = []
    Y_new = []
    for i in range(len(points)):
        X_new.append(points[i,0] - X_mean)
        Y_new.append(points[i,1] - Y_mean)
    # rotating coordinates from which mean has been subtracted
    X_apu = []   #temporary help variable
    Y_apu = []   #temporary help variable
        
    for i in range(len(points)):
        X_apu.append(math.cos(alpha)*X_new[i]-math.sin(alpha)*Y_new[i])
        Y_apu.append(math.sin(alpha)*X_new[i]+math.cos(alpha)*Y_new[i])
            
    # adding mean back to rotated coordinates
    X_new = X_apu + X_mean
    Y_new = Y_apu + Y_mean

    newPoints = np.zeros([len(X_new),2])
    newPoints[:,0] = X_new
    newPoints[:,1] = Y_new
    return newPoints

def generateNewGrid(deletion = False, amountToDelete = 5):
    X = []
    Y = []
    startX = random() * 1000
    startY = random() * 1000
    step = 10

    for i in range(8):
        for j in range(5):
            X.append(startX + i*step)
            Y.append(startY + j*step)
    newPoints = np.zeros([len(X),2])
    newPoints[:,0] = X
    newPoints[:,1] = Y
    np.random.shuffle(newPoints)
    if deletion:
        for i in range(amountToDelete):
            newPoints = np.delete(newPoints,0,axis=0)
    return newPoints

def sortPoints(points, sortAxis = 0):
    sortedPoints = np.zeros([0,len(points[0,:])])
    while len(points) > 0:
        foundIndex = -1
        temp=-1000
        for i in range(len(points)):
            if points[i,sortAxis] >= temp:
                temp = points[i,sortAxis]
                foundIndex=i

        sortedPoints=np.vstack((sortedPoints,points[foundIndex,:]))

        points = np.delete(points,foundIndex,axis=0)
    return sortedPoints

def invSortPoints(points, sortAxis = 0):
    sortedPoints = np.zeros([0,len(points[0,:])])
    while len(points) > 0:
        foundIndex = -1
        temp=+100000000
        for i in range(len(points)):
            if points[i,sortAxis] <= temp:
                temp = points[i,sortAxis]
                foundIndex=i
        if foundIndex != -1:
            
            sortedPoints=np.vstack((sortedPoints,points[foundIndex,:]))
        points = np.delete(points,foundIndex,axis=0)
    return sortedPoints

def invSortPointsKeepOG(points, originals,sortAxis = 0):
    sortedPoints = np.zeros([0,len(points[0,:])])
    sortedOGPoints = np.zeros([0,len(points[0,:])])
    tempOriginals = originals.copy()
    while len(points) > 0:
        foundIndex = -1
        temp=+100000000
        for i in range(len(points)):
            if points[i,sortAxis] <= temp:
                temp = points[i,sortAxis]
                foundIndex=i
        if foundIndex != -1:
            
            sortedPoints=np.vstack((sortedPoints,points[foundIndex,:]))
            sortedOGPoints=np.vstack((sortedOGPoints,tempOriginals[foundIndex,:]))
        points = np.delete(points,foundIndex,axis=0)
        tempOriginals= np.delete(tempOriginals,foundIndex,axis=0)
    return sortedPoints, sortedOGPoints

def sortPointsKeepOG(points, originals,sortAxis = 0):
    sortedPoints = np.zeros([0,len(points[0,:])])
    sortedOGPoints = np.zeros([0,len(points[0,:])])
    tempOriginals = originals.copy()
    while len(points) > 0:
        foundIndex = -1
        temp=-100000000
        for i in range(len(points)):
            if points[i,sortAxis] >= temp:
                temp = points[i,sortAxis]
                foundIndex=i
        if foundIndex != -1:
            
            sortedPoints=np.vstack((sortedPoints,points[foundIndex,:]))
            sortedOGPoints=np.vstack((sortedOGPoints,tempOriginals[foundIndex,:]))
        points = np.delete(points,foundIndex,axis=0)
        tempOriginals= np.delete(tempOriginals,foundIndex,axis=0)
    return sortedPoints, sortedOGPoints

def hough(image):
    p1 = 100
    p2 = 20
    blur = 5
    dp = 1
    minDist = 150
    
    img = image.copy()

    imgGray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    imgGray = cv.medianBlur(imgGray, blur)

    circles = cv.HoughCircles(imgGray, cv.HOUGH_GRADIENT, dp, minDist,
                                param1=p1,param2=p2,minRadius=0,maxRadius=100)

    if( not (circles is None)):
        circles = np.uint16(np.around(circles))

    return circles[0,:,:2]

def findMarkPosition(image, debug = False):
    im = image.copy()
    bordersize = 10
    im = cv.copyMakeBorder(
        im,
        top=bordersize,
        bottom=bordersize,
        left=bordersize,
        right=bordersize,
        borderType=cv.BORDER_CONSTANT,
        value=255
    )

    _,im = cv.threshold(im,50,256,cv.THRESH_BINARY)
    params = cv.SimpleBlobDetector_Params() 

    params.minDistBetweenBlobs = 10
    params.minRepeatability = 1
    params.minThreshold = 0
    params.maxThreshold = 10
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 500000

    params.filterByCircularity = True
    params.minCircularity = 0.4
    params.maxCircularity = 0.90

    params.filterByConvexity = False
    params.minConvexity = 0.87

    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    
    detector = cv.SimpleBlobDetector_create(params)
    keypoints = detector.detect(im)
    if debug:
        # Show keypoints
        im_with_keypoints = cv.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        im_with_keypoints = cv.resize(im_with_keypoints,(1000,1000))
        cv.imshow("Keypoints", im_with_keypoints)
        cv.waitKey(0)
    if len(keypoints) != 0: 
        x = keypoints[0].pt[0]-bordersize
        y = keypoints[0].pt[1]-bordersize
        return [x,y]
    else:
        return (0,0)

def getHoughLines(image):
    """
    in :
        img : Path vers une image
    out:
        lines : array de N lignes {x1,y1,x2,y2}
        Return None si erreur de lecture de l'image
    Fonction generant les lignes Probabilistes de Hough
    """
    
    # Loads an image
    src = image.copy()
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        return None
    

    imgEdges = cv.Canny(src, 50, 200, None, 3)
    lines = cv.HoughLinesP(imgEdges, 1, np.pi / 180, 50, 2, 200, 200)
    #Enlever une dimension inutile de HoughLinesP pour faciliter l'utilisation de lines
    (x,y,z) = lines.shape
    lines = np.resize(lines, (x,z))
    i = 0
    while i < len(lines):
        if abs(lines[i,0]-lines[i,2])+abs(lines[i,1]-lines[i,3]) < 100:
            lines = np.delete(lines,i,axis=0)
        else:
            i+=1
    return lines

def getCenter(line):
    return ((int(line[0])+int(line[2]))/2, (int(line[1])+int(line[3]))/2)

def displayImgWithLines(img, lines, title):
    """
    in :
        img : Path vers une image
        lines : Array des lignes detectees sur une image 
        title : Titre de la fenetre d'affichage
    out:
        None
    Fonction affichant une image avec les lignes detectees dessus
    Bloque le deroulement jusqu'a la pression d'une touche du clavier
    """

    # Loads an image
    src = cv.imread(img, cv.IMREAD_GRAYSCALE)
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        return -1   
    src = cv.cvtColor(src,cv.COLOR_GRAY2RGB)

    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i]
            cv.line(src, (int(l[0]), int(l[1])), (int(l[2]), int(l[3])), (0,0,255), 1, cv.LINE_AA)
            cv.circle(src,getCenter(l),5,(0,255,0))  

    src = cv.resize(src,(1000,1000))        
    cv.imshow(title,src)
    cv.waitKey()

def getLineEquation(line):

    if line[2]==line[0] :
        slope = -1
        b = -1   
    else:
        slope = (line[3]-line[1]) / (line[2]-line[0])
        b = line[1] - line[0]*slope

    return slope, b 

def getPerpendicularSlope(slope):
    if slope == -1:
        return 0
    elif slope == 0:
        return -1
    else:
        return -1/slope

def getSearchZone(line, slopePerp):
    line1_B = line[1] - line[0]*slopePerp
    line2_B = line[3] - line[2]*slopePerp
    
    return np.sort([line1_B, line2_B])

def isMarkCloseby(line, mark):
    
    slope, posAtOrigin = getLineEquation(line)
    slopePerp = getPerpendicularSlope(slope)
    if slopePerp == -1:
        if mark[1] <= max((line[1],line[3])) and mark[1] >= min((line[1],line[3])):
            return True
        else:
            return False

    l_low,l_high = getSearchZone(line,slopePerp)

    if mark[1] <= mark[0]*slopePerp + l_high and mark[1] >= mark[0]*slopePerp + l_low:
        return True
    else:
        return False

def getLineDistanceToMark(line,mark):

    a,b=getLineEquation(line)
    if a == -1:
        return abs(mark[0]-line[0])
    return abs(mark[1] - a*mark[0] - b)/np.sqrt(1+a**2)

def displayUniqueLine(image,foundLine,title):
    src = image.copy()
    src = cv.cvtColor(src,cv.COLOR_GRAY2RGB)
    cv.line(src, (int(foundLine[0]), int(foundLine[1])), (int(foundLine[2]), int(foundLine[3])), (0,0,255), 5, cv.LINE_AA)
    cv.circle(src,getCenter(foundLine),5,(0,255,0))  
    src = cv.resize(src,(1000,1000))        
    cv.imshow(title,src)
    cv.waitKey()

def detectClosestEdge(image, mark):
    lines = getHoughLines(image)
    #displayImgWithLines(image,lines,"Before Traitement")
    i = 0
    while i < len(lines):
        if not isMarkCloseby(lines[i], mark):
            lines = np.delete(lines,i,axis=0)
        else:
            i+=1
    #displayImgWithLines(image,lines,"afterwards")
    
    foundLineDistance = 9999999
    foundLine = None

    for currentLine in lines:
        currentDistance = getLineDistanceToMark(currentLine, mark)


        if currentDistance<foundLineDistance:
            foundLine = currentLine
            foundLineDistance = currentDistance     
    if foundLine is None:
        print("not found")
        return None
    else:
        #displayUniqueLine(image,foundLine,"LineFound")
        return foundLine

def getLinesToFind(holeList):

    
    sortedList = sortPoints(holeList, 1)

    newList = np.empty((0,3))
    finishedLists=[]
    for point in sortedList:
        if len(newList) == 0:
            newList=np.vstack([newList,point])
            continue

        if(abs(newList[-1,1] - point[1]) <= 2):
            newList=np.vstack([newList,point])
            continue
        else:
            sortNewList = invSortPoints(newList, 0) 
            finishedLists.append(sortNewList)
            newList = np.empty([0,3])
            newList=np.vstack((newList,point))
            continue
    finishedLists.append(invSortPoints(newList, sortAxis=0))

    outValues = np.array(finishedLists)
    return outValues

def getLinesFromCloud(cloud, original):
    [cloud, original] = invSortPointsKeepOG(cloud, original, 1)

    newList = np.empty((0,2))
    ogList =np.empty((0,2))
    finishedLists=[]

    for i in range(len(cloud)):
        if len(newList) == 0:
            newList=np.vstack([newList,cloud[i,:]])
            ogList =np.vstack([ogList,original[i,:]])
            continue

        if(abs(newList[-1,1] - cloud[i,1]) <= 20):
            newList=np.vstack([newList,cloud[i,:]])
            ogList =np.vstack([ogList,original[i,:]])
            continue
        else:
            [_,newOgList] = invSortPointsKeepOG(newList, ogList, 0) 
            finishedLists.append(newOgList)
            newList = np.empty([0,2])
            ogList = np.empty([0,2])
            newList=np.vstack((newList,cloud[i,:]))
            ogList=np.vstack((ogList,original[i,:]))
            continue

    [newList,ogList]  = invSortPointsKeepOG(newList, ogList, 0)
    finishedLists.append(ogList)

    outValues = np.array(finishedLists)



    return outValues


def displayPointCloudOnImg(image, cloud):
    plt.imshow(image)
    plt.scatter(cloud[:,0], cloud[:,1], c='b', marker='x', label='1')
    plt.show(block=False)
    plt.waitforbuttonpress()

def addMarkAndLineToCloud(cloud, mark, line): #useful to rotate stuff around
    newStuff = np.vstack([cloud.copy(), mark])
    newStuff = np.vstack([newStuff, [line[0],line[1]]])
    newStuff = np.vstack([newStuff, [line[2],line[3]]])
    return newStuff

def separatePointsAndOthers(cloud): #When it's rotated, we take them out for easy parsing
    newCloud = cloud.copy()
    point1 = newCloud[-1,:]
    newCloud = np.delete(newCloud,-1,axis=0)
    point2 = newCloud[-1,:]
    newCloud = np.delete(newCloud,-1,axis=0)
    mark = newCloud[-1,:]
    newCloud = np.delete(newCloud,-1,axis=0)
    return newCloud, mark,[point1[0],point1[1],point2[0],point2[1] ]

def generatePointLines(image, detectedPoints, holeList):
    #Trouver les lignes dans le modele 3d
    linesFrom3D = getLinesToFind(holeList)

    #Chopper les trous 2D
    if detectedPoints is not None:
        originalPoints = detectedPoints
    else:
        originalPoints = hough(image)
    
    mark = findMarkPosition(image)
    foundLine = detectClosestEdge(image, mark)
    

    #On garde les originaux dans le coin pour la fin
    workingPointSet = originalPoints.copy()
    
    #On peut commencer a mettre les trucs droits, d'abord on s'arrange pour avoir la ligne directrice a l'horizontale
    stop = False
    while not stop:
        [slope,b] = getLineEquation(foundLine)
        if slope == -1:
            #Si la ligne est parfaitement verticale, rotate 90] and stop
            workingPointSet = addMarkAndLineToCloud(workingPointSet, mark, foundLine)
            workingPointSet = rotate2dPoints(workingPointSet, 90)
            [workingPointSet,mark, foundLine] = separatePointsAndOthers(workingPointSet)
            stop = True
        elif abs(slope)>=0.0000005:
            workingPointSet = addMarkAndLineToCloud(workingPointSet, mark, foundLine)
            workingPointSet = rotate2dPoints(workingPointSet, slope)
            [workingPointSet,mark, foundLine] = separatePointsAndOthers(workingPointSet)
        else:
            stop = True

    #A partir d'ici le truc est droit, c'est chouette mais il faut checker si la marque est au dessus ou en dessous, on veut la marque au dessus, donc potentiel 180 degre
    if mark[1] >= max([foundLine[1],foundLine[3]]):
        workingPointSet = addMarkAndLineToCloud(workingPointSet, mark, foundLine)
        workingPointSet = rotate2dPoints(workingPointSet, 180)
        [workingPointSet,mark, foundLine] = separatePointsAndOthers(workingPointSet)

    newStuff=getLinesFromCloud(workingPointSet,originalPoints)

    return linesFrom3D,newStuff


def removeNonSimilarLines(lines2D, lines3D):
    i = 0
    while i < (len(lines2D)):
        if len(lines2D[i]) != len(lines3D[i]):
            lines2D = np.delete(lines2D, i, axis=0)
            lines3D = np.delete(lines3D, i, axis=0)
        else:
            i+=1

    return lines2D,lines3D



def formatPointsForPnP(lines2D,lines3D):
    new2D = np.empty((0,2))
    new3D = np.empty((0,3))
    for i in range(len(lines2D)):
        new2D = np.vstack([new2D,lines2D[i]])
        new3D = np.vstack([new3D,lines3D[i]])
    return new2D,new3D

if __name__=="__main__":
    with open("Data/Plaque1/Model/Plaque_1.stp") as f:
            file = f.readlines()
    object_points = extractHoles.getAllCircles(file)
    object_points = np.delete(object_points, 3, axis=1)

    lines3D, lines2D = generatePointLines(cv.imread("HoleDetection/ShittyDataset/1.bmp"),  None, object_points)

    """Uncomment pour afficher les lignes 1 par 1 pour debug"""
    #for i in range(len(lines2D)):
    #    displayPointCloudOnImg("HoleDetection\ShittyDataset\\1.bmp",lines2D[i])
    #    displayPointCloudOnImg("HoleDetection\ShittyDataset\\1.bmp",lines3D[i])

    #CRUDE MATCHING, on prends vraiment juste les lignes qui font la meme taille
    lines2D,lines3D = removeNonSimilarLines(lines2D,lines3D)
    readyForPnP_2D,readyForPnP_3D = formatPointsForPnP(lines2D,lines3D)

    print(readyForPnP_2D.shape, readyForPnP_3D.shape)
    
