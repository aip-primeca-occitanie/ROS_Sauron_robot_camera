import numpy as np

def extractCircles(file):
    circles = []
    for currentLine in file:
        if "CYLINDRICAL_SURFACE" in currentLine:
            circles.append(currentLine)
    return circles

def findLineWithId(id, file):
    for line in file:
        if id+"=" in line:
            return line
    return None

def parseCircle(circle, file, debug=False):
    [id,circle] = circle.split("=")
    [_,circle] = circle.split("(")
    [circle,_] = circle.split(")")
    [_,centerId,diameter] = circle.split(",")
        
    foundLine = findLineWithId(centerId, file)
    
    parsed = foundLine.split(",")
    repereId = parsed[1]

    foundLine = findLineWithId(repereId, file)
    [_, _,position] = foundLine.split("(")
    [position, _,_] = position.split(")")
    [x, y, z] = position.split(",")

    

    positionPoint = (float(x),float(y),float(z))
    if debug:
        print("id : ", id)
        print("diameter : ", diameter)
        print("x y z : ",positionPoint)
    return positionPoint,float(diameter)



def getAllCircles(file, getBothFaces=False):
    Circles = extractCircles(file)
    results = np.zeros([len(Circles),4])


    for i in range(len(Circles)):
        [currentPos, currentD] = parseCircle(Circles[i],file)    
        if currentD < 50:
            results[i][0:3] = currentPos
            results[i][3] = currentD
    
    results = np.unique(results,axis=0) 


    if not getBothFaces:
        i = 0
        while i < len(results):
            if results[i][2] == 0:
                results = np.delete(results,i,axis=0)
            i+=1
    return results



if __name__ == '__main__':
    #Seul code a comprendre, is okay
    with open('./Data/Plaque3/Model/Plaque_3.stp') as f:
        file = f.readlines()

    results = getAllCircles(file, getBothFaces=False) #getBothFaces si vous voulez aussi les points qui correspondent au dessous de la plaque, probablement useless mais sait-on jamais
    print(results.shape)
    
    #with open('HoleDetection/Points3D/Plaque3.npy', 'wb') as f:
    #    np.save(f, results, allow_pickle=False)
    #print "positions : ", points_3D.shape
    #print points_3D
    #print "diametres : ", diameters.shape
    #print diameters

