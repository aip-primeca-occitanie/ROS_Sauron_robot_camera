
import numpy as np
import rospy
from variables_cognex import Variables
import cv2 as cv
from ftplib import FTP

global variables
variables = Variables()


def read_cognex(case):
    # Send the command with Telnet
    variables.tn.write(case+"\r\n")
    # Wait for the return code
    errorCode = str(variables.tn.read_until("1\r\n", timeout=3))
    # Read of the returned value if the command did not fail
    val = variables.tn.expect(["\r\n"], timeout=3)[2]
    val = val[:len(val)-2]
    # Log info to see what is happening
    rospy.loginfo("Com : "+case+" -- "+"Com OK: "+errorCode[0]+" -- "+"Com Val : "+val+"\r\n")
    return val

def get_image():
    if(variables.identification and not variables.opencv):
        # Send an event for holes detection and wait for it
        variables.tn.write("SW0\r\n")
        # Log info to see what is happening
        rospy.loginfo("Com : SW0 -- Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=6)[2]))
    # Send an acquisition event and wait for it
    variables.tn.write("SW8\r\n")
    # Log info to see what is happening
    rospy.loginfo("Com : SW8 -- Com OK: "+str(variables.tn.expect(["0\r\n", "1\r\n"], timeout=6)[2]))

    # Creation of the FTP connexion
    variables.ftp = FTP(variables.ip)
    variables.ftp.login(variables.user)

    # Download file from cognex
    filename = 'image.bmp'
    lf = open(filename, "wb")
    variables.ftp.retrbinary("RETR " + filename, lf.write)
    lf.close()
    img = cv.imread(filename)

    return img


def localisation(type, modele, photo, Mom, Moc, Mint, dist):
    # tole cintree, tole plate, tole epaisse  
    res = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    get_image()
    try:
        if(type == "tole plate"):
            angle = float(read_cognex("GVE160"))
            u = float(read_cognex("GVC160"))
            v = float(read_cognex("GVD160"))
        elif(type == "tole cintree"):
            angle = float(read_cognex("GVE178"))
            u = float(read_cognex("GVC178"))
            v = float(read_cognex("GVD178"))
        elif(type == "tole epaisse"):
            angle = float(read_cognex("GVE196"))
            u = float(read_cognex("GVC196"))
            v = float(read_cognex("GVD196"))
    except (ValueError, TypeError):
        return None
    
    Mcm = np.linalg.inv(Moc) * Mom
    s = Mcm[2,2]
    XYZ = (1/s) * np.linalg.inv(Mcm) * np.linalg.inv(Mint) * np.array([[u],[v],[1]])

    res[3:] = XYZ[:3]
    res[0] = 0.0
    res[1] = 0.0
    res[2] = np.radians(angle)

    return res
