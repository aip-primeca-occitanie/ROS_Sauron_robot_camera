from deplacement_robot.srv import Robot_move_predef
import rospy
from communication.srv import capture
from deplacement_robot.srv import Robot_move_predef,Move_predef
from cv_bridge import CvBridge
import cv2 as cv
from calib_finale import calibration


def run_calibration(pub=None):
    save_images()
    try:
        matrix_intr, distortion, r_vecs, t_vecs=calibration("../images_calib/")
        return matrix_intr, distortion, r_vecs, t_vecs
    except TypeError:
        pub_state(pub,"CALIBRATION FAILED")
    
def pub_state(pub, msg):
    if not pub is None:
        pub.publish(msg)

def move_to_point(p,pub=None):
    move_robot = rospy.ServiceProxy("move_predef",Move_predef)
    pub_state(pub,"moving to point "+str(p))
    move_robot("calibration_"+str(p))
    

def get_image():
    #capturer l'image
    bridge=CvBridge()
    capture_image = rospy.ServiceProxy("camera/capture", capture)
    img = capture_image()
    rosimage = img.image
    cv_image = bridge.imgmsg_to_cv2(rosimage,'bgr8')
    #print("shape = ",cv_image.shape)
    assert (len(cv_image.shape) == 3),"(1) probleme dimensions, image BGR ?"
    return cv_image

def save_images():
    for i in range(10):
        move_to_point(i+1)
        img = get_image()
        cv.imwrite("../images_calib/image"+str(i+1)+".bmp",img)



if __name__ == "__main__":
    rospy.init_node('test_calibration', anonymous=True)
    matrix_intr, distortion, r_vecs, t_vecs = run_calibration()