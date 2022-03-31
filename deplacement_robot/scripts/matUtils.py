from matplotlib import projections
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import cv2
import numpy as np
from numpy import isclose
     

def isRotationMatrix(R,rtol=1.e-06):
    assert(R.shape == (3,3)),"Rotation matrix must be 3x3"
    assert(isclose(np.linalg.det(R),1,rtol=rtol)),"Rotation matrix must have it's determinant equal to 1"
    assert np.isclose(np.linalg.inv(R),R.T,rtol).all(),"Rotation matrix must be orthogonal"   
    
def R_from_vect(vec):
    """
    Cree matrice coordonnees homogenes
    input : vec (tx,ty,tz,rx,ry,rz)
    rx,ry,rz : vecteur autour duquel a tourne le repere (et angle = norme vecteur)
    output : matrice R3*3|T3*1
                     01*3|1
    """
    #assert(vec.shape==(6L,1L))
    
    v1 = vec[0]
    v2 = vec[1]
    v3 = vec[2]
    x = vec[3]
    y = vec[4]
    z = vec[5]

    matPos = np.eye(4)

    matPos[0, 3] = x
    matPos[1, 3] = y
    matPos[2, 3] = z
    
    matPos[0:3,0:3] = cv2.Rodrigues(np.array([[v1],[v2],[v3]]))[0]
    isRotationMatrix(matPos[0:3,0:3])
    return matPos
    
def bryant_to_R(angle):
    """
    in : tuple of bryant angles (lambda,mu,nu) : (rot x,rot nouveau y,rot nouveau z)
    output : rotation matrix
    """
    assert(type(angle)==type(())),"type tuple expected"
    assert(len(angle)==3),"expected 3 angles"
    r1 = angle[0]
    r2 = angle[1]
    r3 = angle[2]
    s1=np.sin(r1)
    c1=np.cos(r1)
    s2=np.sin(r2)
    c2=np.cos(r2)
    s3=np.sin(r3)
    c3=np.cos(r3)
    R1 = np.array([ [1.,0.,0.],
                    [0.,c1,-s1],
                    [0.,s1,c1]])
    R2 = np.array([ [c2,0.,s2],
                    [0.,1.,0.],
                    [-s2,0.,c2]])
    R3 = np.array([ [c3,-s3,0.],
                    [s3,c3,0.],
                    [0.,0.,1.]])
    R = np.dot(R1,np.dot(R2,R3))
    isRotationMatrix(R)
    return R
    
    
def R_to_bryant(R):
    """
    transformation matrice rotation vers angles bryant
    input : np array 3*3
    output : tuple (lambda,mu,nu)
    bryant :
    1 : lambda = rotation autour de x d'un angle lambda
    2 : mu = rotation autour du nouveau y d'un angle mu
    3 : nu = rotation autour du nouveau z d'un angle nu
    -pi<lambda<=pi
    -pi/2<=mu<=pi/2
    -pi<nu<=pi
    """ 
    isRotationMatrix(R)
    
    #A FIXER (valeurs numeriquement superieures a 1 ou inferieures a -1 parfois, arcsin plante...)
    for i in range(3):
        for j in range(3):
            if(R[i:i+1,j:j+1] >1):
                R[i:i+1,j:j+1] = 1.
            if(R[i:i+1,j:j+1] <-1):
                R[i:i+1,j:j+1] = -1.

    isRotationMatrix(R)
    #hakim
    if (R[0:1,2:3]==1) or (R[0:1,2:3]==-1):
        print("warning : R13 = +/-1, nu set to 0")
        lamb = float(np.arctan2(R[1:2,0:1],R[1:2,1:2])/(R[0:1,2:3]))
        mu = float(np.arcsin(R[0:1,2:3]))
        nu = 0.
    else:
        lamb = float(-np.arctan2(R[1:2,2:3],R[2:3,2:3]))
        mu = float(np.arcsin(R[0:1,2:3]))
        nu = float(-np.arctan2(R[0:1,1:2],R[0:1,0:1]))
    return(lamb,mu,nu)

def transform_and_draw_model(edges_Ro, intrinsic, extrinsic, fig_axis):
    # ********************************************************************* #
    # A COMPLETER.                                                          #
    # UTILISER LES FONCTIONS :                                              #
    #   - perspective_projection                                            #
    #   - transform_point_with_matrix                                       #
    # Input:                                                                #
    #   edges_Ro : ndarray[Nx6]                                             #
    #             N = nombre d'aretes dans le modele                        #
    #             6 = (X1, Y1, Z1, X2, Y2, Z2) les coordonnees des points   #
    #                 P1 et P2 de chaque arete                              #
    #   intrinsic : ndarray[3x3] - parametres intrinseques de la camera     #
    #   extrinsic : ndarray[4x4] - parametres extrinseques de la camera     #
    #   fig_axis : figure utilisee pour l'affichage                         #
    # Output:                                                               #
    #   Pas de retour de fonction, mais calcul et affichage des points      #
    #   transformes (u1, v1) et (u2, v2)                                    #
    # ********************************************************************* #

    # A remplacer #
    #u_1 = np.zeros((edges_Ro.shape[0], 1))
    #u_2 = np.zeros((edges_Ro.shape[0], 1))
    #v_1 = np.zeros((edges_Ro.shape[0], 1))
    #v_2 = np.zeros((edges_Ro.shape[0], 1))
    ###############

    P1_cam = transform_point_with_matrix(extrinsic, edges_Ro[:,:3])
    P2_cam = transform_point_with_matrix(extrinsic, edges_Ro[:,3:])

    [u_1, v_1] = perspective_projection(intrinsic, P1_cam)
    [u_2, v_2] = perspective_projection(intrinsic, P2_cam)

    for p in range(edges_Ro.shape[0]):
        fig_axis.plot([u_1[p], u_2[p]], [v_1[p], v_2[p]], color='m')


def perspective_projection(intrinsic, P_c):
    # ***************************************************** #
    # A COMPLETER.                                          #
    # Fonction utile disponible :                           #
    #   np.dot                                              #
    # Input:                                                #
    #   intrinsic : ndarray[3x3] - parametres intrinseques  #
    #   P_c : ndarray[Nx3],                                 #
    #         N = nombre de points a transformer            #
    #         3 = (X, Y, Z) les coordonnees des points      #
    # Output:                                               #
    #   u, v : deux ndarray[N] contenant les                #
    #          coordonnees Ri des points P_c transformes    #
    # ***************************************************** #
    
    Z = P_c[:,2]
    [u,v,tmp] = (1/Z) * np.dot(intrinsic, P_c.T)



    return u, v


def transform_point_with_matrix(transformation_matrix, initial_point):
    initial_point_cpy = np.ones((initial_point.shape[0], 4))
    initial_point_cpy[:, 0:3] = np.copy(initial_point)

    transformed_point = np.dot(transformation_matrix, initial_point_cpy.T)

    return transformed_point[0:3, :].T


def plot_3d_model(model, fig, sub=111):
    ax = fig.add_subplot(sub, projection='3d')

    lines = []
    for idx in range(model.shape[0]):
        lines.append(ax.plot([model[idx, 0], model[idx, 3]],
                                [model[idx, 1], model[idx, 4]],
                                [model[idx, 2], model[idx, 5]],
                                color='k', picker=5)[0])
    ax.scatter(0, 0, 0, color='r', s=30)
    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    ax.set_zlabel('z (mm)')
    ax.set_title('3D model')

    return ax, lines
