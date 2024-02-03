import numpy as np
from scipy.special import gamma
import os

def direct_sphere(d,r_i=0,r_o=1):
    """Direct Sampling from the d Ball based on Krauth, Werner. Statistical Mechanics: Algorithms and Computations. Oxford Master Series in Physics 13. Oxford: Oxford University Press, 2006. page 42

    Parameters
    ----------
    d : int
        dimension of the ball
    r_i : int, optional
        inner radius, by default 0
    r_o : int, optional
        outer radius, by default 1

    Returns
    -------
    np.array
        random vector directly sampled from the solid d Ball
    """
    # vector of univariate gaussians:
    rand=np.random.normal(size=d)
    # get its euclidean distance:
    dist=np.linalg.norm(rand,ord=2)
    # divide by norm
    normed=rand/dist
    
    # sample the radius uniformly from 0 to 1 
    rad=np.random.uniform(r_i,r_o**d)**(1/d)
    # the r**d part was not there in the original implementation.
    # I added it in order to be able to change the radius of the sphere
    # multiply with vect and return
    return normed*rad

def sample_from_ellipsoid(indexes,M,rho,r_i=0,r_o=1):
    """sample directly from the ellipsoid defined by xT M x.

    Parameters
    ----------
    M : np.array
        Matrix M such that xT M x leq rho defines the hyperellipsoid to sample from
    rho : float
        rho such that xT M x leq rho defines the hyperellipsoid to sample from
    r_i : int, optional
        inner radius, by default 0
    r_o : int, optional
        outer radius, by default 1

    Returns
    -------
    np.array
        random vector from within the hyperellipsoid
    """
    idx0 = indexes[0]
    idx1 = indexes[1]
    M = np.array([[M[idx0,idx0],M[idx0,idx1]],
                  [M[idx1,idx0],M[idx1,idx1]]])
    lamb,eigV=np.linalg.eigh(M/rho) 
    d=len(M)
    xy=direct_sphere(d,r_i=r_i,r_o=r_o) #sample from outer shells
    T=np.linalg.inv(np.dot(np.diag(np.sqrt(lamb)),eigV.T)) #transform sphere to ellipsoid (refer to e.g. boyd lectures on linear algebra)
    return np.dot(T,xy.T).T

def sampleFromEllipsoid(S,rho,rInner=0,rOuter=1):
    lamb,eigV=np.linalg.eigh(S/rho) 
    d=len(S)
    xy=direct_sphere(d,r_i=rInner,r_o=rOuter) #sample from outer shells
    T=np.linalg.inv(np.dot(np.diag(np.sqrt(lamb)),eigV.T)) #transform sphere to ellipsoid (refer to e.g. boyd lectures on linear algebra)
    return np.dot(T,xy.T).T

def quad_form(M,x):
    """
    Helper function to compute quadratic forms such as x^TMx
    """
    return np.dot(x,np.dot(M,x))

def projectedEllipseFromCostToGo(s0Idx,s1Idx,rho,M):
    """
    Returns ellipses in the plane defined by the states matching the indices s0Idx and s1Idx for funnel plotting.
    """
    ellipse_widths=[]
    ellipse_heights=[]
    ellipse_angles=[]
    
    #loop over all values of rho
    for idx, rho in enumerate(rho):
        #extract 2x2 matrix from S
        S=M[idx]
        ellipse_mat=np.array([[S[s0Idx][s0Idx],S[s0Idx][s1Idx]],
                              [S[s1Idx][s0Idx],S[s1Idx][s1Idx]]])*(1/rho)
        
        #eigenvalue decomposition to get the axes
        w,v=np.linalg.eigh(ellipse_mat) 

        try:
            #let the smaller eigenvalue define the width (major axis*2!)
            ellipse_widths.append(2/float(np.sqrt(w[0])))
            ellipse_heights.append(2/float(np.sqrt(w[1])))

            #the angle of the ellipse is defined by the eigenvector assigned to the smallest eigenvalue (because this defines the major axis (width of the ellipse))
            ellipse_angles.append(np.rad2deg(np.arctan2(v[:,0][1],v[:,0][0])))
        except:
            continue
    return ellipse_widths,ellipse_heights,ellipse_angles

def vol_ellipsoid(rho,M):
    """
    Calculate the Volume of a Hyperellipsoid
    Volume of the Hyperllipsoid according to https://math.stackexchange.com/questions/332391/volume-of-hyperellipsoid/332434
    Intuition: https://textbooks.math.gatech.edu/ila/determinants-volumes.html
    Volume of n-Ball https://en.wikipedia.org/wiki/Volume_of_an_n-ball
    """
    
    # For a given hyperellipsoid, find the transformation that when applied to the n Ball yields the hyperellipsoid
    lamb,eigV=np.linalg.eigh(M/rho) 
    A=np.dot(np.diag(np.sqrt(lamb)),eigV.T) #transform ellipsoid to sphere
    detA=np.linalg.det(A)
    
    # Volume of n Ball (d dimensions)
    d=M.shape[0] # dimension 
    volC=(np.pi**(d/2))/(gamma((d/2)+1))

    # Volume of Ellipse
    volE=volC/detA

    return volE

def getEllipseFromCsv(csv_path, index):

    data = np.loadtxt(csv_path, skiprows=1, delimiter=",").T

    rho = data[0].T[index]

    M = data[1:len(data)].T[index]
    state_dim = int(np.sqrt(len(data)-1))
    M = np.reshape(M,(state_dim,state_dim))

    return rho, M

def getEllipseContour(rho, M,x0, nSamples = 1000):
    """
    Returns a certain number(nSamples) of sampled states from the contour of a given ellipse.

    Parameters
    ----------
    M : np.array
        Matrix S that define one ellipse
    rho : np.array
        rho value that define one ellipse
    x0 : np.array
        center of the ellipse

    Returns
    -------
    c : np.array
        random vector of states from the contour of the ellipse
    """
    c = sampleFromEllipsoid(M,rho,rInner = 0.99) +x0
    for i in range(nSamples-1):
        xBar = sampleFromEllipsoid(M,rho,rInner = 0.99)
        c = np.vstack((c,xBar+x0))
    return c

from scipy.spatial import ConvexHull

def ellipseVolume_convexHull(rho, M, x0):
    '''
    Function to calculate the volume of the final ellipse. This implementation makes use of the convex hull concept
    as done in the MATLAB code of the Robot Locomotion Group (https://groups.csail.mit.edu/locomotion/software.html).
    Parameters
    ----------
    M : np.array
        Matrix S that define one ellipse
    rho : np.array
        rho value that define one ellipse
    x0 : np.array
        center of the ellipse
    '''
    c = getEllipseContour(rho,M, x0, nSamples=1000)
    points = c

    # plot the convex hull of the two contours
    hull_i = ConvexHull(points) 
    V = hull_i.volume
    
    return V

def storeEllipse(rho, M, estMethod = "",RoA_path = None):
    M = np.array(M).flatten()

    log_dir = "data/cart_pole/RoA"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    csv_data = np.array([np.append([rho],M)])

    if RoA_path is None:
        csv_path = os.path.join(log_dir, estMethod + f"RoA.csv")
    else:
        csv_path = RoA_path
    np.savetxt(csv_path, csv_data, delimiter=',',
            header="rho,S", comments="")

def storeFunnel(M,rho,T, RoA_path = None, estMethod = ""):
    log_dir = "data/cart_pole/RoA"
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    S_t = np.zeros((len(rho),16))
    for i in range(len(rho)):
        S_t[i] = np.array(M.value(T[i])).flatten()

    rho = np.reshape(rho, (len(rho),1))
    csv_data = np.hstack((rho,S_t))

    if RoA_path is None:
        csv_path = os.path.join(log_dir, estMethod + f"RoA.csv")
    else:
        csv_path = RoA_path
    np.savetxt(csv_path, csv_data, delimiter=',',
            header="rho,S", comments="")

def funnelVolume_convexHull(funnel_path, traj_path):
    #load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0 = np.array([trajectory.T[1].T, trajectory.T[2].T, trajectory.T[3].T, trajectory.T[4].T])

    vol = 0
    for i in range(len(time)):
        rho, M = getEllipseFromCsv(funnel_path, i)
        vol += ellipseVolume_convexHull(rho,M,x0.T[i])

    return vol
