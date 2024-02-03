import numpy as np
import matplotlib as mpl
mpl.use("WebAgg")
import matplotlib.pyplot as plt
from matplotlib import patches

def get_ellipse_params(idx0,idx1,rho,M):
    """
    Returns ellipse params (excl center point)
    """

    M = np.array([[M[idx0,idx0],M[idx0,idx1]],
                  [M[idx1,idx0],M[idx1,idx1]]])

    #eigenvalue decomposition to get the axes
    w,v=np.linalg.eigh(M/rho) 

    try:
        #let the smaller eigenvalue define the width (major axis*2!)
        width = 2/float(np.sqrt(w[0]))
        height = 2/float(np.sqrt(w[1]))
        
        #the angle of the ellipse is defined by the eigenvector assigned to the smallest eigenvalue (because this defines the major axis (width of the ellipse))
        angle = np.rad2deg(np.arctan2(v[:,0][1],v[:,0][0]))

    except:
        print("parameters do not represent an ellipse.")

    return width,height,angle

def get_ellipse_patch(idx0, idx1, xG,rho,M,alpha_val=1,linec="red",facec="none",linest="solid"):
    """
    return an ellipse patch
    """
    px = xG[idx0]
    py = xG[idx1]
    w,h,a = get_ellipse_params(idx0,idx1,rho,M)
    return patches.Ellipse((px,py), w, h, a, alpha=alpha_val,ec=linec,facecolor=facec,linestyle=linest)

def plot_ellipse(indexes, xG,rho, M, save_to=None, show=True, ax = None):
    px = xG[indexes[0]]
    py = xG[indexes[1]]
    p=get_ellipse_patch(indexes[0], indexes[1], xG,rho,M)
    
    if ax is None:
        fig, ax = plt.subplots()
        
    ax.add_patch(p)
    l=np.max([p.width,p.height])

    labels = ["x0(x_cart)", "x1(theta)", "x2(x_cart_dot)", "x3(theta_dot)"]
    ax.set_xlim(px-l/2,px+l/2)
    ax.set_xlabel(labels[indexes[0]])
    ax.set_ylim(py-l/2,py+l/2)
    ax.set_ylabel(labels[indexes[1]])

    ax.grid(True)

    if not (save_to is None):
        plt.savefig(save_to)
    if show:
        plt.show()
    else:
        return ax