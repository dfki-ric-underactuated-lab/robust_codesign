import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import matplotlib as mpl
import mpl_toolkits.mplot3d.art3d as art3d
mpl.use("WebAgg")
mpl.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from matplotlib.collections import LineCollection

from simple_pendulum.controllers.tvlqr.roa.utils import projectedEllipseFromCostToGo, getEllipseContour, \
                                                        sample_from_ellipsoid, quad_form
from simple_pendulum.simulation.simulation import Simulator
from simple_pendulum.utilities.process_data import prepare_trajectory, getEllipseFromCsv

def get_ellipse_params(rho,M):
    """
    Returns ellipse params (excl center point)
    """

    #eigenvalue decomposition to get the axes
    w,v=np.linalg.eigh(M/rho) 

    try:
        #let the smaller eigenvalue define the width (major axis*2!)
        width = 2/float(np.sqrt(w[0]))
        height = 2/float(np.sqrt(w[1]))
        
        #the angle of the ellipse is defined by the eigenvector assigned to the smallest eigenvalue (because this defines the major axis (width of the ellipse))
        angle = np.rad2deg(np.arctan2(v[:,0][1],v[:,0][0]))

    except:
        print("paramters do not represent an ellipse.")

    return width,height,angle

def get_ellipse_patch(px,py,rho,M,alpha_val=1,linec="red",facec="none",linest="solid", label = ""):
    """
    return an ellipse patch
    """
    w,h,a = get_ellipse_params(rho,M)
    return patches.Ellipse((px,py), w, h, a, alpha=alpha_val,ec=linec,facecolor=facec,linestyle=linest, label = label)

def plot_ellipse(px,py,rho, M, save_to=None, show=True):
    p=get_ellipse_patch(px,py,rho,M)
    
    fig, ax = plt.subplots()
    ax.add_patch(p)
    l=np.max([p.width,p.height])

    ax.set_xlim(px-l/2,px+l/2)
    ax.set_ylim(py-l/2,py+l/2)

    ax.grid(True)

    if not (save_to is None):
        plt.savefig(save_to)
    if show:
        plt.show()

##############
# Funnels plot
##############

def plotFirstLastEllipses(funnel_path, traj_path):
    # load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    x0_traj = [trajectory.T[1].T, trajectory.T[2].T]
    x0 = x0_traj[0]
    goal = [np.pi, 0]

    # load funnel data
    rho_first, S_first = getEllipseFromCsv(funnel_path, 0)
    rho_last, S_last = getEllipseFromCsv(funnel_path, -1)

    # plots
    fig,ax = plt.subplots(1,2, figsize=(18,8))
    p0 = get_ellipse_patch(np.array(x0_traj).T[0][0],np.array(x0_traj).T[0][1],rho_first,S_first,linec= "black")
    ax[0].scatter([x0[0]],[x0[1]],color="black",marker="o")
    ax[0].add_patch(p0)
    ax[0].grid(True)
    ax[0].set_xlabel("x")
    ax[0].set_ylabel(r"$\dot{x}$")
    ax[0].title.set_text('First ellipse')
    pl = get_ellipse_patch(np.array(x0_traj).T[-1][0],np.array(x0_traj).T[-1][1],rho_last,S_last ,linec= "black")
    ax[1].scatter([goal[0]],[goal[1]],color="black",marker="x")
    ax[1].add_patch(pl)
    ax[1].grid(True)
    ax[1].set_xlabel("x")
    ax[1].set_ylabel(r"$\dot{x}$")
    ax[1].title.set_text('Last ellipse')

def plotRhoEvolution(funnel_path, traj_path):
    # load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0_traj = [trajectory.T[1].T, trajectory.T[2].T]
    N = len(time)

    # load funnel data
    funnel_data = np.loadtxt(funnel_path, skiprows=1, delimiter=",")
    rho = funnel_data[0].T

    # plots
    fig = plt.figure()
    plt.title("rho evolution")
    ax = fig.add_subplot()
    ax.set_xlabel("Number of steps")
    ax.plot(np.arange(N),rho, color = "yellow", label = "final rho")
    ax2=ax.twinx()
    d = round(len(time)/N)
    ang = np.abs([x0_traj[0][d*p] for p in range(N)])
    vel = np.abs([x0_traj[1][d*p] for p in range(N)])
    ax2.plot(np.arange(N),ang,color="blue", label = "nominal traj, angle")
    ax2.plot(np.arange(N),vel,color="red", label = "nominal traj, velocity")
    ax.legend(loc = "upper left")
    ax2.legend(loc = "upper right")

def plotFunnel3d(csv_path, traj_path, ax = None, fontSize = 18, ticksSize = 16):
    '''
    Function to draw a discrete 3d funnel plot. Basically we are plotting a 3d ellipse patch in each 
    knot point.
    Parameters
    ----------
    rho : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array of matrices that define ellipses in all the knot points, from tvlqr controller.
    x0: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    ax: matplotlib.axes
        axes of the plot where we want to add the 3d funnel plot, useful in the verification function.
    '''

    # load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0 = [trajectory.T[1].T, trajectory.T[2].T]

    # create figure if not provided, plot then the nominal trajectory
    nominal = None
    if(ax == None):
       fig = plt.figure(figsize = (20,20)) 
       ax = fig.add_subplot(111, projection='3d')
       nominal, = ax.plot(time,x0[0],x0[1],label = r"$(\mathbf{x}^{\star}, \mathbf{u}^{\star})$", color = "C1", linestyle = "--", linewidth = "0.3", zorder = 3) # plot of the nominal trajectory

    for i in range(len(time)):
        (rho_i, S_i) = getEllipseFromCsv(csv_path, i)
        # Drawing the main ellipse
        ctg=np.asarray(S_i)
        labels=[r"$\theta$"+" [rad]",r'$\dot \theta$'+" [rad/s]"]
        s0=0
        s1=1

        w,h,a=projectedEllipseFromCostToGo(s0,s1,[rho_i],[ctg])

        elliIn=patches.Ellipse((x0[s0][i],x0[s1][i]), 
                                w[0], 
                                h[0],
                                a[0],ec="black",linewidth=1.25, color = "green", alpha = 0.1)
        ax.add_patch(elliIn)
        art3d.pathpatch_2d_to_3d(elliIn, z=time[i], zdir="x") # 3d plot of a patch

    #plt.title("3d resulting Funnel", fontsize = fontSize)
    ax.set_xlabel("time [s]", fontsize = fontSize)
    ax.set_ylabel(labels[s0], fontsize = fontSize)
    ax.set_zlabel(labels[s1], fontsize = fontSize)
    ax.tick_params('x', labelsize=ticksSize)
    ax.tick_params('y', labelsize=ticksSize)
    ax.tick_params('z', labelsize=ticksSize)
    ax.set_xlim(0, time[-1])
    ax.set_ylim(-1, 5)
    ax.set_zlim(-6, 6)
    ax.xaxis.labelpad=20
    ax.yaxis.labelpad=20
    ax.zaxis.labelpad=20
    return ax, nominal

def plotFunnel(funnel_path, traj_path, ax = None, fontSize = 18, ticksSize = 16, noTraj = False):
    '''
    Function to draw a continue 2d funnel plot. This implementation makes use of the convex hull concept
    as done in the MATLAB code of the Robot Locomotion Group (https://groups.csail.mit.edu/locomotion/software.html).
    Parameters
    ----------
    rho : np.array
        array that contains the estimated rho value for all the knot points
    S: np.array
        array of matrices that define ellipses in all the knot points, from tvlqr controller.
    x0: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    '''
    # load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0 = [trajectory.T[1].T, trajectory.T[2].T]

    # figure initialization
    zorder = 2
    funnel_color = 'red'
    traj_color = "black"#funnel_color# "orange"
    if (ax == None):
        fig = plt.figure(figsize=(15,12))
        #fig.set_size_inches(w=6, h=5)
        ax = fig.add_subplot()
        zorder = 1
        funnel_color = 'green'
        traj_color = "black"#funnel_color#"blue"
        ax.grid(True)
        labels=[r"$\theta$"+" [rad]",r'$\dot \theta$'+" [rad/s]"]
        ax.set_xlabel(labels[0], fontsize = fontSize)
        ax.set_ylabel(labels[1], fontsize = fontSize)
        ax.set_xlim(-1, 4)
        ax.set_ylim(-10, 10)
        plt.xticks(fontsize = ticksSize)
        plt.yticks(fontsize = ticksSize)

    if not noTraj:
        ax.plot(x0[0],x0[1], zorder = 3, color = traj_color) # plot of the nominal trajectory

    not_last = -1 # TODO: weird last ellipse, why? maybe Qf != Q?
    for i in range(len(time)-1):#+not_last):
        (rho_i, S_i) = getEllipseFromCsv(funnel_path,i)
        (rho_iplus1, S_iplus1) = getEllipseFromCsv(funnel_path,i+1)
        # c_prev = getEllipseContour(S_i,rho_i, np.array(x0).T[i]) # get the contour of the previous ellipse
        # c_next = getEllipseContour(S_iplus1,rho_iplus1, np.array(x0).T[i+1]) # get the contour of the next ellipse
        # points = np.vstack((c_prev,c_next))

        # # plot the convex hull of the two contours
        # hull = ConvexHull(points) 
        # line_segments = [hull.points[simplex] for simplex in hull.simplices]
        # ax.add_collection(LineCollection(line_segments,
        #                              colors=funnel_color,
        #                              linestyle='solid', zorder = zorder, alpha = 0.5))
        # plot the ellipse patch
        w,h,a=projectedEllipseFromCostToGo(0,1,[rho_i],[S_i])
        e = patches.Ellipse((x0[0][i],x0[1][i]), 
                                w[0], 
                                h[0],
                                a[0],ec="black",linewidth=1.25, color = funnel_color)#, alpha = 0.1), zorder=zorder
        ax.add_patch(e)
    return ax

def TVrhoVerification(pendulum, controller, funnel_path, traj_path, nSimulations, ver_idx, fontSize = 18, ticksSize = 16):
    '''
    Function to verify the time-variant RoA estimation. This implementation permitts also to choose
    which knot has to be tested. Furthermore the a 3d funnel plot has been implemented.

    Parameters
    ----------
    pendulum: simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller: simple_pendulum.controllers.tvlqr.tvlqr
        configured tvlqr controller object
    x0_t: np.array 
        pre-computed nominal trajectory
    time: np.array
        time array related to the nominal trajectory
    nSimulations: int
        number of simulations for the verification
    ver_idx: int
        knot point to be verified
    '''
    # load trajectory data
    trajectory = np.loadtxt(traj_path, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0_t = [trajectory.T[1].T, trajectory.T[2].T]

    # load funnel data
    funnel_data = np.loadtxt(funnel_path, skiprows=1, delimiter=",")
    rho = funnel_data[0].T

    # simulation time interval
    dt = time[1]-time[0] 

    # plot of the verified ellipse
    fig, ax = plt.subplots(figsize = (10,12))
    labels=[r"$\theta$"+" [rad]",r'$\dot \theta$'+" [rad/s]"]
    ax.set_xlabel(labels[0], fontsize = fontSize)
    ax.set_ylabel(labels[1], fontsize = fontSize)
    ax.grid(True)
    ax.tick_params(axis='both', which='major', labelsize=ticksSize)
    ax.set_title(f"Verified ellipse, knot {ver_idx}", fontsize = fontSize)

    S_t = controller.tvlqr.S
    p = get_ellipse_patch(np.array(x0_t).T[ver_idx][0],np.array(x0_t).T[ver_idx][1],rho[ver_idx],S_t.value(time[ver_idx]),linec= "black")
    ax.add_patch(p)
    ax.scatter(x0_t[0][ver_idx],x0_t[1][ver_idx],color="blue", s = 5)

    # plot if the verified 3d funnel and of the nominal trajectory
    fig1 = plt.figure(figsize = (15,12)) 
    ax1 = fig1.add_subplot(111, projection='3d')
    plotFunnel3d(funnel_path, traj_path, ax1, fontSize=fontSize, ticksSize=ticksSize)
    nominal, = ax1.plot(time, x0_t[0],x0_t[1], label = "nominal trajectory", color = "blue", linestyle = "--", linewidth = "0.3")

    one_green = False
    one_red = False
    one_orange = False
    for j in range(1,nSimulations+1):      

        color_green = False
        color_red = False
        color_orange = False                                                                                                        

        xBar0=sample_from_ellipsoid(S_t.value(time[ver_idx]),rho[ver_idx]) # sample new initial state inside the estimated RoA
        x_i=xBar0+np.array(x0_t).T[ver_idx] 

        sim = Simulator(plant=pendulum) # init the simulation

        # plotting the checked initial states and resulting trajectories, the color depends on the result 
        x_start = x_i 
        scale = 50
        X_sim = []
        T_sim = []
        for i in range(len(time)-ver_idx-1):
            T, X, U = sim.simulate(time[ver_idx+i], x_start, time[ver_idx+i+1], dt/scale, controller) # simulating this interval 
            x_start = X[-1]
            T_sim = np.append(T_sim,T)
            if i == 0:
                X_sim = X
            else:
                X_sim = np.vstack((X_sim,X))
            ctg_i = quad_form(S_t.value(time[ver_idx+i+1]),x_start-np.array(x0_t).T[ver_idx+i+1])
            if (ctg_i > rho[ver_idx+i+1]):
                if i == (len(rho)-1):    
                    color_red = True
                    one_red = True
                else:
                    color_orange = True
                    one_orange = True   
            else:
                color_green = True   
                one_green = True       

        # managing the dynamic colors of the plot TODO: FIX index of X plot in 3d funnels
        if (color_green and (not color_orange)):
            greenDot = ax.scatter([x_i[0]],[x_i[1]],color="green",marker="o", s = 5)
            simulated, = ax1.plot(T_sim, X_sim[:,0], X_sim[:,1], color = "green", label = "simulated trajectory", linewidth = "0.3")
        elif (color_green and color_orange): 
            orangeDot = ax.scatter([x_i[0]],[x_i[1]],color="orange",marker="o")
            simulated, = ax1.plot(T_sim, X_sim[:,0], X_sim[:,1], color = "orange")
        else:
            redDot = ax.scatter([x_i[0]],[x_i[1]],color="red",marker="o")
            simulated, = ax1.plot(T_sim, X_sim[:,0], X_sim[:,1], color = "red")
    ax1.legend(handles = [simulated, nominal], fontsize=fontSize)
    return (ax, ax1)

def funnel2DComparison(funnel_path1, funnel_path2, traj_path1, traj_path2 = None, volumes = None, fontSize = 18, ticksSize = 16):
    # load trajectory data
    trajectory = np.loadtxt(traj_path1, skiprows=1, delimiter=",")
    time = trajectory.T[0].T
    x0 = [trajectory.T[1].T, trajectory.T[2].T]

    # figure initialization
    fig = plt.figure(figsize=(12, 9))
    axes = fig.subplots(2,1)
    zorder = 1
    funnel_color = 'green'
    
    # plot settings
    axes[0].grid(True)
    axes[1].grid(True)
    labels=[r"$\theta$"+" [rad]",r'$\dot \theta$'+" [rad/s]"]
    axes[0].set_ylabel(labels[1], fontsize = fontSize)
    axes[1].set_ylabel(labels[1], fontsize = fontSize)
    axes[1].set_xlabel(labels[0], fontsize = fontSize)
    axes[0].set_xlim(-2, 5)
    axes[0].set_ylim(-10, 10)
    axes[1].set_xlim(-2, 5)
    axes[1].set_ylim(-15, 15)
    axes[0].tick_params(axis='both', which='major', labelsize=ticksSize)
    axes[1].tick_params(axis='both', which='major', labelsize=ticksSize)
    if volumes is not None:
        text0 = f"V = {volumes[0]}"
        text1 = f"V = {volumes[1]}"
        props = dict(boxstyle='round', facecolor='white', alpha=0.5)
        axes[0].text(0.05, 0.95, text0, transform=axes[0].transAxes, fontsize=fontSize,
        verticalalignment='top', bbox=props)
        axes[1].text(0.05, 0.95, text1, transform=axes[1].transAxes, fontsize=fontSize,
        verticalalignment='top', bbox=props)

    axes[0].plot(x0[0],x0[1], zorder = 3) # plot of the nominal trajectory

    for i in range(len(time)-1):
        (rho_i, S_i) = getEllipseFromCsv(funnel_path1,i)
        (rho_iplus1, S_iplus1) = getEllipseFromCsv(funnel_path1,i+1)
        c_prev = getEllipseContour(S_i,rho_i, np.array(x0).T[i]) # get the contour of the previous ellipse
        c_next = getEllipseContour(S_iplus1,rho_iplus1, np.array(x0).T[i+1]) # get the contour of the next ellipse
        points = np.vstack((c_prev,c_next))

        # plot the convex hull of the two contours
        hull = ConvexHull(points) 
        line_segments = [hull.points[simplex] for simplex in hull.simplices]
        axes[0].add_collection(LineCollection(line_segments,
                                     colors=funnel_color,
                                     linestyle='solid', zorder = zorder, alpha = 0.5))
    
    if(traj_path2 == None):
        traj_path2 = traj_path1
    plotFunnel(funnel_path2, traj_path2, axes[1], fontSize= fontSize, ticksSize= ticksSize)

def rhoComparison(csv_pathFunnelSos, csv_pathFunnelProb, label1 = None, label2 = None, fontSize = 18, ticksSize = 16):
    # load funnel data
    funnel_data = np.loadtxt(csv_pathFunnelSos, skiprows=1, delimiter=",")
    rho_sos = funnel_data[0].T
    funnel_data = np.loadtxt(csv_pathFunnelProb, skiprows=1, delimiter=",")
    rho_prob = funnel_data[0].T
    N = len(rho_sos)

    # plots
    fig, ax = plt.subplots(figsize = (9,9))
    ax.tick_params(axis='both', which='major', labelsize=ticksSize)
    ax.set_title("rho evolution comparison", fontsize= fontSize)
    ax.set_xlabel("Number of steps", fontsize= fontSize)
    ax.set_ylabel(r"$\rho$", fontsize= fontSize)
    if (label1 is not None) and (label2 is not None):
        ax.plot(range(N),rho_sos,color = "red", label = label1)
        ax.plot(range(N),rho_prob,color = "green", label = label2)
        pass
    else:
        ax.plot(range(N),rho_sos,color = "red", label = "SOS method")
        ax.plot(range(N),rho_prob,color = "green", label = "Probabilistic Method")
    ax.plot(range(N),rho_prob-rho_sos,color = "yellow", label = "Difference")
    ax.legend(fontsize= fontSize)