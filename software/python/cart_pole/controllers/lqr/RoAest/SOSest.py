
########################
# Time invarying RoA est
########################

import numpy as np

# drake imports
from pydrake.all import (MathematicalProgram, Solve, Variables)

def bisect_and_verify(sys, Kf, Sf, hyper_params, rho_min=1e-10,
                      rho_max=5, maxiter=20, verbose=False):
    """
    Simple bisection root finding for finding the RoA using the feasibility
    problem.
    The default values have been choosen after multiple trials.
    """
    for i in range(maxiter):
        # np.random.uniform(rho_min,rho_max)
        rho_probe = rho_min+(rho_max-rho_min)/2
        res = verify_double_pendulum_rho(rho_probe,
                                         sys,
                                         Kf,
                                         Sf,
                                         taylor_deg=hyper_params["taylor_deg"],
                                         lambda_deg=hyper_params["lambda_deg"])
        if verbose:
            print("---")
            print("rho_min:   "+str(rho_min))
            print("rho_probe: "+str(rho_probe)+" verified: "+str(res))
            print("rho_max:   "+str(rho_max))
            print("---")
        if res:
            rho_min = rho_probe
        else:
            rho_max = rho_probe

    return rho_min

def verify_double_pendulum_rho(rho, sys, Kf, Sf, taylor_deg=3,
                               lambda_deg=4, verbose=False,
                               x_bar_eval=[np.pi, 0, 0, 0]):

    # Opt. Problem definition (Indeterminates in error coordinates)
    prog = MathematicalProgram()
    x_bar = prog.NewIndeterminates(4, "x_bar")

    # Dynamics definition and Taylor approximation
    x_star = np.array([0,0,0,0])
    x = x_star + x_bar 

    u_star = 0
    ubar_plus = sys.fl - u_star # Saturation parameters
    ubar_minus = - sys.fl - u_star
    u_bar      = -Kf.dot(x_bar)[0]
    u  = u_star + u_bar # Control input

    x_star_lin = np.array([0,0,0,0]) # linearization point in error coordinates 
    f = sys.linearized_continuous_dynamics3(x_bar, [u_bar], x_star_lin, taylor_deg)
    f_minus = sys.linearized_continuous_dynamics3(x_bar, [ubar_minus], x_star_lin, taylor_deg)
    f_plus = sys.linearized_continuous_dynamics3(x_bar, [ubar_plus], x_star_lin, taylor_deg)

    # Definition of the Lyapunov function and of its derivative
    V = x_bar.dot(Sf.dot(x_bar))
    Vdot = (V.Jacobian(x_bar).dot(f))
    Vdot_minus = (V.Jacobian(x_bar).dot(f_minus))
    Vdot_plus = (V.Jacobian(x_bar).dot(f_plus))

    # Multipliers definition
    lambda_b = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    # u in linear range
    lambda_2 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    lambda_3 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    # uplus
    lambda_c = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    lambda_4 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    # uminus
    lambda_a = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
    lambda_1 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()

    # Considering the input saturation in the constraints
    epsilon = 10e-20
    nom1 = (- u + ubar_minus) # where both nom1 and nom2 are < 0, the nominal dynamics have to be fullfilled
    nom2 = (u - ubar_plus)  
    neg = (u - ubar_minus)  # where this is < 0, the negative saturated dynamics have to be fullfilled
    pos = (- u + ubar_plus)   # where this is < 0, the positive saturated dynamics have to be fullfilled     
    prog.AddSosConstraint((-Vdot + lambda_b*(V-rho) + lambda_2*nom1 + lambda_3*nom2 - epsilon*x_bar.dot(x_bar)))
    # neg saturation
    prog.AddSosConstraint((-Vdot_minus + lambda_a*(V - rho) + lambda_1*neg - epsilon*x_bar.dot(x_bar)))
    # pos saturation
    prog.AddSosConstraint((-Vdot_plus + lambda_c*(V - rho) + lambda_4*pos - epsilon*x_bar.dot(x_bar)))

    # Problem solution
    result = Solve(prog)
    # print(prog) # usefull for debugging

    return result.is_success()