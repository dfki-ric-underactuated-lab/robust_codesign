#  Cartpole Model #

The Cartpole class is to be found within the file 'parameters.py'. It contains the parameters of the cartpole system 
including both the short and the long pendulum setup. It also contains the method 'statespace' for the state space
representation of the linearized plant of the pendulum in the upright position (\theta = 0\degree). Further embedded is
a method calculating the input Voltage for the corresponding input force on the cart.

## Theory #

<div align="center">
<img width="500" src="../../../pics/quanser_cartpole.jpg">
</div>

The amplitude of the input voltage is to be calculated with

<img src="https://render.githubusercontent.com/render/math?math=I\ddot{\theta} %2B b\dot{\theta} %2B c_f \text{sign}(\dot{\theta}) %2B mgl \sin(\theta) = \tau">

where

- <img src="https://render.githubusercontent.com/render/math?math=V_m"> is the motor voltage [V]
- ***J_eq*** 
- ***R_m*** Motor Armature Resistance [Ohm]
- ***r_mp***
- ***eta_g*** Planetary Gearbox Efficiency [-]
- ***K_g*** Planetary Gearbox (a.k.a. Internal) Gear Ratio [-]
- ***eta_m*** Motor Electromechanical Efficiency [ = Tm * w / ( Vm * Im ) ]
- ***K_t*** Motor Torque Constant [N.m/A]
- ***K_m*** Motor Back-EMF Constant [V.s/rad]
- ***u*** Input force on the cart [N]
- ***\dot{x}*** [m/s]


## API #

The cartpole plant can be initialized as follows

    cartpole = Cartpole('selection')

where the input parameter 'selection' corresponds to the length of the pendulum. This parameter has to be either
'short' or 'long'. 

In order to calculate a LQR gain for stabilization of the pendulum in the upright position, a linearized model of the
plant is required. This is to obtained with the 'statespace' method. This method does not require any inputs to
generate the state space matrices A, B, C and D.

    [A, B, C, D] = cartpole.statespace()

As the control input happens to be the Force on the cart, a mapping from this force to the input Voltage of the motor is
required. The method 'amplitude' requiring as inputs the desired force and the velocity of the cart, calculates the
corresponding voltage.

    [voltage] = cartpole.aplitude(inputForce, cartVelocity)

## Usage #

The class is intended to be used for the standard setup of the cartpole system.

## Parameter Identification #

The parameters are provided by the pendulum distributor QUANSER.

## References #

[^fn1]:  **Bruno Siciliano et al.** _Robotics_. Red. by Michael J. Grimble and Michael A.Johnson. Advanced Textbooks in Control and Signal Processing. London: Springer  London,  2009. ISBN:  978-1-84628-641-4  978-1-84628-642-1. DOI: 10.1007/978-1-84628-642-1. URL: http://link.springer.com/10.1007/978-1-84628-642-1 (visited on 09/27/2021).
[^fn2]: **Vinzenz Bargsten, José de Gea Fernández, and Yohannes Kassahun.** _Experimental Robot Inverse Dynamics Identification Using Classical and Machine Learning Techniques_. In: ed. by International Symposium on Robotics. OCLC: 953281127. 2016. URL: https://www.dfki.de/fileadmin/user_upload/import/8264_ISR16_Dynamics_Identification.pdf (visited on 09/27/2021).
[^fn3]: **Jan  Swevers,  Walter  Verdonck,  and  Joris  De  Schutter.** _Dynamic  ModelIdentification for Industrial Robots_. In: IEEE Control Systems27.5 (Oct.2007), pp. 58–71. ISSN: 1066-033X, 1941-000X.doi:10.1109/MCS.2007.904659. URL: https://ieeexplore.ieee.org/document/4303475/(vis-ited on 09/27/2021).
