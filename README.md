# Robust Co-Design of Canonical Underactuated Systems for Increased Certifiable Stability

This repository contains the code, data and plots of the paper [Robust Co-Design of Canonical Underactuated Systems for Increased Certifiable Stability](https://dfki-ric-underactuated-lab.github.io/robust_codesign/). The paper has been submitted at the [ICRA 2024 Conference](https://2024.ieee-icra.org/).

### Abstract
Optimal behaviours of a system to perform a specific task can be achieved by leveraging the coupling between trajectory optimization, stabilization and design optimization. This approach proves particularly advantageous for underactuated systems, which are systems that have fewer actuators than degrees of freedom and thus require for more elaborate control systems. This paper proposes a novel co-design algorithm, namely Robust Trajectory Control with Design optimization
(RTC-D). An inner optimization layer (RTC) simultaneously performs direct transcription (DIRTRAN) to find a nominal trajectory while computing optimal hyperparameters for a stabilizing time-varying linear quadratic regulator (TVLQR). RTC-D augments RTC with a design optimization layer, maximizing the system’s robustness through a time-varying Lyapunov-based region of attraction (ROA) analysis. This analysis provides a formal guarantee of stability for a set of off-nominal states. 

</div>
<div align="center">
<img width="600" src="results/media/robustCodesign.png">
</div>
</div>

The proposed algorithm has been tested on two different underactuated systems: the torque-limited simple pendulum and the cart-pole. Extensive simulations of off-nominal initial conditions demonstrate improved robustness, while real-system experiments show increased insensitivity to torque disturbances.

### Content
In the [examples](examples) folder the user can find all the code necessary to generate the data and plots that have been included in the paper.
Running

    python examples/simple_pendulum/resultsPlots.py
    python examples/cart_pole/resultsPlots.py 

will show the final results of our approach. The main algorithm RTC-D and it's reduced version RTC are implemented for each system in the *rtc_CMAES.py* and *rtcd_CMAES.py* files respectively.

A set of example code is provided both for [simple pendulum](examples/simple_pendulum/) and [cart-pole](examples/cart_pole/). Both the simulated and the experimental verification of the obtained results can be visualized by running the *verificationPlots.py* script. ROA estimation examples are implementedted in the *lqr_roa.py* and *tvlqr_roa.py* files. Also, a direct transcription trajectory optimization is included in a specific file named *dirtran.py*.

### Installation
The use of a virtual environment for the installation is suggested as a common good programming choice. For example, the use of *pipenv* requires the following commands

    pipenv shell
    pipenv install software/python
    pipenv install 'drake==1.5.0'    
A LaTeX distribution installation might be necessary for the correct visualization of the results. In case of error due to the version of a library please refer to versions available from March 2022.

### Results
Torque-limited simple pendulum:
</div>
<div align="center">
<img width="210" src="results/media/RTCDpendulum.png">
<img width="210" src="results/media/realSPsystem.png">
<img width="210" src="results/media/RTCDpendulumVer.png">
</div>
</div>
Cart-pole:
<div align="center">
<img width="210" src="results/media/RTCcartpole.png">
<img width="210" src="results/media/realCPsystem.png">
<img width="210" src="results/media/RTCcartpoleVer.png">
</div>

### Aknowledgements and Citation

Code contained in [software/python](software/python/) has been inspired by pre-existing software of DFKI GmbH. It's usage has been agreed with the owner.

Copyright on the material in this webpage has been transferred to IEEE for ICRA 2024: 
F. Girlanda, L. Shala, S. Kumar and F. Kirchner, "Robust Co-Design of Canonical Underactuated Systems for Increased Certifiable Stability," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 13271-13277, doi: 10.1109/ICRA57147.2024.10611645.

    @INPROCEEDINGS{10611645, 
    author={Girlanda, Federico and Shala, Lasse and Kumar, Shivesh and Kirchner, Frank}, 
    booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
    title={Robust Co-Design of Canonical Underactuated Systems for Increased Certifiable Stability}, 
    year={2024}, 
    volume={}, 
    number={}, 
    pages={13271-13277}, 
    keywords={Couplings;Torque;Regulators;Stability analysis;Robustness;Task analysis;Robotics and automation}, 
    doi={10.1109/ICRA57147.2024.10611645}}


</div>
<div align="center">
  <img src="results/media/logo.svg" style="width:281px">
  <img src="results/media/ulab.gif" style="width:225px">
  <img src="results/media/MRock-Logo.png" style="width:198px">
</div>