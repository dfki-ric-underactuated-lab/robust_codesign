---
title: "Robust Co-Design of Canonical Underactuated Systems for Increased Certifiable Stability"

# pdf: https://www.researchgate.net/publication/376186142_Fast_Dynamic_Walking_with_RH5_Humanoid_Robot
# youtube: https://www.youtube.com/watch?v=39GL2vPedGY&ab_channel=GermanResearchCenterforArtificialIntelligence

authors:
  - {name: Federico Girlanda, affiliation_key: [1,2], link: linkedin.com/in/federico-girlanda-6a3336218mitarbeiter/ivbe01.html}
  - {name: Lasse Maywald, affiliation_key: 1, link: linkedin.com/in/lasse-jenning-shala-b8502b187}
  - {name: Shivesh Kumar, affiliation_key: [1,3], link: https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/shku02.html}
  - {name: Frank Kirchner, affiliation_key: [1,4], link: https://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/frki01.html}
affiliations:
  - {name: German Research Center for Artificial Intelligence, affiliation_key: 1, link: https://robotik.dfki-bremen.de/de/startseite}
  - {name: University of Padua (Italy), affiliation_key: 2, link: https://www.dei.unipd.it/en/home-page}
  - {name: Chalmers University of Technology (Sweden), affiliation_key: 3, link: https://www.chalmers.se/en/departments/m2/}
  - {name: Universität Bremen, affiliation_key: 4, link: https://www.uni-bremen.de/robotik}
---

## Introduction
Optimal behaviours of a system to perform a specific task can be achieved by leveraging the coupling between trajectory optimization, stabilization, and design optimization. This approach is particularly advantageous for underactuated systems, which are systems that have fewer actuators than degrees of freedom and thus require for more elaborate control systems. This paper proposes a novel co-design algorithm, namely Robust Trajectory Control with Design optimization (RTC-D). An inner optimization layer (RTC) simultaneously performs direct transcription (DIRTRAN) to find a nominal trajectory while computing optimal hyperparameters for a stabilizing time-varying linear quadratic regulator (TVLQR). RTC-D augments RTC with a design optimization layer, maximizing the system's robustness through a time-varying Lyapunov-based region of attraction (ROA) analysis. This analysis provides a formal guarantee of stability for a set of off-nominal states.

<figure>
  <img src="../results/media/robustCodesign.png" width="100%" alt="">
  <figcaption>Fig.1 - Robust co-optimization for the optimal fitness of the desired motion.</figcaption>
</figure> 

## Results and Discussion
The proposed algorithm has been tested on two different underactuated systems: the torque-limited simple pendulum and the cart-pole.

<figure>
  <img src="../results/media/RTCcartpole.png" width="40%" alt="">
  <img src="../results/media/RTCDpendulum.png" width="40%" alt="">
  <figcaption>Fig.2 - Funnel volume increasing due to RTC for Cart-pole (left) and comparison between RTC and RTC-D optimization  for Simple pendulum (right).</figcaption>
</figure>

Extensive simulations of off-nominal initial conditions demonstrate improved robustness, while real-system experiments show increased insensitivity to torque disturbances.
The experiment...

The robot was tested in two different conditions:
<ul>  
  <li>Initial situation,</li>
  <li>Optimized scenario.</li>
</ul>

<figure>
  <img src="../results/media/RTCcartpoleVer.png" width="40%" alt="">
  <img src="../results/media/RTCDpendulumVer.png" width="40%" alt="">
  <figcaption>Fig.3 - Experimental verification of stability guarantee (green funnel) given by the RTC for Cart-pole (left) and RTC-D for Simple pendulum (right). The optimal configuration (RTC and RTC-D) manages to achieve the desired final stabilization where the initial one does not.</figcaption>
</figure>

Increased robustness...

<!-- <table>
  <tbody>
    <tr align="center" valign="center">
      <td>
        <figure>
          <img src="static/figures/plot_footsteps_best_3_video.png" width="95%" alt="">
          <figcaption>Fig.3 - Measured footsteps for fast walking</figcaption>
        <figure>
      </td>
      <td>
        </figure>
          <img src="static/figures/plot_walk_in_place_force_z_left.png" width="95%" alt="">
          <figcaption>Fig.4 - Measured vertical GRF while stepping in place</figcaption>
        </figure>
      </td>
    </tr>
    <tr align="center" valign="center">
      <td>
        <figure>
          <img src="static/figures/plot_feet_cop_walk_fast.png" width="95%" alt="">
          <figcaption>Fig.5 - Measured CoP on fastest walk</figcaption>
        </figure>
      </td>
      <td>
        <figure>
          <img src="static/figures/plot_footsteps_long_strides.png" width="95%" alt="">
          <figcaption>Fig.6 - Measured footsteps for long stride walk</figcaption>
        </figure>
      </td>
    </tr>
  </tbody>
</table> -->