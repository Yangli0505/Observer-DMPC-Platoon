# Observer-Based DMPC for String-Stable Vehicle Platoons
This repository provides the MATLAB simulation code for the paper:

> **Robust string-stable platoon control via observer-based distributed MPC under Markovian switching V2V networks**
> Wenwei Que, Yang Li, Lu Wang, Manjiang Hu, Hongmao Qin, Xin Huang, Yougang Bian, Jianqiang Wang
> *Transportation Research Part C: Emerging Technologies*, Vol. 187, 105641, 2026.
> https://doi.org/10.1016/j.trc.2026.105641

## Overview

This work develops an **observer-based distributed model predictive control (DMPC)** framework for connected and automated vehicle platoons under **directed Markovian switching V2V communication topologies**.

The switching communication topology is modeled as a continuous-time Markov chain. A fully distributed adaptive observer is designed to estimate the leader vehicle information under randomly switching communication networks. Based on the estimated information, the DMPC controller incorporates a string-stability constraint to maintain stable propagation of disturbances along the vehicle platoon.

The proposed framework provides:

* Modeling of directed Markovian switching V2V communication topologies
* Fully distributed adaptive observation of leader vehicle states
* Observer error mean-square stability
* Distributed model predictive platoon control
* Explicit predecessor-following string-stability constraints
* Recursive feasibility and closed-loop stability
* Robust platoon control under communication switching and disturbances

## Code

The MATLAB simulation contains the following main files:

* `simulation_cav_observer_final.m` — main simulation program
* `MyVehicleDynamics.m` — vehicle dynamics model
* `MyCostFunction.m` — DMPC cost function
* `MyConstraints.m` — DMPC constraints
* `myProb.mat` — data used for the Markovian switching process

The main simulation file allows different platoon configurations, vehicle dynamics, communication topologies, leader-vehicle maneuvers, disturbances, model mismatch, and string-stability settings to be configured.

## Citation

If you find this code useful for your research, please cite our paper:

```bibtex
@article{QUE2026105641,
  title   = {Robust string-stable platoon control via observer-based distributed MPC under Markovian switching V2V networks},
  author  = {Wenwei Que and Yang Li and Lu Wang and Manjiang Hu and Hongmao Qin and Xin Huang and Yougang Bian and Jianqiang Wang},
  journal = {Transportation Research Part C: Emerging Technologies},
  volume  = {187},
  pages   = {105641},
  year    = {2026},
  doi     = {10.1016/j.trc.2026.105641}
}
```

## Contact

For questions regarding the paper or code, please feel free to contact the authors lyxc56@gmail.com.
