# Dynamic Survey for Flapping Wing UAV

--- 

This repository contains analysis, comparison, and realization of different formulation of dynamics of Flapping Wing UAV.
These dynamic formulation are specific to small-large sized Flapping Wing UAV, which primarly have a flexible membrane instead of a traditional airfoil. The majority of the dynamic formualation assume a traditional airfoil which generate the lift using thee pressure different and they tend to ingore the unstead wakes, leading edge suction force, and apparent force. This repository mainly focuses on implementing the formulation which will work on bat-type flapping wing uav which have the film type airfoil which considers the following effects: 

- "Aeroelasticity" : The interaction of the non ridig bodies with fluid.
- "Unsteady Forces" : The Inertial forces of the Air around Flapping Wing UAV.
- "Leading Edge Suction Effect" : Which provide additional lift, thrust majorily present due to the thin leading edge.

The implemented formulation should implement the above effects accurately, and should be relizable in real time for the simulation. The formulation structure should be in such a way that it should be able to offload some of the compute to GPU.

--- 

## Literature Survey 
The following research paper's formulation was used to implement to analyse the dynamics of the flapping wing uav.

- ### An Improved Quasi-Steady Model Capable of Calculating Flexible Deformation for Bird-Sized Flapping Wings (2024) 
#### Authors: Tianyou Mao, Chuangqiang Guo, Bosong Duan, et al. 
