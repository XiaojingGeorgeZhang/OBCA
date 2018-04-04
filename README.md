# OBCA
Optimization-Based Collision Avoidance - a path planner for autonomous navigation

Paper describing the theory can be found [here](http://arxiv.org/abs/1711.03449).

*Note*: An OBCA version specialized towards autonomous parking can be found at [H-OBCA](https://github.com/XiaojingGeorgeZhang/H-OBCA).

## Short Description
OBCA is a novel method for formulating collision avoidance constraints. It provides a smooth reformulation of collision avoidance constraints, allowing the use of generic non-linear optimization solvers. 

OBCA can be used to in path planning algorithms to generate *high-quality paths* that satisfy the system dynamics as well as satefy constraints. We provide [Julia](https://julialang.org/)-based implementations for a quadcopter navigation problem and for autonomous parking problems.

## Examples


### OBCA for Quadcopter Navigation
<img src="https://github.com/XiaojingGeorgeZhang/OBCA/blob/master/images/TrajQuad_3D_Video.gif" width="700" />

### OBCA for Autonomous Parking 

#### Backwards Parking
<img src="https://github.com/XiaojingGeorgeZhang/OBCA/blob/master/images/TrajBack_ParkingVideo.gif" width="700" />

#### Parallel Parking
<img src="https://github.com/XiaojingGeorgeZhang/OBCA/blob/master/images/TrajPar_ParkingVideo.gif" width="700" />

#### Parking of Truck with Trailer
<img src="https://github.com/XiaojingGeorgeZhang/OBCA/blob/master/images/TrajTrailer_ParkingVideo.gif" width="700" />

