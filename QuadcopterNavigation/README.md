# OBCA - Quadcopter Path Planning
Optimization-Based Collision Avoidance - an application in quadcopter path planning

Paper describing the theory can be found [here](http://arxiv.org/abs/1711.03449).

## How to run the code:

### First steps

1. Change to the directory

2. Install Julia from https://julialang.org/downloads/ (code tested on version 0.5 and 0.6) 

3. Open Julia in terminal

4. Install Julia package JuMP using Pkg.add("JuMP")

5. Install Julia package Ipopt using Pkg.add("Ipopt")

6. Install Julia package PyPlot using Pkg.add("PyPlot")

7. Install Julia package PyPlot using Pkg.add("NearestNeighbors")


### Running the parking example 

1. Start Julia in terminal

2. Type in terminal: include("setupQuadcopter.jl")

3. Type in terminal: include("mainQuadcopter.jl")


### modifying the code 

1. To play with start points, change xF (or x0) in mainQuadcopter.jl and run 
the code by include("mainQuadcopter.jl")

2. If you change anything in one of the collision avoidance
problems, you need to activate the changes by running 
include("setupQuadcopter.jl")


### Note
1. This code has been tested on Julia 0.5 and 0.6, and might not run on any other Julia versions.

2. For best results, run code in Julia terminal
