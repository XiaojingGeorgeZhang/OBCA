# OBCA - Autonomous Parking
Optimization-Based Collision Avoidance - an application towards autonomous parking

Paper describing the theory can be found [here](http://arxiv.org/abs/1711.03449).

## How to run the Parking code:

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

2. Type in terminal: include("setup.jl")

3. Type in terminal: include("main.jl")


### modifying the code 

1. To play with start points, change x0 in main.jl and run 
the code by include("main.jl")

2. If you change anything in one of the collision avoidance
problems, you need to activate the changes by running 
include("setup.jl")


### Note
1. This code has been tested on Julia 0.5 and 0.6, and might not run on any other Julia versions.

2. For best results, run code in Julia terminal
