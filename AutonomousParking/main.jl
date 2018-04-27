###############
# OBCA: Optimization-based Collision Avoidance - a path planner for autonomous parking
# Copyright (C) 2018
# Alexander LINIGER [liniger@control.ee.ethz.ch; Automatic Control Lab, ETH Zurich]
# Xiaojing ZHANG [xiaojing.zhang@berkeley.edu; MPC Lab, UC Berkeley]
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
###############
# The paper describing the theory can be found here:
# 	X. Zhang, A. Liniger and F. Borrelli; "Optimization-Based Collision Avoidance"; Technical Report, 2017, [https://arxiv.org/abs/1711.03449]
###############

###############
# Main file: computes Collision-Free and Minimum-Penetration trajectories for parking
###############

# function defined in setup.jl
clear()
using PyCall
close("all")

##################################################

# choose one of two predefined scenarios
scenario = "parallel"
scenario = "backwards"

# fixed or variable time 1/0
fixTime = 0				# default: 0 (variable time steps)

#### problem parameters ####
TsPF = 0.05
if scenario == "backwards"
	# nominal sampling time
	sampleN = 3
	if fixTime == 1
		Ts = 0.55/3*sampleN		# 0.55/3 must be compatible with motion resolution of Hybrid A* algorithm
	else
		Ts = 0.6/3*sampleN		# 0.6/3 must be compatible with motion resolution of Hybrid A* algorithm
	end
else
	sampleN = 3
	if fixTime == 1
		Ts = 0.95/3*sampleN		# 0.95/3 must be compatible with motion resolution of Hybrid A* algorithm
	else
		Ts = 0.9/3*sampleN		# 0.9/3 must be compatible with motion resolution of Hybrid A* algorithm
	end
end


#wheelbase
L  = 2.7

# step length of Hybrid A*",
motionStep = 0.1


# "nominal" shape of ego/controlled car,  ego object is later rotated around the car center
# center of rear wheel axis is reference point
# size of car is: (x_upper + x_lower) + (y_upper + y_lower)
#	   [x_upper, y_upper, -x_lower, -y_lower ]
ego  = [ 3.7   , 1      ,  1      ,  1       ]


if scenario == "backwards"
	println("Backwards Parking")
elseif scenario == "parallel"
	println("Parallel Parking")
else
	println("ERROR: please specify parking scenario")
end


if scenario == "backwards"
	##### define obstacles; for simplicity, only polyhedral obstacles are supported at this point
	# obstacles are defined by vertices, which are assumed to be enumerated in clock-wise direction
	#     	[ 	[[obst1_x1;obst1_y1],[obst1_x2;obst1_y2],[obst1_x3;obst1_y4],...,[obst1_x1;obst1_y1]]    , 		[[obst2_x1;obst2_y1],[obst2_x2;obst2_y2],[obst2_x3;obst2_y4],...,[obst2_x1;obst2_y1]]     ,     ...   ]	
	
	# obstacles for plotting
	nObPlot =  3 	# number of obstacles
	vObPlot = [4 4 4]	# number of vertices of each obstacle, vector of dimenion nOb
	# obstacles for plotting
	lObPlot = [   [ [-20;5], [-1.3;5], [-1.3;-5], [-20;-5], [-20;5] ]  ,
	 	  [ [1.3;5], [20;5], [20;-5], [1.3;-5], [1.3;5] ] ,
		  [ [-20;15], [20;15], [20;11], [-20,11], [-20;15] ]		]		#vetices given in CLOCK-WISE direction

	# obstacles for optimization problem
	nOb =  3 	# number of obstacles 
	vOb = [3 3 2]	# number of vertices of each obstacle, vector of dimenion nOb
	vObMPC = vOb-1
	lOb = [   [ [-20;5], [-1.3;5], [-1.3;-5]]  , 
	 	  [ [1.3;-5] , [1.3;5] , [20;5] ] , 
		  [ [20;11], [-20;11]]	]		#vetices given in CLOCK-WISE direction
	
	
	# final state
	xF = [ 0 1.3 pi/2 0]
	

	# build obstacles for Hybrid A* algorithm
	ox = Float64[]
	oy = Float64[]
	# obstacle 1
	for i = -12:0.1:-1.3
	    push!(ox, Float64(i))
	    push!(oy, 5.0)
	end
	for i in -2:5
	    push!(ox, -1.3)
	    push!(oy, Float64(i))
	end
	# obstacle 2
	for i in -2:5
	    push!(ox, 1.3)
	    push!(oy, Float64(i))
	end
	for i = 1.3:0.1:12
	    push!(ox, Float64(i))
	    push!(oy, 5.0)
	end
	# obstacle 3
	for i = -12:12
	    push!(ox, Float64(i))
	    push!(oy, 11.0)
	end

elseif scenario == "parallel"
	##### define obstacles; for simplicity, only polyhedral obstacles are supported at this point
	# obstacles are defined by vertices, which are assumed to be enumerated in clock-wise direction
	# define obstacles for plotting
	nObPlot =  4 	# number of obstacles
	vObPlot = [4 4 4 4]	# number of vertices of each obstacle, vector of dimenion nOb
	#     	[ 	[[obst1_x1;obst1_y1],[obst1_x2;obst1_y2],[obst1_x3;obst1_y4],...,[obst1_x1;obst1_y1]]    , 		[[obst2_x1;obst2_y1],[obst2_x2;obst2_y2],[obst2_x3;obst2_y4],...,[obst2_x1;obst2_y1]]     ,     ...   ]
	lObPlot = [   [ [-15;5], [-3;5], [-3;0], [-15;0], [-15;5] ]  , 
		 	  [ [3;5], [15;5], [15;0], [3;0], [3;5] ] , 
			  [ [-3;0], [-3;2.5], [3;2.5], [3,0], [-3;0] ]	,
			  [ [-15;15], [15;15], [15;11], [-15,11], [-15;15] ]		]		
	
  	# define obstacles for optimization problem
	nOb =  4 	# number of obstacles 
	vOb = [3 3 2 2]	# number of vertices of each obstacle, vector of dimenion nOb
	vObMPC = vOb-1
  	lOb = [   [ [-20;5], [-3.;5], [-3.;0]]  , 
	 	  [ [3.;0] , [3.;5] , [20;5] ] , 
		  [ [-3;2.5], [ 3;2.5]]	,
		  [ [ 20;11 ], [-20;11]]					]		#vetices given in CLOCK-WISE direction

		  # [ [ 3;11 ], [-3;11]]	
			  
	# final state
	xF = [-L/2 4 0 0]
	
	# range of initial points
	x0X_range = -9 : 1 : 9			# 19 points
	x0X_range = -10 : 1 : 10	# 21 points
	x0Y_range = 6.5 : 1.5 :  9.5	# 3
	x0Y_range = 6.5 : 1 :  9.5	# 3
	

	
	ox = Float64[]
	oy = Float64[]

	# obstacles for Hybrid A* algorithms
	# obstacle 1
	for i in -12:0.1: -3.
		push!(ox,Float64(i))
		push!(oy,5.0)
	end

	for i in -2  : 5
		push!(ox,-3.0)
		push!(oy,Float64(i))
	end
	# obstacle 2
	for i in -3 : 3
		push!(ox,Float64(i))
		push!(oy,2.5)
	end
	# obstacle 3
	for i in -2 : 5
		push!(ox,3.0)
		push!(oy,Float64(i))
	end

	for i in 3 :0.1: 12
		push!(ox,Float64(i))
		push!(oy,5.0)
	end
	# obstacle 4
	for i in -12 : 12
		push!(ox,Float64(i))
		push!(oy,11.5)		# 11.0
	end
end


# [x_lower, x_upper, -y_lower,   y_upper  ]
XYbounds =  [ -15   , 15      ,  1      ,  10       ]	# constraints are on (X,Y)

# set initial state
x0 = [-6  9.5   0.0    0.0]

# call Hybrid A*
tic()
rx, ry, ryaw = hybrid_a_star.calc_hybrid_astar_path(x0[1], x0[2], x0[3], xF[1], xF[2], xF[3], ox, oy, hybrid_a_star.XY_GRID_RESOLUTION, hybrid_a_star.YAW_GRID_RESOLUTION, hybrid_a_star.OB_MAP_RESOLUTION)
timeHybAstar = toq();


### extract (smooth) velocity profile from Hybrid A* solution ####
rv = zeros(length(rx),1)
for i=1:length(rx)
	if i < length(rx)
		rv[i] = (rx[i+1] - rx[i])/(Ts/sampleN)*cos(ryaw[i]) + (ry[i+1]-ry[i])/(Ts/sampleN)*sin(ryaw[i])
	else
		rv[i] = 0
	end
end
### Smoothen velocity 0.3 m/s^2 max acceleration ###
v,a = veloSmooth(rv,0.3,Ts/sampleN)
### compute steering angle ###
delta = atan(diff(ryaw)*L/motionStep.*sign(v[1:end-1]));


### Down-sample for Warmstart ##########
rx_sampled = rx[1:sampleN:end]	# : 5
ry_sampled = ry[1:sampleN:end]
ryaw_sampled = ryaw[1:sampleN:end]
rv_sampled = rv[1:sampleN:end]
v_sampled = v[1:sampleN:end]

a_sampled = a[1:sampleN:end]
delta_sampled = delta[1:sampleN:end]

## initialize warm start solution
xWS = [rx_sampled ry_sampled ryaw_sampled v_sampled]
uWS = [delta_sampled a_sampled]					

### prepare for OBCA ###
N = length(rx_sampled)-1
AOb, bOb = obstHrep(nOb, vOb, lOb)


###### park using Distance Approach ######
println("Parking using Distance Approach (A* warm start)")
# believe it's correct; in "ParkingDist1.jl"
xp20, up20, scaleTime20, exitflag20, time20, lp20, np20 = ParkingDist(x0,xF,N,Ts,L,ego,XYbounds,nOb,vObMPC,AOb,bOb,rx_sampled,ry_sampled,ryaw_sampled,fixTime,xWS,uWS)
if exitflag20==1
	println("  --> Distance: SUCCESSFUL.")
	plotTraj(xp20',up20',length(rx_sampled)-1,ego,L,nObPlot,vObPlot,lObPlot,"Distance Approach (Collision Avoidance )",2)
else
	println("  --> WARNING: Problem could not be solved.")
end


###### park using Signed Distance Approach ######
println("Parking using Signed Distance Approach (A* warm start)")
xp10, up10, scaleTime10, exitflag10, time10, lp10, np10 = ParkingSignedDist(x0,xF,N,Ts,L,ego,XYbounds,nOb,vObMPC,AOb,bOb,rx_sampled,ry_sampled,ryaw_sampled,fixTime,xWS,uWS)
if exitflag10==1
	println("  --> Signed Distance: SUCCESSFUL.")
	plotTraj(xp10',up10',length(rx_sampled)-1,ego,L,nObPlot,vObPlot,lObPlot,"Signed Distance Approach (Min. Penetration)",1)
	
else
	println("  --> WARNING: Problem could not be solved.")
end



println("********************* summary *********************")
println("  Time Hybrid A*: ", timeHybAstar, " s")
println("  Time Distance approach: ", time20, " s")
println("  Time Signed Distance approach: ", time10, " s")

println("********************* DONE *********************")



