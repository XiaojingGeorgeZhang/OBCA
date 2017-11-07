###############
# OBCA: Optimization-based Collision Avoidance - a path planner for autonomous parking
# Copyright (C) 2017 
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
# 	X. Zhang, A. Liniger and F. Borrelli; "Optimization-Based Collision Avoidance"; Technical Report, 2017
###############

###############
# Main file: computes Collision-Free and Minimum-Penetration trajectories for parking
###############

# function defined in setup.jl
clear()

#### problem parameters ####
# horizon
N  = 80
# nominal sampling time
Ts = 0.3
#wheelbase
L  = 2.7


# "nominal" shape of ego/controlled car,  ego object is later rotated around the car center
# center of rear wheel axis is reference point
# size of car is: (x_upper + x_lower) + (y_upper + y_lower)
#	   [x_upper, y_upper, -x_lower, -y_lower ]
ego  = [ 3.7   , 1      ,  1      ,  1       ]

##### define obstacles; for simplicity, only polyhedral obstacles are supported at this point
# obstacles are defined by vertices, which are assumed to be enumerated in clock-wise direction
# the first vertex must appear at the end of the list
nOb =  3 	# number of obstacles 
vOb = [4 4 4]	# number of vertices of each obstacle, vector of dimenion nOb
#     	[ 	[[obst1_x1;obst1_y1],[obst1_x2;obst1_y2],[obst1_x3;obst1_y4],...,[obst1_x1;obst1_y1]]    , 		[[obst2_x1;obst2_y1],[obst2_x2;obst2_y2],[obst2_x3;obst2_y4],...,[obst2_x1;obst2_y1]]     ,     ...   ]	
lOb = [   [ [-20;5], [-1.3;5], [-1.3;-5], [-20;-5], [-20;5] ]  , 
	 	  [ [1.3;5], [20;5], [20;-5], [1.3;-5], [1.3;5] ] , 
		  [ [-20;15], [20;15], [20;11], [-20,11], [-20;15] ]		]		#vetices given in CLOCK-WISE direction

# shape of obstacles
#	   [x_upper, y_upper, -x_lower, -y_lower ]
ob1  = [-1.3   , 5      ,  20     ,  5       ]
ob2  = [ 20    , 5      , -1.3    ,  5       ]
ob3  = [ 20    , 15     ,  20     , -11      ]


#           [x_lower, x_upper, -y_lower,   y_upper  ]
XYbounds =  [ -15   , 15      ,  1      ,  10       ]

# initial state
x0 = [-6 7   0    0]

# final state
xF = [ 0 1.3 pi/2 0]

# warm start variables
xWS = zeros(4,N+1)
uWS = zeros(2,N)
timeWS = zeros(1,N+1);

# solution from  distance approach
xp1 = zeros(4,N+1)
up1 = zeros(2,N)
scaleTime1 = zeros(1,N+1)
exitflag1 = 0
time1 = 0

# solution from signed distance approach
xp2 = zeros(4,N+1)
up2 = zeros(2,N)
scaleTime2 = zeros(1,N+1)
exitflag2 = 0
time2 = 0


println("**** START ****")

###### plot setup ######
# plotSetup(x0,xF,ego,L,nOb,vOb,lOb)

###### park without Obstacles ######
println("Parking without Obstacles")
xWS, uWS, timeWS = WarmStart(N,Ts,L,x0,xF,XYbounds)
close("all")
plotTraj(xWS',uWS',N,ego,L,nOb,vOb,lOb,"Trajectory without obstacles",1)

###### obtain H-rep of all obstacles ######
AOb, bOb = obstHrep(nOb, vOb, lOb)

###### park using Distance Approach ######
println("Parking using Distance Approach (Collision Avoidance)")
xp1, up1, scaleTime1, exitflag1, time1, lp1 = ParkingDist(x0,xF,N,Ts,L,ego,xWS,uWS,timeWS,XYbounds,nOb,vOb,AOb,bOb)
if exitflag1==1
	println("  Problem solved SUCCESSFULLY.")
	# close("all")
	plotTraj(xp1',up1',N,ego,L,nOb,vOb,lOb,"Collision Avoidance with distance approach",2)
else
	plotTraj(xp1',up1',N,ego,L,nOb,vOb,lOb,"Potentially infeasible solution",2)
	println("  WARNING: Problem could not be solved.")
end

###### park using Signed-Distance Approach ######
println("Parking using Signed Distance Approach (Minimum Penetration Trajectory)")
xp2, up2, scaleTime2, exitflag2, time2 = ParkingSignedDist(x0,xF,N,Ts,L,ego,xWS,uWS,timeWS,XYbounds,nOb,vOb,AOb,bOb)
if exitflag2==1
	println("  Problem solved SUCCESSFULLY.")
	# close("all")
	plotTraj(xp2',up2',N,ego,L,nOb,vOb,lOb, "Minimum Penetration with signed-distance approach",3)
else
	plotTraj(xp2',up2',N,ego,L,nOb,vOb,lOb, "Potentially infeasible solution",3)
	println("  WARNING: Problem could not be solved.")
end

println("**** DONE ****")