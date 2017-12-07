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
# run setup.jl before running this file
###############



# function defined in setup.jl
clear()

# horizon
N  = 80
# nominal sampling time
Ts = 0.25

egoR = 0.25

ob12 = [2.5,12,7,   -2,2,-0.6]

ob22 = [7.5,12,7,   -7,-5,2]
ob32 = [7.5,4, 7,   -7, 2,2]

ob42 = [7.5,5, 2,   -7,-4,2]
ob52 = [7.5,5, 7,   -7,-4,-3]

ob6  = [10,10, 5, 0,0, 0]

# x0 = [1,Y,Z,0,0,0] Y and Z can be modified
x0 = [1 1 3 0 0 0 0 0 0 0 0 0]
# xF = [9,Y,Z,0,0,0] Y and Z can be modified
xF = [9 3 2 0 0 0 0 0 0 0 0 0]


xWS = zeros(12,N+1)
uWS = zeros(4,N)
timeWS = 1;

xp1 = zeros(12,N+1)
up1 = zeros(4,N)
scaleTime1 = 1

xp2 = zeros(12,N+1)
up2 = zeros(4,N)
scaleTime2 = 1
exitflag = -1


# warm starting trajectory
println("Warm starting trajectory")
xWS, uWS, timeWS = WarmStartQuadcopter(N,Ts,x0,xF)
close("all")
plotTrajQuadcopter(xWS',uWS',N,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Warm Start")

# trajectory with Distance Approach
println("Trajectory using Distance Approach (Collision Avoidance)")
xp1,up1,scaleTime1,exitflag1,time1 = QuadcopterDist(x0,xF,N,Ts,R,ob12,ob22,ob32,ob42,ob52,xWS,uWS,timeWS)
close("all")
plotTrajQuadcopter(xp1',up1',N,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Collision Avoidance with distance approach")


# trajectory with Signed Distance Approach
println("Trajectory using Signed Distance Approach (Minimum Penetration)")
xp2,up2,scaleTime2,exitflag2,time2 = QuadcopterSignedDist(x0,xF,N,Ts,R,ob12,ob22,ob32,ob42,ob52,xWS,uWS,timeWS)
close("all")
plotTrajQuadcopter(xp2',up2',N,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Minimum Penetration with signed-distance approach")


