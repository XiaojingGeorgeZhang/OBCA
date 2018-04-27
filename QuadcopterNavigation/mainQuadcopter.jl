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
# run setup.jl before running this file
###############




# function defined in setup.jl
clear()
close("all")

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

# reshape the state
Ts = 0.25
timeScalep = 0.5;

################## construct environment for hybrid A-star ##################
### note: environment scaled by 10x for convenience ###
ox = Float64[]
oy = Float64[]
oz = Float64[]
# first wall
for xx in 20 : 25
	for yy in 0 : 105
		for zz in 6 : 55
			push!(ox,Float64(xx))
			push!(oy,Float64(yy))
			push!(oz,Float64(zz))	
		end
	end	
end
# second wall; only for plotting purposes
for xx = 70 : 75
	# left piece
	for yy = 0 : 40
		for zz = 0 : 55
			push!(ox, Float64(xx))
			push!(oy, Float64(yy))
			push!(oz, Float64(zz))
		end
	end
	# right piece
	for yy = 50 : 105
		for zz = 0 : 55
			push!(ox, Float64(xx))
			push!(oy,Float64(yy))
			push!(oz,Float64(zz))
		end
	end
	# top piece
	for yy = 40 : 50
		for zz = 30 : 55
			push!(ox, Float64(xx))
			push!(oy,Float64(yy))
			push!(oz,Float64(zz))
		end
	end
	# right piece
	for yy = 40 : 50
		for zz = 0 : 20
			push!(ox, Float64(xx))
			push!(oy,Float64(yy))
			push!(oz,Float64(zz))
		end
	end
end
# room size
xmin = 0.0
ymin = 0.0
zmin = 0.0
xmax = 105.0
ymax = 105.0
zmax = 55.0
# extract start and end position
sx = x0[1]*10.0  # [m]
sy = x0[2]*10.0  # [m]
sz = x0[3]*10.0  # [m]
gx = xF[1]*10.0  # [m]
gy = xF[2]*10.0  # [m]
gz = xF[3]*10.0  # [m]
# call A* algorithm
rx, ry, rz, rtime = a_star.calc_astar_path( sx, sy, sz, 		# start
									gx, gy, gz, 		# goal
									ox, oy, oz, 		# list of obstacles
									xmin, ymin, zmin, 	# box constraint
									xmax, ymax, zmax, 	# box constraint
									1.0	)				# (scaled) grid resolution 
									
N_as = length(rx)-1	# length of Astar
# nominal sampling time
Ts_as = round((Ts*80/N_as)*100)/100	# scale sampling time for Astar

###### stitch together Astar solution for warm starting ######
rxryrz=[rx'/10 ; ry'/10 ; rz'/10]
vWS = zeros(3,N_as+1)
xWS_as = [rx'/10 ; ry'/10 ; rz'/10 ; zeros(3,N_as+1) ; vWS ; zeros(3,N_as+1) ];
uWS_as = 0.5*ones(4,N_as);	# same length as horizon
timeWS_as = 1
# not plotting might get rid of IPOPT restoration failure messages...
# plotTrajQuadcopter(xWS_as',uWS_as',N_as,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Warm Start (A*) ",0)


######### trajectory with Distance Approach
println("Trajectory using Distance Approach (Collision Avoidance, A star)")
xp1_as,up1_as,scaleTime1_as,exitflag1_as,time1_as,l1_as,exitstatus1_as  = QuadcopterDist(x0,xF,N_as,Ts_as,egoR,ob12,ob22,ob32,ob42,ob52,xWS_as,uWS_as,timeWS_as)
plotTrajQuadcopter(xp1_as',up1_as',N_as,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Collision Avoidance with distance approach ",10)
trajFeas1_as = constrSatisfaction(xp1_as,up1_as,scaleTime1_as,x0,xF,Ts_as,l1_as,ob12,ob22,ob32,ob42,ob52,egoR) 


######### trajectory with Signed Distance Approach
println("Trajectory using Signed Distance Approach (Minimum Penetration, A star)")
xp2_as,up2_as,scaleTime2_as,exitflag2_as,time2_as,l2_as,exitstatus2_as = QuadcopterSignedDist(x0,xF,N_as,Ts_as,egoR,ob12,ob22,ob32,ob42,ob52,xWS_as,uWS_as,timeWS_as)
plotTrajQuadcopter(xp2_as',up2_as',N_as,egoR,ob12,ob22,ob32,ob42,ob52,ob6,egoR,"Minimum Penetration with signed-distance approach",20)
trajFeas2_as = constrSatisfaction(xp2_as,up2_as,scaleTime2_as,x0,xF,Ts_as,l2_as,ob12,ob22,ob32,ob42,ob52,egoR) 

println("---- Done ----")

