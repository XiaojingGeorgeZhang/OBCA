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
# function plots start position, end position, and obstacles
###############



function plotSetup(x0_s,xF_s,ego,L,nOb,vOb,lOb)

    W_ev = ego[2]+ego[4]
	L_ev = ego[1]+ego[3]

	w = W_ev/2;
	offset = L_ev/2 - ego[3]

	# initial state
	Rot0 = [cos(x0_s[3]) -sin(x0_s[3]); sin(x0_s[3]) cos(x0_s[3])]
	x0 = [x0_s[1]; x0_s[2]]
    centerCar0 = x0 + Rot0*[offset;0]

	# end state
	RotF = [cos(xF_s[3]) -sin(xF_s[3]); sin(xF_s[3]) cos(xF_s[3])]
	xF = [xF_s[1]; xF_s[2]]
    centerCarF = xF + RotF*[offset;0]

	# plot stuff
	figure(1)
	title("Setup")
	plot(x0[1],x0[2],"ob")
	hold(1)
	
	# plot start position
	plot(x0[1],x0[2],"ob")
	carBox(centerCar0,x0_s[3],W_ev/2,L_ev/2)
	carBox(x0 + (Rot0*[L;w-0.15])  ,x0_s[3],0.15,0.3)
	carBox(x0 + (Rot0*[L;-w+0.15]) ,x0_s[3],0.15,0.3)
    carBox(x0 + (Rot0*[0; w-0.15]) ,x0_s[3], 0.15,0.3)
	carBox(x0 + (Rot0*[0;-w+0.15]) ,x0_s[3], 0.15,0.3)
	
	# plot end position
	plot(xF[1],xF[2],"ob")
	carBox_dashed(centerCarF,xF_s[3],W_ev/2,L_ev/2)
	carBox_dashed(xF + (RotF*[L;w-0.15])  ,xF_s[3],0.15,0.3)
	carBox_dashed(xF + (RotF*[L;-w+0.15]) ,xF_s[3],0.15,0.3)
    carBox_dashed(xF + (RotF*[0; w-0.15]) ,xF_s[3], 0.15,0.3)
	carBox_dashed(xF + (RotF*[0;-w+0.15]) ,xF_s[3], 0.15,0.3)
	
	for j = 1 : nOb
		for k = 1 : vOb[j]
			plot([lOb[j][k][1],lOb[j][k+1][1]] , [lOb[j][k][2],lOb[j][k+1][2]] ,"k")
		end
	end
	
	axis("equal")
	sleep(10)
	hold(0)	

end

# plot cars
function carBox(x0,phi,w,l)
    car1 = x0[1:2] + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0[1:2] + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0[1:2] - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0[1:2] - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    plot([car1[1],car2[1],car4[1],car3[1],car1[1]],[car1[2],car2[2],car4[2],car3[2],car1[2]],"k")
end

# plot cars
function carBox_dashed(x0,phi,w,l)
    car1 = x0[1:2] + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0[1:2] + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0[1:2] - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0[1:2] - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    plot([car1[1],car2[1],car4[1],car3[1],car1[1]],[car1[2],car2[2],car4[2],car3[2],car1[2]],":k")
end