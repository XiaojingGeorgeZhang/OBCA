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
 
 
function WarmStartQuadcopter(N,Ts,x0,xF)

	xW =[	x0[1:3]';
 		 	[x0[1:2]' 0.01];
 	 	   	[5 4.5 0.01];
 	 	   	[5 4.5 2.5];
 	 	   	[9 4.5 2.5];
	 	   	xF[1:3]'		];

	d = zeros(5,1)
	 
	for i = 1:5	
		d[i] = norm(xW[i,:]-xW[i+1,:])
	end 
	sumd = sum(d)

	pointsPerMeter = sumd/N

	i = 1
	s1 = zeros(3,length(linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))))
	s1[1,:] = linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))
	s1[2,:] = linspace(xW[i,2],xW[i+1,2],round(d[i]/pointsPerMeter))
	s1[3,:] = linspace(xW[i,3],xW[i+1,3],round(d[i]/pointsPerMeter))

	i = 2
	s2 = zeros(3,length(linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))))
	s2[1,:] = linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))
	s2[2,:] = linspace(xW[i,2],xW[i+1,2],round(d[i]/pointsPerMeter))
	s2[3,:] = linspace(xW[i,3],xW[i+1,3],round(d[i]/pointsPerMeter))

	i = 3
	s3 = zeros(3,length(linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))))
	s3[1,:] = linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))
	s3[2,:] = linspace(xW[i,2],xW[i+1,2],round(d[i]/pointsPerMeter))
	s3[3,:] = linspace(xW[i,3],xW[i+1,3],round(d[i]/pointsPerMeter))

	i = 4
	s4 = zeros(3,length(linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))))
	s4[1,:] = linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))
	s4[2,:] = linspace(xW[i,2],xW[i+1,2],round(d[i]/pointsPerMeter))
	s4[3,:] = linspace(xW[i,3],xW[i+1,3],round(d[i]/pointsPerMeter))

	i = 5
	s5 = zeros(3,length(linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))))
	s5[1,:] = linspace(xW[i,1],xW[i+1,1],round(d[i]/pointsPerMeter))
	s5[2,:] = linspace(xW[i,2],xW[i+1,2],round(d[i]/pointsPerMeter))
	s5[3,:] = linspace(xW[i,3],xW[i+1,3],round(d[i]/pointsPerMeter))

	sp = [s1 s2 s3 s4 s5];
	sp = sp[:,1:N-1];

	timeScalep = 2;

	vsp = zeros(3,N-1);# [diff(sp,2)/(Ts*timeScalep) zeros(3,1)];#zeros(3,N-1);# 

	S = [sp; 0*ones(3,N-1); vsp; zeros(3,N-1)]

	xp = [x0' S xF']

	up = 0.5*ones(4,N)

	timeScalep = 1;

	return xp, up, timeScalep, time
end
