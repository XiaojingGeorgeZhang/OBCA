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


function ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vOb, A, b,x,u,l,n,timeScale,fixTime,sd)


	# desired safety distance
	dmin = 0.05			# anything bigger than 0, e.g. 0.05

	c0 = zeros(5,1)
	c1 = zeros(4,1)
	c2 = zeros(4,1)
	c3 = zeros(4,N)
	c4 = zeros(1,1)
	c5 = zeros(1,1)
	c6 = zeros(4,N+1)

	e = zeros(7,1)

	c0[1] = maximum(abs(u[1,:]))-0.6	# should be <= 0
	c0[2] = maximum(abs(u[2,:]))-0.4	# should be <= 0
	c0[3] = maximum(abs(timeScale-1))-0.2 # should be <= 0
	c0[4] = -minimum(l)	# should be <= 0
	c0[5] = -minimum(n)	# should be <= 0

	#starting point
	c1[1] = abs(x[1,1] - x0[1])		# should be <= 0
	c1[2] = abs(x[2,1] - x0[2])		# should be <= 0
	c1[3] = abs(x[3,1] - x0[3])		# should be <= 0
	c1[4] = abs(x[4,1] - x0[4])		# should be <= 0

	#end point
	c2[1] = abs(x[1,N+1] - xF[1])	# should be <= 0
	c2[2] = abs(x[2,N+1] - xF[2])	# should be <= 0
	c2[3] = abs(x[3,N+1] - xF[3])	# should be <= 0
	c2[4] = abs(x[4,N+1] - xF[4])	# should be <= 0

	##############################
	# dynamics of the car
	##############################
	# - unicycle dynamic with euler forward
	# - sampling time scaling, is identical over the horizon

	for i in 1:N
 		if fixTime == 1
			c3[1,i] = x[1,i+1] - (x[1,i] + Ts*(x[4,i] + Ts/2*u[2,i])*cos((x[3,i] + Ts/2*x[4,i]*tan(u[1,i])/L)))
		    c3[2,i] = x[2,i+1] - (x[2,i] + Ts*(x[4,i] + Ts/2*u[2,i])*sin((x[3,i] + Ts/2*x[4,i]*tan(u[1,i])/L)))
		    c3[3,i] = x[3,i+1] - (x[3,i] + Ts*(x[4,i] + Ts/2*u[2,i])*tan(u[1,i])/L)
		    c3[4,i] = x[4,i+1] - (x[4,i] + Ts*u[2,i])
	    else
		    c3[1,i] = x[1,i+1] - (x[1,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*cos((x[3,i] + timeScale[i]*Ts/2*x[4,i]*tan(u[1,i])/L)))
		    c3[1,i] = x[2,i+1] - (x[2,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*sin((x[3,i] + timeScale[i]*Ts/2*x[4,i]*tan(u[1,i])/L)))
		    c3[1,i] = x[3,i+1] - (x[3,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*tan(u[1,i])/L)
		    c3[1,i] = x[4,i+1] - (x[4,i] + timeScale[i]*Ts*u[2,i])
	    end
	end

	u0 = [0,0]

	if fixTime == 1
		c5 = maximum(abs(diff([0. u[1,:]']'))/Ts) - 0.6
		c4 = 0
	else
		c4 = maximum(abs(diff(timeScale)))
		c5 = maximum(abs(diff([0 u[1,:]']'))/(timeScale[1]*Ts)) - 0.6
	end


	##############################
	# obstacle avoidance constraints
	##############################

	# width and length of ego set
	W_ev = ego[2]+ego[4]
	L_ev = ego[1]+ego[3]

	g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2]
	
	# ofset from X-Y to the center of the ego set
	offset = (ego[1]+ego[3])/2 - ego[3]


	for i in 1:N+1 	# iterate over time steps
		for j = 1 : nOb 	# iterate over obstacles
			Aj = A[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract obstacle matrix associated with j-th obstacle
			lj = l[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract lambda dual variables associate j-th obstacle
			nj = n[(j-1)*4+1:j*4 ,:] 						# extract mu dual variables associated with j-th obstacle
			bj = b[sum(vOb[1:j-1])+1 : sum(vOb[1:j])]		# extract obstacle matrix associated with j-th obstacle

			# norm(A'*lambda) <= 1
			if sd == 1
				c6[1,i] = abs((sum(Aj[k,1]*lj[k,i] for k = 1 : vOb[j]))^2 + (sum(Aj[k,2]*lj[k,i] for k = 1 : vOb[j]))^2)  - 1 
			else
				c6[1,i] = (sum(Aj[k,1]*lj[k,i] for k = 1 : vOb[j]))^2 + (sum(Aj[k,2]*lj[k,i] for k = 1 : vOb[j]))^2  - 1 
			end

			# G'*mu + R'*A*lambda = 0
			c6[2,i] = abs((nj[1,i] - nj[3,i]) +  cos(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + sin(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]))
			c6[3,i] = abs((nj[2,i] - nj[4,i]) -  sin(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + cos(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]))

			# -g'*mu + (A*t - b)*lambda > 0
			c6[4,i] = -(-sum(g[k]*nj[k,i] for k = 1:4) + (x[1,i]+cos(x[3,i])*offset)*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j])
							+ (x[2,i]+sin(x[3,i])*offset)*sum(Aj[k,2]*lj[k,i] for k=1:vOb[j]) - sum(bj[k]*lj[k,i] for k=1:vOb[j]))  + dmin
		end

	end

	e[1] = maximum(c0)<= 5e-5
	e[2] = maximum(c1)<= 5e-5
	e[3] = maximum(c2)<= 5e-5
	e[4] = maximum(abs(c3))<= 5e-5
	e[5] = c4 <= 5e-5
	e[6] = c5 <= 5e-5
	e[7] = maximum(c6)<= 5e-5

	# print(e,"\n")

	if sum(e) == 7
		return 1
	else
		return 0
	end

end
