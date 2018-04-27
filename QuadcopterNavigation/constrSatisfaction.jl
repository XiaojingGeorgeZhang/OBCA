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


function constrSatisfaction(x, u, timeScale,x0,xF,Ts,lambda,ob1,ob2,ob3,ob4,ob5,R)

	# xp: 12 x (N+1)
	# up: 4 x N
	# timeScalep : 1x(N+1)
	
	mass = 0.5;
	g = 9.81;
	reg = 0;
	reg2 = 1e-4;
	reg3 = 0.0001;

	k_F = 0.0611;#6.11*1e-8         #[N/rpm^2]
	k_M = 0.0015;#1.5*1e-9          #[Nm/rpm^2]
	I = [3.9,4.4,4.9]*1e-3  #[kg/m2]
	L = 0.225               #[m]



	# unwrap dual variables: lp = [l1p ; l2p ; l3p ; l4p ; l5p]
	l1 = lambda[1:6,:]
	l2 = lambda[7:12,:]
	l3 = lambda[13:18,:]
	l4 = lambda[19:24,:]
	l5 = lambda[25:30,:]
	

	w_H = sqrt((mass*g)/(k_F*4))
	
	N = size(x,2)-1
	
	tmp_slack = x[:,1] - x0'
	if norm(tmp_slack, Inf)>1e-3
		# println("initial state not satisfied")
		# println(tmp_slack)
		return false
	end
	
	tmp_slack = x[:,end] - xF'
	if norm(tmp_slack, Inf)>1e-3
		# println("final state not satisfied")
		# println(tmp_slack)
		return false
	end
	
	A = [eye(3);
	    -eye(3)];
		
    b1 = ob1
    b2 = ob2
    b3 = ob3
    b4 = ob4
	b5 = ob5
	    
	
	for i = 1 : N
		# check input constraints
		tmp_slack = [1.2 ; 1.2 ; 1.2 ; 1.2] - u[:,i] 	# must be <= 0
		if maximum(tmp_slack) > 0
			# println("input constraint 1 not satisfied at i = ", i)
			# println(tmp_slack)
			return false
		end
		# for j = 1 : 4
# 			if tmp_slack[j] >0
# 				println("input constraint 1 not satisfied at i = ", i)
# 				println(tmp_slack)
# 			end
# 		end
		tmp_slack = u[:,i] - [7.8 ; 7.8 ; 7.8 ; 7.8] 	# must be <= 0
		if maximum(tmp_slack)>0
			# println("input constraint 2 not satisfied at i = ", i)
			# println(tmp_slack)
			return false
		end
		# for j = 1 : 4
# 			if tmp_slack[j] >0
# 				println("input constraint 2 not satisfied at i = ", i)
# 				println(tmp_slack)
# 			end
# 		end
		# check state box constraints
		tmp_slack = [0;0;0;-3;-0.2;-0.2;-1;-1;-1;-1.5;-1;-1] - x[:,i]
		if maximum(tmp_slack)>0
			# println("state constraint 1 not satisfied at i = ", i)
			# println(tmp_slack)
			return false
		end
		# for j = 1 : 7
		# 	if tmp_slack[j] >0
		# 		println("state constraint 1 not satisfied at i = ", i)
		# 		println(tmp_slack)
		# 	end
		# end
		tmp_slack = x[:,i] - [10;10;5;3;0.2;0.2;1;1;1;3;1;1]
		if maximum(tmp_slack)>0
			# println("state constraint 2 not satisfied at i = ", i)
			# println(tmp_slack)
			return false
		end
		# for j = 1 : 7
		# 	if tmp_slack[j] >0
		# 		println("state constraint 2 not satisfied at i = ", i)
		# 		println(tmp_slack)
		# 	end
		# end
		
		# check state dynamic constraints
	    #X,Y,Z
		
		tmp_slack = zeros(13,1)
		tmp_slack[1] = x[1,i+1] - ( x[1,i] + timeScale[i]*Ts*x[7,i] )
	   	tmp_slack[2] = x[2,i+1] - ( x[2,i] + timeScale[i]*Ts*x[8,i] )
	    tmp_slack[3] = x[3,i+1] - ( x[3,i] + timeScale[i]*Ts*x[9,i] )
	    # pitch, roll, yaw
	    tmp_slack[4] = x[4,i+1] - ( x[4,i] + timeScale[i]*Ts*( cos(x[5,i])            *x[10,i]        +sin(x[5,i])            *x[12,i]) )
		tmp_slack[5] = x[5,i+1] - ( x[5,i] + timeScale[i]*Ts*( sin(x[5,i])*tan(x[4,i])*x[10,i]+x[11,i]-cos(x[5,i])*tan(x[4,i])*x[12,i]) )
		tmp_slack[6] = x[6,i+1] - ( x[6,i] + timeScale[i]*Ts*(-sin(x[5,i])*sec(x[4,i])*x[10,i]        +cos(x[5,i])*sec(x[4,i])*x[12,i]) )
	    #v_x, v_y, v_z
	    tmp_slack[7] = x[7,i+1] - ( x[7,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*( sin(x[4,i])*cos(x[5,i])*sin(x[6,i]) + sin(x[5,i])*cos(x[6,i]) )) )
	    tmp_slack[8] = x[8,i+1] - ( x[8,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*(-sin(x[4,i])*cos(x[5,i])*cos(x[6,i]) + sin(x[5,i])*sin(x[6,i]) )) )
	    # tmp_slack[9] = x[9,i+1] - ( x[9,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*( cos(x[4,i])*cos(x[5,i])) - mass*g ) )
		tmp_slack[9] = x[9,i+1] - ( x[9,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*( cos(x[4,i])*cos(x[5,i])) - mass*g ) )
		
		
	    # pitch_rate, roll_rate
	    tmp_slack[10] = x[10,i+1] - ( x[10,i] + timeScale[i]*Ts*1/I[1]*(L*k_F*(u[2,i]^2 - u[4,i]^2)                     - (I[3] - I[2])*x[11]*x[12]) )
	    tmp_slack[11] = x[11,i+1] - ( x[11,i] + timeScale[i]*Ts*1/I[2]*(L*k_F*(u[3,i]^2 - u[1,i]^2)                     - (I[1] - I[3])*x[10]*x[12]) )
	    tmp_slack[12] = x[12,i+1] - ( x[12,i] + timeScale[i]*Ts*1/I[3]*(k_M*(u[1,i]^2 - u[2,i]^2 + u[3,i]^2 - u[4,i]^2) - (I[2] - I[1])*x[10]*x[11]) )
	    tmp_slack[13] = timeScale[i] - timeScale[i+1]
		
		if norm(tmp_slack, Inf)>1e-3
			# println("state dynamics / timeScale inside verification not satisfied at i = ", i)
			# println(tmp_slack)
			return false
		end
		
		if minimum(minimum(lambda)) < -1e-3
			# println("dual variables constraints ", i)
			# println(lambda)
			return false
		end
		
		
		# checking of obstacle avoidance constraints
		
	    tmp_slack = zeros(5,1)
		tmp_slack[1] = (l1[1,i]-l1[4,i])^2 + (l1[2,i]-l1[5,i])^2 + (l1[3,i]-l1[6,i])^2 - 1		# should be <= 0
	    tmp_slack[2] = (l2[1,i]-l2[4,i])^2 + (l2[2,i]-l2[5,i])^2 + (l2[3,i]-l2[6,i])^2 - 1
		tmp_slack[3] = (l3[1,i]-l3[4,i])^2 + (l3[2,i]-l3[5,i])^2 + (l3[3,i]-l3[6,i])^2 - 1
	    tmp_slack[4] = (l4[1,i]-l4[4,i])^2 + (l4[2,i]-l4[5,i])^2 + (l4[3,i]-l4[6,i])^2 - 1
		tmp_slack[5] = (l5[1,i]-l5[4,i])^2 + (l5[2,i]-l5[5,i])^2 + (l5[3,i]-l5[6,i])^2 - 1
		
		if maximum(tmp_slack) > 1e-3
			# println("obstacle avoidance constraints 1 violated")
			# println(tmp_slack)
			return false
		end
		
		tmp_slack = zeros(5,1)
		tmp_slack[1] = sum(-b1[j]*l1[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l1[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l1[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l1[j,i] for j=1:6) - R	# should be >=0
		tmp_slack[2] = sum(-b2[j]*l2[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l2[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l2[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l2[j,i] for j=1:6) - R
		tmp_slack[3] = sum(-b3[j]*l3[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l3[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l3[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l3[j,i] for j=1:6) - R
		tmp_slack[4] = sum(-b4[j]*l4[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l4[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l4[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l4[j,i] for j=1:6) - R
		tmp_slack[5] = sum(-b5[j]*l5[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l5[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l5[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l5[j,i] for j=1:6) - R
		if minimum(tmp_slack) < -1e-3
			# println("obstacle avoidance constraints 2 violated")
			# println(tmp_slack)
			return false
		end
	end

	# all constraints satisfied
	return true
end

