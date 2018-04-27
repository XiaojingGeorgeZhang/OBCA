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


function ParkingSignedDist(x0,xF,N,Ts,L,ego,XYbounds,nOb,vOb, A, b,rx,ry,ryaw,fixTime,xWS,uWS)


	# desired safety distance
	dmin = 0.05			# anything bigger than 0, e.g. 0.05

	##############################
	# Define JuMP file
	##############################
	# Define IPOPT as solver and well as solver settings
	##############################
	# seems to work best
	m = Model(solver=IpoptSolver(hessian_approximation="exact",mumps_pivtol=1e-6,alpha_for_y="min",recalc_y="yes",
	                             mumps_mem_percent=6000,max_iter=200,tol=1e-5, print_level=0,
	                             min_hessian_perturbation=1e-12,jacobian_regularization_value=1e-7))#,nlp_scaling_method="none"

	##############################
	# defining optimization variables
	##############################
	#state
	@variable(m, x[1:4,1:(N+1)])
	#scaling on sampling time
	if fixTime == 0
		@variable(m, timeScale[1:N+1])
	end
	#control
	@variable(m, u[1:2,1:(N)])
	# lagrange multipliers for dual dist function
	@variable(m, l[1:sum(vOb),1:(N+1)])	# dual multiplier associated with obstacleShape
	@variable(m, n[1:nOb*4,1:(N+1)])	# dual multiplier associated with carShape
	@variable(m, sl[1:nOb,1:(N+1)])	# slack variable to avoid infeasibilities


	# regularization parameter to improve numerical stability
	reg = 1e-7;
	##############################
	# cost function
	##############################
	# (min control inputs)+      
	# (min time)+
	# (regularization dual variables)
	##############################
	 # @NLobjective(m, Min,sum(0.1*u[1,i]^2 + 1*u[2,i]^2 for i = 1:N) + 
	 # 					 sum(0.5*timeScale[i] + 1*timeScale[i]^2 for i = 1:N+1)+
	 #                     0*sum(sum(reg*n[j,i]^2 for i = 1:N+1)  for j = 1:4) + 
		# 				 0*sum(sum(reg*l[j,i]^2 for i = 1:N+1)  for j = 1:sum(vOb)) ) 
	u0 = [0,0]
	# fix time objective
	if fixTime == 1
		@NLobjective(m, Min,sum(0.01*u[1,i]^2 + 0.5*u[2,i]^2 for i = 1:N) + 
		 					 sum(0.1*((u[1,i+1]-u[1,i])/Ts)^2 + 0.1*((u[2,i+1]-u[2,i])/Ts)^2 for i = 1:N-1)+
		 					 	(0.1*((u[1,1]-u0[1])/(Ts))^2 + 0.1*((u[2,1]-u0[2])/(Ts))^2) +
		 					 sum(0.0001*x[4,i]^2 for i=1:N+1)+
		 					 sum(0.001*(x[1,i]-rx[i])^2 + 0.001*(x[2,i]-ry[i])^2 + 0.01*(x[3,i]-ryaw[i])^2 for i=1:N+1) + 
							 sum(sum(1e2*sl[j,i] + 1e4*sl[j,i]^2 for i = 1:N+1) for j = 1 : nOb)							)
	else
	# variable time objective
		@NLobjective(m, Min,sum(0.01*u[1,i]^2 + 0.1*u[2,i]^2 for i = 1:N) + 
		 					 sum(0.1*((u[1,i+1]-u[1,i])/(timeScale[i]*Ts))^2 + 0.1*((u[2,i+1]-u[2,i])/(timeScale[i]*Ts))^2 for i = 1:N-1) +
		 					    (0.1*((u[1,1]-u0[1])   /(timeScale[1]*Ts))^2 + 0.1*((u[2,1]-u0[2])   /(timeScale[1]*Ts))^2) +
		 					 sum(0.5*timeScale[i] + 1*timeScale[i]^2 for i = 1:N+1)+
		 					 sum(0.0001*x[4,i]^2 for i=1:N+1)+
		 					 sum(0.001*(x[1,i]-rx[i])^2 + 0.001*(x[2,i]-ry[i])^2 + 0.0001*(x[3,i]-ryaw[i])^2 for i=1:N+1) + 
							 sum(sum(1e2*sl[j,i] + 1e4*sl[j,i]^2 for i = 1:N+1) for j = 1 : nOb)							) 	#  
	end

	##############################
	# bounds on states, inputs,
	# and dual multipliers.
	##############################
	#input constraints
	@constraint(m, [i=1:N], -0.6 <= u[1,i] <= 0.6)
	@constraint(m, [i=1:N], -0.4 <= u[2,i] <= 0.4)

	#state constraints
	@constraint(m, [i=1:N+1], XYbounds[1] <= x[1,i] <= XYbounds[2])
	@constraint(m, [i=1:N+1], XYbounds[3] <= x[2,i] <= XYbounds[4])
	@constraint(m, [i=1:N+1], -1  <= x[4,i] <= 2)

	# bounds on time scaling
	if fixTime == 0
		@constraint(m, 0.8 .<= timeScale .<= 1.2)
	end

	# positivity constraints on dual multipliers
	@constraint(m, l .>= 0)
	@constraint(m, n .>= 0)

	##############################
	# start and finish point
	##############################

	#starting point
	@constraint(m, x[1,1] == x0[1])
	@constraint(m, x[2,1] == x0[2])
	@constraint(m, x[3,1] == x0[3])
	@constraint(m, x[4,1] == x0[4])

	#end point
	@constraint(m, x[1,N+1] == xF[1])
	@constraint(m, x[2,N+1] == xF[2])
	@constraint(m, x[3,N+1] == xF[3])
	@constraint(m, x[4,N+1] == xF[4])

	##############################
	# dynamics of the car
	##############################
	# - unicycle dynamic with euler forward
	# - sampling time scaling, is identical over the horizon

	for i in 1:N

 		if fixTime == 1
			@NLconstraint(m, x[1,i+1] == x[1,i] + Ts*(x[4,i] + Ts/2*u[2,i])*cos((x[3,i] + Ts/2*x[4,i]*tan(u[1,i])/L)))
		    @NLconstraint(m, x[2,i+1] == x[2,i] + Ts*(x[4,i] + Ts/2*u[2,i])*sin((x[3,i] + Ts/2*x[4,i]*tan(u[1,i])/L)))
		    @NLconstraint(m, x[3,i+1] == x[3,i] + Ts*(x[4,i] + Ts/2*u[2,i])*tan(u[1,i])/L)
		    @NLconstraint(m, x[4,i+1] == x[4,i] + Ts*u[2,i])
	    else
		    @NLconstraint(m, x[1,i+1] == x[1,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*cos((x[3,i] + timeScale[i]*Ts/2*x[4,i]*tan(u[1,i])/L)))
		    @NLconstraint(m, x[2,i+1] == x[2,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*sin((x[3,i] + timeScale[i]*Ts/2*x[4,i]*tan(u[1,i])/L)))
		    @NLconstraint(m, x[3,i+1] == x[3,i] + timeScale[i]*Ts*(x[4,i] + timeScale[i]*Ts/2*u[2,i])*tan(u[1,i])/L)
		    @NLconstraint(m, x[4,i+1] == x[4,i] + timeScale[i]*Ts*u[2,i])
	    end
	    if fixTime == 0
	    	@constraint(m, timeScale[i] == timeScale[i+1])
    	end
	end

	u0 = [0,0]
	if fixTime == 1
		for i in 1:N
			if i==1
				@constraint(m,-0.6<=(u0[1]-u[1,i])/Ts <= 0.6)
			else
				@constraint(m,-0.6<=(u[1,i-1]-u[1,i])/Ts <= 0.6)
			end
		end
	else
		for i in 1:N
			if i==1
				@NLconstraint(m,-0.6<=(u0[1]-u[1,i])/(timeScale[i]*Ts) <= 0.6)
			else
				@NLconstraint(m,-0.6<=(u[1,i-1]-u[1,i])/(timeScale[i]*Ts) <= 0.6)
			end
		end
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
			@NLconstraint(m, (sum(Aj[k,1]*lj[k,i] for k = 1 : vOb[j]))^2 + (sum(Aj[k,2]*lj[k,i] for k = 1 : vOb[j]))^2  == 1   )

			# G'*mu + R'*A*lambda = 0
			@NLconstraint(m, (nj[1,i] - nj[3,i]) +  cos(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + sin(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )
			@NLconstraint(m, (nj[2,i] - nj[4,i]) -  sin(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + cos(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )

			# -g'*mu + (A*t - b)*lambda > 0
			@NLconstraint(m, -sum(g[k]*nj[k,i] for k = 1:4) + (x[1,i]+cos(x[3,i])*offset)*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j])
							+ (x[2,i]+sin(x[3,i])*offset)*sum(Aj[k,2]*lj[k,i] for k=1:vOb[j]) - sum(bj[k]*lj[k,i] for k=1:vOb[j]) + sl[j,i]  >= 1*dmin  )
		end
	end

	##############################
	# set initial guesses
	##############################
	if fixTime == 0
		setvalue(timeScale,1*ones(N+1,1))
	end
	setvalue(x,xWS')
	setvalue(u,uWS[1:N,:]')

	lWS,nWS = DualMultWS(N,nOb,vOb, A, b,rx,ry,ryaw)

	setvalue(l,lWS')
	setvalue(n,nWS')


	##############################
	# solve problem
	##############################
	# ipopt has sometimes problems in the restoration phase,
	# it turns out that restarting ipopt with the previous solution
	# as an initial guess works well to achieve a high success rate.
	##############################

	# at most three attempts considered
	time1 = 0
	time2 = 0

	exitflag = 0

	tic()
	status = solve(m; suppress_warnings=true)
	time1 = toq();
	
	# tmp check
	xp = getvalue(x)
	up = getvalue(u)
	if fixTime == 1
		timeScalep = ones(1,N+1)
	else
		timeScalep = getvalue(timeScale)
	end
	lp = getvalue(l)
	np = getvalue(n)
	tmp_useless = ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vOb, A, b,xp,up,lp,np,timeScalep,fixTime,1)
	

	if status == :Optimal
	    exitflag = 1
	elseif status ==:Error || status ==:UserLimit# || status ==:Infeasible
		Feasible = 0
		if Feasible == 0
		    tic()
		    status = solve(m; suppress_warnings=true)
		    time2 = toq();

		    if status == :Optimal
		        exitflag = 1
		    elseif status ==:Error || status ==:UserLimit
		    	xp = getvalue(x)
				up = getvalue(u)
				if fixTime == 1
					timeScalep = ones(1,N+1)
				else
					timeScalep = getvalue(timeScale)
				end
				lp = getvalue(l)
				np = getvalue(n)
				Feasible = 0
				Feasible = ParkingConstraints(x0,xF,N,Ts,L,ego,XYbounds,nOb,vOb, A, b,xp,up,lp,np,timeScalep,fixTime,1)
		        if Feasible == 1
		            exitflag = 1
		        else
		            exitflag = 0
		        end
	        end
	    else
	        exitflag = 1
	    end
	else
	    exitflag = 0
	end

	##############################
	# return values
	##############################

	# computation times is the sum of all trials
	time = time1+time2
	# print("  elapsed time: ")
	# print(time)
	# println(" seconds")

	xp = getvalue(x)
	up = getvalue(u)
	if fixTime == 1
		timeScalep = ones(1,N+1)
	else
		timeScalep = getvalue(timeScale)
	end

	lp = getvalue(l)
	np = getvalue(n)

	return xp, up, timeScalep, exitflag, time, lp, np
end
