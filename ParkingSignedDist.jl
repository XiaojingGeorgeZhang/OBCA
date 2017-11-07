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
# computes collision-free trajectory by appropriately reformulating the signed distance function
# in case collision cannot be avoided, computes minimum-penetration trajectory
###############



function ParkingSignedDist(x0,xF,N,Ts,L,ego,xWS,uWS,timeWS,XYbounds,nOb,vOb,A,b)

	# desired safety distance
	dmin = 0.05		# anything bigger than 0, e.g. 0.05


	##############################
	# Define JuMP file
	##############################
	# Define IPOPT as solver,
	# as well as solver settings
	##############################
	m = Model(solver=IpoptSolver(hessian_approximation="exact",mumps_pivtol=1e-7,
	                             mumps_mem_percent=6000,max_iter=750,tol=1e-5, print_level=0, suppress_all_output="yes"))

	##############################
	# defining optimization variables
	##############################
	#state
	@variable(m, x[1:4,1:(N+1)])
	#scaling on sampling time
	@variable(m, timeScale[1:N+1])
	#control
	@variable(m, u[1:2,1:(N)])


	@variable(m, l[1:sum(vOb),1:(N+1)])	# dual multiplier associated with obstacleShape
	@variable(m, n[1:nOb*4,1:(N+1)])	# dual multiplier associated with carShape
	@variable(m, sl[1:nOb,1:(N+1)])	# slack variable to avoid infeasibilities


	reg = 1e-4;
	##############################
	# cost function
	##############################
	# (min control inputs)+      
	# (min time)+
	# (regularization dual variables)
	##############################
	 @NLobjective(m, Min,sum(0.1*u[1,i]^2 + 1*u[2,i]^2 for i = 1:N) + 
	                     sum(0.5*timeScale[i] + 1*timeScale[i]^2 for i = 1:N+1)+
	                     sum(sum(reg*n[j,i]^2 for i = 1:N+1)  for j = 1:4) +
	 					 sum(sum(reg*l[j,i]^2 for i = 1:N+1)  for j = 1:sum(vOb)) +
	 					 sum(sum(1e4*sl[j,i] + 1e5*sl[j,i]^2 for i = 1:N+1) for j = 1 : nOb)  )

	##############################
	# bounds on states, inputs,
	# and dual multipliers.
	##############################
	#input constraints
	@constraint(m, [i=1:N], -0.6 <= u[1,i] <= 0.6)
	@constraint(m, [i=1:N], -1   <= u[2,i] <= 1)

	#state constraints
	@constraint(m, [i=1:N+1], XYbounds[1] <= x[1,i] <= XYbounds[2])
	@constraint(m, [i=1:N+1], XYbounds[3] <= x[2,i] <= XYbounds[4])
	@constraint(m, [i=1:N+1], -7  <= x[3,i] <= 7)
	@constraint(m, [i=1:N+1], -1  <= x[4,i] <= 2)
	# bounds on time scaling
	@constraint(m, 0.5 .<= timeScale .<= 2.5)

	# positivity constraints on dual multipliers
	@constraint(m, l .>= 0)
	@constraint(m, n .>= 0)

	# positivity constraints on slack multipliers
	@constraint(m, sl.>= 0)
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
	    @NLconstraint(m, x[1,i+1] == x[1,i] + timeScale[i]*Ts*x[4,i]*cos(x[3,i]))
	    @NLconstraint(m, x[2,i+1] == x[2,i] + timeScale[i]*Ts*x[4,i]*sin(x[3,i]))
	    @NLconstraint(m, x[3,i+1] == x[3,i] + timeScale[i]*Ts*x[4,i]*tan(u[1,i])/L)
	    @NLconstraint(m, x[4,i+1] == x[4,i] + timeScale[i]*Ts*u[2,i])

	    @constraint(m, timeScale[i] == timeScale[i+1])
	end

	##############################
	# obstacle avoidance constraints
	##############################

	#width and length of ego set
	W_ev = ego[2]+ego[4]
	L_ev = ego[1]+ego[3]

	g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2]
	# ofset from X-Y to the center of the ego set
	offset = (ego[1]+ego[3])/2 - ego[3]


	for i in 1:N+1

		for j = 1 : nOb
			Aj = A[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract obstacle matrix associated with j-th obstacle
			lj = l[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract lambda dual variables associate j-th obstacle
			nj = n[(j-1)*4+1:j*4 ,:] # extract mu dual variables associated with j-th obstacle
			bj = b[sum(vOb[1:j-1])+1 : sum(vOb[1:j])]		# extract obstacle matrix associated with j-th obstacle

			# norm(A'*lambda) <= 1
			@NLconstraint(m, (sum(Aj[k,1]*lj[k,i] for k = 1 : vOb[j]))^2 + (sum(Aj[k,2]*lj[k,i] for k = 1 : vOb[j]))^2  == 1   )

			# G'*mu + R'*A*lambda = 0
			@NLconstraint(m, (nj[1,i] - nj[3,i]) +  cos(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + sin(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )
			@NLconstraint(m, (nj[2,i] - nj[4,i]) -  sin(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + cos(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )

			# -g'*mu + (A*t - b)*lambda > 0
			@NLconstraint(m, -sum(g[k]*nj[k,i] for k = 1:4) + (x[1,i]+cos(x[3,i])*offset)*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j])
							+ (x[2,i]+sin(x[3,i])*offset)*sum(Aj[k,2]*lj[k,i] for k=1:vOb[j]) - sum(bj[k]*lj[k,i] for k=1:vOb[j]) + sl[j,i]  >= dmin  )


		end
	end

	##############################
	# set initial guess
	##############################
	setvalue(timeScale,timeWS)

	setvalue(x,xWS)
	setvalue(u,uWS) 


	##############################
	# solve problem
	##############################
	# ipopt has sometimes problems in the restoration phase,
	# it turns out that restarting ipopt with the previous solution
	# as an initial guess works well to achieve a high success rate.
	##############################
	time1 = 0
	time2 = 0
	time3 = 0

	exitflag = 0

	tic()
	status = solve(m; suppress_warnings=true);
	time1 = toq()


	if status == :Optimal
	    exitflag = 1
	elseif status ==:Error || status ==:UserLimit
	    tic()
	    status = solve(m; suppress_warnings=true)  # automatically resolves with last solution
	    time2 = toq()

	    if status == :Optimal
	        exitflag = 1
	    elseif status ==:Error || status ==:UserLimit
	        tic()
	        status = solve(m; suppress_warnings=true)
	        time3 = toq()
	        if status == :Optimal
	            exitflag = 1
	        else
	            exitflag = 0
	        end
	    else
	        exitflag = 0
	    end
	else
	    exitflag = 0
	end



	##############################
	# return values
	##############################

	# computation times is the sum of all trials
	time = time1+time2+time3
	print("  elapsed time: ")
	print(time)
	println(" seconds")

	xp = getvalue(x)
	up = getvalue(u)
	timeScalep = getvalue(timeScale)

	return xp, up, timeScalep, exitflag, time

end
