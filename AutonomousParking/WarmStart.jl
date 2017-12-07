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
# Computes trajectory by ignoring the obstacles
###############


function WarmStart(N,Ts,L,x0,xF,XYbounds)

	##############################
	# Define JuMP file
	##############################
	# Define IPOPT as solver,
	# as well as solver settings
	##############################
	m = Model(solver=IpoptSolver(hessian_approximation="exact",mumps_pivtol=1e-7,
	                             mumps_mem_percent=6000,max_iter=750,tol=1e-5, print_level=0))

	##############################
	# defining optimization variables
	##############################
	#state variables
	@variable(m, x[1:4,1:(N+1)])
	#scaling on sampling time
	@variable(m, timeScale[1:N+1])
	#control inputs
	@variable(m, u[1:2,1:(N)])

	##############################
	# cost function
	##############################
	# (min control inputs)+      
	# (min time)+
	##############################
	 @NLobjective(m, Min,sum(0.1*u[1,i]^2 + 1*u[2,i]^2 for i = 1:N) + 
	                     sum(0.5*timeScale[i] + 1*timeScale[i]^2 for i = 1:N+1)) 


	##############################
	# bounds on states and inputs
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
	# - kinematic bicycle model with forward Euler integration
	# - sampling time scaling, is identical over the horizon

	for i in 1:N
	    @NLconstraint(m, x[1,i+1] == x[1,i] + timeScale[i]*Ts*x[4,i]*cos(x[3,i]))
	    @NLconstraint(m, x[2,i+1] == x[2,i] + timeScale[i]*Ts*x[4,i]*sin(x[3,i]))
	    @NLconstraint(m, x[3,i+1] == x[3,i] + timeScale[i]*Ts*x[4,i]*tan(u[1,i])/L)
	    @NLconstraint(m, x[4,i+1] == x[4,i] + timeScale[i]*Ts*u[2,i])

	    @constraint(m, timeScale[i] == timeScale[i+1])
	end



	##############################
	# set initial guess
	##############################
	for b = 1:4
	    xWS[b,:] = linspace(x0[b],xF[b],N+1)
	end

	uWS = 0.6*ones(2,N)

	setvalue(timeScale,0.5*ones(N+1,1))
	setvalue(x,xWS)
	setvalue(u,uWS)


	##############################
	# solve problem
	##############################
	time = 0

	tic()
	status = solve(m)
	time = toq()

	# print solution time
	print("  elapsed time: ")
	print(time)
	println(" seconds")


	##############################
	# return values
	##############################
	xp = getvalue(x)
	up = getvalue(u)
	timeScalep = getvalue(timeScale)

	return xp, up, timeScalep

end
