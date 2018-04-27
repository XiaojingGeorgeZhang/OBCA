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


function QuadcopterSignedDist(x0,xF,N,Ts,R,ob1,ob2,ob3,ob4,ob5,xWS,uWS,timeWS)

	# define solver
 	m = Model(solver=IpoptSolver(hessian_approximation="exact",mumps_pivtol=5e-7,mumps_pivtolmax=0.1,mumps_mem_percent=10000,
 	                             recalc_y="no",alpha_for_y="min",required_infeasibility_reduction=0.6,
 	                             min_hessian_perturbation=1e-10,jacobian_regularization_value=1e-7,tol=1e-5,
 	                             print_level=0))#state


	@variable(m, x[1:12,1:(N+1)])
	@variable(m, timeScale[1:N+1])
	#control
	@variable(m, u[1:4,1:(N)])
	# lagrange multipliers for dual dist function
	@variable(m, l1[1:6,1:(N+1)])

	@variable(m, l2[1:6,1:(N+1)])

	@variable(m, l3[1:6,1:(N+1)])

	@variable(m, l4[1:6,1:(N+1)])

	@variable(m, l5[1:6,1:(N+1)])

	@variable(m, slack[1:5,1:(N+1)])

	mass = 0.5;
	g = 9.81;
	reg = 0;
	reg2 = 1e-4;
	reg3 = 0.0001;

	k_F = 0.0611;#6.11*1e-8         #[N/rpm^2]
	k_M = 0.0015;#1.5*1e-9          #[Nm/rpm^2]
	I = [3.9,4.4,4.9]*1e-3  #[kg/m2]
	L = 0.225               #[m]

	w_H = sqrt((mass*g)/(k_F*4))

	# cost function
	 @NLobjective(m, Min,1e-3*sum( sum((w_H-u[j,i])^2 for j=1:4)  for i = 1:N) + 
	                     1e-2*sum( sum((u[j,i]-u[j,i+1])^2 for j=1:4)  for i = 1:N-1) + 
	                     1*sum(sum(reg3*x[j,i]^2  for i = 1:N+1) for j = [10,11,12]) +
	                     sum(0.25*timeScale[i] + 5*timeScale[i]^2 for i = 1:N+1)  +
	                     sum(sum(1e2*slack[j,i] + 1e3*slack[j,i]^2 for i = 1:N+1) for j = 1:5) +
						 # sum(sum(8e0*slack[j,i] + 1e2*slack[j,i]^2 for i = 1:N+1) for j = 1:5) +
	                     1*sum(sum(reg2*l1[j,i]^2 + reg2*l2[j,i]^2 + reg2*l3[j,i]^2 + reg2*l4[j,i]^2+ reg2*l5[j,i]^2 for i = 1:N+1) for j = 1:6));

	#input constraints
	@constraint(m, [i=1:N],  1.200  <= u[1,i] <= 7.800)
	@constraint(m, [i=1:N],  1.200  <= u[2,i] <= 7.800)
	@constraint(m, [i=1:N],  1.200  <= u[3,i] <= 7.800)
	@constraint(m, [i=1:N],  1.200  <= u[4,i] <= 7.800)
	#state constraints
	#X,Y,Z
	@constraint(m, [i=1:N+1], 0 <= x[1,i] <= 10)
	@constraint(m, [i=1:N+1], 0 <= x[2,i] <= 10)        # -0.1 <= x[2,i] <= 20
	@constraint(m, [i=1:N+1], 0 <= x[3,i] <=  5)
	# pitch, roll
	@constraint(m, [i=1:N+1], -3 <= x[4,i] <=  3)
	@constraint(m, [i=1:N+1], -0.2 <= x[5,i] <=  0.2) #pm 0.2
	@constraint(m, [i=1:N+1], -0.2 <= x[6,i] <=  0.2)
	#v_x, v_y, v_z
	@constraint(m, [i=1:N+1],-1 <= x[7,i] <=  1)
	@constraint(m, [i=1:N+1],-1 <= x[8,i] <=  1)
	@constraint(m, [i=1:N+1],-1 <= x[9,i] <=  1)
	# pitch_rate, roll_rate
	@constraint(m, [i=1:N+1],-1 <= x[10,i] <= 1) #pm 1
	@constraint(m, [i=1:N+1],-1 <= x[11,i] <= 1)
	@constraint(m, [i=1:N+1],-1 <= x[12,i] <= 1)

	@constraint(m, 0.5 .<= timeScale .<= 2)
	# positivity constraints on lambda
	@constraint(m, l1.>= 0)
	@constraint(m, l2.>= 0)
	@constraint(m, l3.>= 0)
	@constraint(m, l4.>= 0)
	@constraint(m, l5.>= 0)


	@constraint(m, slack.>= 0)

	#starting point
	@constraint(m, x[1,1] == x0[1])
	@constraint(m, x[2,1] == x0[2])
	@constraint(m, x[3,1] == x0[3])
	@constraint(m, x[4,1] == x0[4])
	@constraint(m, x[5,1] == x0[5])
	@constraint(m, x[6,1] == x0[6])
	@constraint(m, x[7,1] == x0[7])
	@constraint(m, x[8,1] == x0[8])
	@constraint(m, x[9,1] == x0[9])
	@constraint(m, x[10,1] == x0[10])
	@constraint(m, x[11,1] == x0[11])
	@constraint(m, x[12,1] == x0[12])


	#end point
	@constraint(m, x[1,N+1] == xF[1])
	@constraint(m, x[2,N+1] == xF[2])
	@constraint(m, x[3,N+1] == xF[3])
	@constraint(m, x[4,N+1] == xF[4])
	@constraint(m, x[5,N+1] == xF[5])
	@constraint(m, x[6,N+1] == xF[6])
	@constraint(m, x[7,N+1] == xF[7])
	@constraint(m, x[8,N+1] == xF[8])
	@constraint(m, x[9,N+1] == xF[9])
	@constraint(m, x[10,N+1] == xF[10])
	@constraint(m, x[11,N+1] == xF[11])
	@constraint(m, x[12,N+1] == xF[12])

	for i in 1:N
	    #X,Y,Z
	    @NLconstraint(m, x[1,i+1] == x[1,i] + timeScale[i]*Ts*x[7,i])
	    @NLconstraint(m, x[2,i+1] == x[2,i] + timeScale[i]*Ts*x[8,i])
	    @NLconstraint(m, x[3,i+1] == x[3,i] + timeScale[i]*Ts*x[9,i])

	    # pitch, roll
	    @NLconstraint(m, x[4,i+1] == x[4,i] + timeScale[i]*Ts*( cos(x[5,i])            *x[10,i]        +sin(x[5,i])            *x[12,i]))
	    @NLconstraint(m, x[5,i+1] == x[5,i] + timeScale[i]*Ts*( sin(x[5,i])*tan(x[4,i])*x[10,i]+x[11,i]-cos(x[5,i])*tan(x[4,i])*x[12,i]))
	    @NLconstraint(m, x[6,i+1] == x[6,i] + timeScale[i]*Ts*(-sin(x[5,i])*sec(x[4,i])*x[10,i]        +cos(x[5,i])*sec(x[4,i])*x[12,i]))

	    #v_x, v_y, v_z
	    @NLconstraint(m, x[7,i+1] == x[7,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*( sin(x[4,i])*cos(x[5,i])*sin(x[6,i]) + sin(x[5,i])*cos(x[6,i]) )))
	    @NLconstraint(m, x[8,i+1] == x[8,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*(-sin(x[4,i])*cos(x[5,i])*cos(x[6,i]) + sin(x[5,i])*sin(x[6,i]) )))
	    @NLconstraint(m, x[9,i+1] == x[9,i] + timeScale[i]*Ts*1/mass*(sum(k_F*u[j,i]^2 for j=1:4)*( cos(x[4,i])*cos(x[5,i])) - mass*g ))

	    # pitch_rate, roll_rate
	    @NLconstraint(m, x[10,i+1] == x[10,i] + timeScale[i]*Ts*1/I[1]*(L*k_F*(u[2,i]^2 - u[4,i]^2)                     - (I[3] - I[2])*x[11]*x[12]))
	    @NLconstraint(m, x[11,i+1] == x[11,i] + timeScale[i]*Ts*1/I[2]*(L*k_F*(u[3,i]^2 - u[1,i]^2)                     - (I[1] - I[3])*x[10]*x[12]))
	    @NLconstraint(m, x[12,i+1] == x[12,i] + timeScale[i]*Ts*1/I[3]*(k_M*(u[1,i]^2 - u[2,i]^2 + u[3,i]^2 - u[4,i]^2) - (I[2] - I[1])*x[10]*x[11]))

	    @constraint(m, timeScale[i] == timeScale[i+1])
	end



	A = [eye(3);
	    -eye(3)];

	for i in 1:N+1
	    # rotation matrix

	    b1 = ob1
	    @NLconstraint(m, (l1[1,i]-l1[4,i])^2 + (l1[2,i]-l1[5,i])^2 + (l1[3,i]-l1[6,i])^2 == 1)
	    @NLconstraint(m,sum(-b1[j]*l1[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l1[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l1[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l1[j,i] for j=1:6) + 0.01*slack[1,i]>=R)

	    ######################
	    b2 = ob2
	    @NLconstraint(m, (l2[1,i]-l2[4,i])^2 + (l2[2,i]-l2[5,i])^2 + (l2[3,i]-l2[6,i])^2 == 1)
	    @NLconstraint(m,sum(-b2[j]*l2[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l2[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l2[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l2[j,i] for j=1:6) + 0.01*slack[2,i]>=R)

	    #########################
	    b3 = ob3
	    @NLconstraint(m, (l3[1,i]-l3[4,i])^2 + (l3[2,i]-l3[5,i])^2 + (l3[3,i]-l3[6,i])^2 == 1)
	    @NLconstraint(m,sum(-b3[j]*l3[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l3[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l3[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l3[j,i] for j=1:6) + 0.01*slack[3,i]>=R)

	    #########################
	    b4 = ob4
	    @NLconstraint(m, (l4[1,i]-l4[4,i])^2 + (l4[2,i]-l4[5,i])^2 + (l4[3,i]-l4[6,i])^2 == 1)
	    @NLconstraint(m,sum(-b4[j]*l4[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l4[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l4[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l4[j,i] for j=1:6) + 0.01*slack[4,i]>=R)

	    #########################
	    b5 = ob5
	    @NLconstraint(m, (l5[1,i]-l5[4,i])^2 + (l5[2,i]-l5[5,i])^2 + (l5[3,i]-l5[6,i])^2 == 1)
	    @NLconstraint(m,sum(-b5[j]*l5[j,i] for j = 1:6) + x[1,i]*sum(A[j,1]*l5[j,i] for j=1:6) + 
	                         x[2,i]*sum(A[j,2]*l5[j,i] for j=1:6) + x[3,i]*sum(A[j,3]*l5[j,i] for j=1:6) + 0.01*slack[5,i]>=R)

	end

	setvalue(timeScale,timeWS*ones(N+1,1))

	setvalue(x,xWS)
	setvalue(u,w_H*ones(4,N)) 		# faster not to warm-start

	setvalue(l1,0.05*ones(6,N+1))
	setvalue(l2,0.05*ones(6,N+1))
	setvalue(l3,0.05*ones(6,N+1))
	setvalue(l4,0.05*ones(6,N+1))
	setvalue(l5,0.05*ones(6,N+1))

	setvalue(slack,1*ones(5,N+1))		# setvalue(slack,0.1*ones(5,N+1))
	# setvalue(slack,zeros(5,N+1))	# slows down solver very much


	time1 = 0
	time2 = 0
	time3 = 0
	time4 = 0


	tic()
	# status = solve(m; suppress_warnings=true)
	status = solve(m)
	time1 = toq()

	# println(time1)

	flag = 1;
	
	# println("solver status after 1 trial: ", status)
	if flag == 1
	    if status == :Optimal
	        exitflag = 1
	    else
	        exitflag = 0
	    end
	elseif flag == 2
	    if status == :Optimal
	        exitflag = 1
	    elseif status ==:Error || status ==:UserLimit
	        tic()
	        status = solve(m; suppress_warnings=true)
	        time2 = toq()

	        if status == :Optimal
	            exitflag = 1
	        elseif status ==:Error || status ==:UserLimit
	            tic()
	            status = solve(m; suppress_warnings=true)
	            time3 = toq()

	            if status == :Optimal
	                exitflag = 1
	            elseif status ==:Error || status ==:UserLimit

	                tic()
	                status = solve(m; suppress_warnings=true)
	                time4 = toq()

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
	    else
	        exitflag = 0
	    end
	end

	time = time1+time2+time3+time4

	xp = getvalue(x)
	up = getvalue(u)
	timeScalep = getvalue(timeScale)

	slackp = getvalue(slack)

	sumSlack = sum(slackp)
	# println(sumSlack)
	if exitflag == 1 && sumSlack > 1e-3
		println("sum-slack condition not satisfied")
	    exitflag = 2
	end

	l1p = getvalue(l1)
	l2p	= getvalue(l2)
	l3p = getvalue(l3)
	l4p = getvalue(l4)
	l5p = getvalue(l5)
	lp = [l1p ; l2p ; l3p ; l4p ; l5p]


	return xp, up, timeScalep, exitflag, time, lp, string(status)

end

