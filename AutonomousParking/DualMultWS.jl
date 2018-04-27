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
# DualMultWS.jl: computes warm starting points for dual multipliers lambda and mu
###############
# 
# 
function DualMultWS(N,nOb,vOb, A, b,rx,ry,ryaw)

	x = zeros(3,N+1)
	x[1,:] = rx
	x[2,:] = ry
	x[3,:] = ryaw

	m = Model(solver=IpoptSolver(hessian_approximation="exact",mumps_pivtol=1e-5,
	                             max_iter=100,tol=1e-5, print_level=0, suppress_all_output="yes"))

	W_ev = ego[2]+ego[4]
	L_ev = ego[1]+ego[3]

	g = [L_ev/2,W_ev/2,L_ev/2,W_ev/2]
	
	# ofset from X-Y to the center of the ego set
	offset = (ego[1]+ego[3])/2 - ego[3]


	@variable(m, l[1:sum(vOb),1:(N+1)])	# dual multiplier associated with obstacleShape
	@variable(m, n[1:nOb*4,1:(N+1)])	# dual multiplier associated with carShape
	@variable(m, d[1:nOb,1:(N+1)])

	@NLobjective(m, Max,sum(sum(d[i,k] for k=1:N+1) for i=1:nOb ))
	
	@constraint(m, l .>= 0 )
	@constraint(m, n .>= 0)

	for i in 1:N+1 	# iterate over time steps
		for j = 1 : nOb 	# iterate over obstacles
			Aj = A[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract obstacle matrix associated with j-th obstacle
			lj = l[sum(vOb[1:j-1])+1 : sum(vOb[1:j]) ,:]	# extract lambda dual variables associate j-th obstacle
			nj = n[(j-1)*4+1:j*4 ,:] 						# extract mu dual variables associated with j-th obstacle
			bj = b[sum(vOb[1:j-1])+1 : sum(vOb[1:j])]		# extract obstacle matrix associated with j-th obstacle

			# norm(A'*lambda) <= 1
			@constraint(m, (sum(Aj[k,1]*lj[k,i] for k = 1 : vOb[j]))^2 + (sum(Aj[k,2]*lj[k,i] for k = 1 : vOb[j]))^2  <= 1   )

			# G'*mu + R'*A*lambda = 0
			@constraint(m, (nj[1,i] - nj[3,i]) +  cos(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + sin(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )
			@constraint(m, (nj[2,i] - nj[4,i]) -  sin(x[3,i])*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j]) + cos(x[3,i])*sum(Aj[k,2]lj[k,i] for k = 1:vOb[j]) == 0  )

			# -g'*mu + (A*t - b)*lambda > 0
			@constraint(m, d[j,i] == -sum(g[k]*nj[k,i] for k = 1:4) + (x[1,i]+cos(x[3,i])*offset)*sum(Aj[k,1]*lj[k,i] for k = 1:vOb[j])
							+ (x[2,i]+sin(x[3,i])*offset)*sum(Aj[k,2]*lj[k,i] for k=1:vOb[j]) - sum(bj[k]*lj[k,i] for k=1:vOb[j]))
		end
	end
	tic()
	solve(m)
	time = toq();
	# print("Auxillery Problem time = ",time,"\n")

	lp = getvalue(l)'
	np = getvalue(n)'

	return lp,np

end