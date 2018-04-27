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
# Function computes H-representation for obstacles given their vertices
# it is assumed that the vertices are given in CLOCK-WISE, and that the first vertex is repeated at the end of the vertex list
###############



function obstHrep(nOb, vOb, lOb)

	# do simple checks
	if nOb != length(lOb)
		println("ERROR in number of obstacles")
	end

	# these matrices contain the H-rep
	A_all = zeros(sum(vOb)-nOb,2)
	b_all = zeros(sum(vOb)-nOb,1)

	# counter for lazy people
	lazyCounter = 1;

	for i = 1 : nOb	# building H-rep
		A_i = zeros(vOb[i]-1,2)
		b_i = zeros(vOb[i]-1,1)
	
		# take two subsequent vertices, and compute hyperplane
		for j = 1 : vOb[i]-1

			# extract two vertices
			v1 = lOb[i][j]		# vertex 1
			v2 = lOb[i][j+1]	# vertex 2
		
			# find hyperplane passing through v1 and v2
			if v1[1] == v2[1]	# perpendicular hyperplane, not captured by general formula
				if v2[2] < v1[2]   # line goes "down"
					A_tmp = [1 0]
					b_tmp = v1[1]
				else
					A_tmp = [-1 0]
					b_tmp = -v1[1]
				end
			elseif v1[2] == v2[2] # horizontal hyperplane, captured by general formula but included for numerical stability
				if v1[1] < v2[1]
					A_tmp = [0 1]
					b_tmp = v1[2]
				else
					A_tmp = [0 -1]
					b_tmp = -v1[2]
				end
			else   # general formula for non-horizontal and non-vertical hyperplanes
				ab = [v1[1] 1 ; v2[1] 1] \ [v1[2] ; v2[2]]
				a = ab[1]
				b = ab[2]
			
				if v1[1] < v2[1]  # v1 --> v2 (line moves right)
					A_tmp = [-a 1]
					b_tmp = b
				else  # v2 <-- v1 (line moves left)
					A_tmp = [a -1]
					b_tmp = -b
				
				end
			end
			# store vertices
			A_i[j,:] = A_tmp
			b_i[j] = b_tmp
		end
	
		# store everything
		A_all[lazyCounter : lazyCounter+vOb[i]-2,:] = A_i
		b_all[lazyCounter : lazyCounter+vOb[i]-2] = b_i
	
		# update counter
		lazyCounter = lazyCounter + vOb[i]-1
	end

	return A_all, b_all

end
