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
# veloSmooth: a velocity smoother
###############


function veloSmooth(v,amax,Ts)
	v_ex  = zeros(length(v)+40,1)
	v_bar = zeros(4,length(v)+40)
	v_bar2 = zeros(4,length(v)+40)
	v_barMM = zeros(1,length(v))

	for i = 1:length(v)
		for j = 1:4
			v_bar[j,i+19] = v[i];
			v_ex[i+19] = v[i];
		end
	end

	v_cut1 = 0.25*abs(v[1])
	v_cut2 = 0.25*abs(v[1])+abs(v[1])

	accPhase = Int(round(abs(v[1])/amax/Ts))

	index1 = find((diff(v_ex).>v_cut1) & (diff(v_ex).<v_cut2))
	index2 = find(diff(v_ex).>v_cut2)

	index3 = find((diff(v_ex).<-v_cut1) & (diff(v_ex).>-v_cut2))
	index4 = find(diff(v_ex).<-v_cut2)

	if length(index1) >=1 && index1[1]==19
		index1[1] = index1[1]+1
	end
	if length(index3) >=1 && index3[1]==19
		index3[1] = index3[1]+1
	end
	

	for j = 1:length(index1)
		if v_ex[index1[j]] > v_cut1 || v_ex[index1[j]+1] > v_cut1
			v_bar[1,index1[j]:index1[j]+accPhase] = linspace(0,abs(v[1]),accPhase+1)''
		elseif v_ex[index1[j]] < -v_cut1 || v_ex[index1[j]+1] < -v_cut1
			v_bar[1,index1[j]-accPhase+1:index1[j]+1] = linspace(-abs(v[1]),0,accPhase+1)''
		end
	end

	for j = 1:length(index3)
		if v_ex[index3[j]] > v_cut1 || v_ex[index3[j]+1] > v_cut1
			v_bar[2,index3[j]-accPhase+1:index3[j]+1] = linspace(abs(v[1]),0,accPhase+1)''
		elseif v_ex[index3[j]] < -v_cut1 || v_ex[index3[j]+1] < -v_cut1
			v_bar[2,index3[j]:index3[j]+accPhase] = linspace(0,-abs(v[1]),accPhase+1)''
		end
	end

	for j = 1:length(index2)
		v_bar[3,index2[j]-accPhase:index2[j]+accPhase] = linspace(-abs(v[1]),abs(v[1]),2*accPhase+1)''
	end

	for j = 1:length(index4)
		v_bar[4,index4[j]-accPhase:index4[j]+accPhase] = linspace(abs(v[1]),-abs(v[1]),2*accPhase+1)''
	end

	for i = 20:length(v)+19
		for j = 1:4
			if v_bar[j,i] == 0
				v_bar2[j,i] = v_bar[j,i]
			elseif sign(v_ex[i]) != sign(v_bar[j,i])
				v_bar2[j,i] = v_ex[i]
			else
				v_bar2[j,i] = v_bar[j,i]
			end
		end
	end

	for i = 20:length(v)+19
		if v_ex[i] > 0
			v_barMM[i-19] = minimum(v_bar2[:,i])
		else
			v_barMM[i-19] = maximum(v_bar2[:,i])
		end
	end
	
	a = diff(v_barMM')./Ts

	return v_barMM', a

end
