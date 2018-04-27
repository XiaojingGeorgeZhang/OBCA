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


function plotTrajQuadcopter(xp,up,N,ego,ob1,ob2,ob3,ob4,ob5,ob6,R,disp_title,figNumOffset)
	###########
	for i = 1:2
		if ob1[i] >= 10
			ob1[i] = 10
		end
	end
	if ob1[3] >= 5
		ob1[3] = 5
	end
	for i = 4:5
		if ob1[i] >= 0
			ob1[i] = 0
		end
	end
	if ob1[6] >= 0
		ob1[6] = 0
	end
	###########
	for i = 1:2
		if ob2[i] >= 10
			ob2[i] = 10
		end
	end
	if ob2[3] >= 5
		ob2[3] = 5
	end
	for i = 4:5
		if ob2[i] >= 0
			ob2[i] = 0
		end
	end
	if ob2[6] >= 0
		ob2[6] = 0
	end
	###########
	for i = 1:2
		if ob3[i] >= 10
			ob3[i] = 10
		end
	end
	if ob3[3] >= 5
		ob3[3] = 5
	end
	for i = 4:5
		if ob3[i] >= 0
			ob3[i] = 0
		end
	end
	if ob3[6] >= 0
		ob3[6] = 0
	end
	###########
	for i = 1:2
		if ob4[i] >= 10
			ob4[i] = 10
		end
	end
	if ob4[3] >= 5
		ob4[3] = 5
	end
	for i = 4:5
		if ob4[i] >= 0
			ob4[i] = 0
		end
	end
	if ob4[6] >= 0
		ob4[6] = 0
	end
	###########
	for i = 1:2
		if ob5[i] >= 10
			ob5[i] = 10
		end
	end
	if ob5[3] >= 5
		ob5[3] = 5
	end
	for i = 4:5
		if ob5[i] >= 0
			ob5[i] = 0
		end
	end
	if ob5[6] >= 0
		ob5[6] = 0
	end

	# println(ob1)
	# println(ob2)
	# println(ob3)
	# println(ob4)
	# println(ob5)
	# println(ob6)

	obcenter1 = [(ob1[1]+ob1[4])/2-ob1[4];
            	 (ob1[2]+ob1[5])/2-ob1[5];
            	 (ob1[3]+ob1[6])/2-ob1[6]]

	obcenter2 = [(ob2[1]+ob2[4])/2-ob2[4];
            	 (ob2[2]+ob2[5])/2-ob2[5];
            	 (ob2[3]+ob2[6])/2-ob2[6]]

	obcenter3 = [(ob3[1]+ob3[4])/2-ob3[4];
            	 (ob3[2]+ob3[5])/2-ob3[5];
            	 (ob3[3]+ob3[6])/2-ob3[6]]

	obcenter4 = [(ob4[1]+ob4[4])/2-ob4[4];
	        	 (ob4[2]+ob4[5])/2-ob4[5];
	        	 (ob4[3]+ob4[6])/2-ob4[6]]

    obcenter5 = [(ob5[1]+ob5[4])/2-ob5[4];
            	 (ob5[2]+ob5[5])/2-ob5[5];
            	 (ob5[3]+ob5[6])/2-ob5[6]]

    obcenter6 = [(ob6[1]+ob6[4])/2-ob6[4];
            	 (ob6[2]+ob6[5])/2-ob6[5];
            	 (ob6[3]+ob6[6])/2-ob6[6]]



	L_tv1 = ob1[1]+ob1[4]
	W_tv1 = ob1[2]+ob1[5]
	H_tv1 = ob1[3]+ob1[6]
	
	L_tv2 = ob2[1]+ob2[4]
	W_tv2 = ob2[2]+ob2[5]
	H_tv2 = ob2[3]+ob2[6]

	L_tv3 = ob3[1]+ob3[4]
	W_tv3 = ob3[2]+ob3[5]
	H_tv3 = ob3[3]+ob3[6]

	L_tv4 = ob4[1]+ob4[4]
	W_tv4 = ob4[2]+ob4[5]
	H_tv4 = ob4[3]+ob4[6]

	L_tv5 = ob5[1]+ob5[4]
	W_tv5 = ob5[2]+ob5[5]
	H_tv5 = ob5[3]+ob5[6]

	L_tv6 = ob6[1]+ob6[4]
	W_tv6 = ob6[2]+ob6[5]
	H_tv6 = ob6[3]+ob6[6]

	for i = 1:1:N
		######### X-Y plot #############
	    figure(1+figNumOffset)
	    # subplot(3,1,1)
	    carBox(obcenter1,0,W_tv1/2,L_tv1/2)
		title("X-Y plot")
	    hold(1)
	    carBox(obcenter2,0,W_tv2/2,L_tv2/2)
	    carBox(obcenter3,0,W_tv3/2,L_tv3/2)
	    carBox(obcenter4,0,W_tv4/2,L_tv4/2)
	    carBox(obcenter5,0,W_tv5/2,L_tv5/2)
	    carBox(obcenter6,0,W_tv6/2,L_tv6/2)


	    x0 = [xp[i,1];
	          xp[i,2]]

	    plot(xp[1:i,1],xp[1:i,2],"b")
	    hold(1)
	    quadCircle(x0,R)

	    axis("equal")
	    axis([-2,12,-2,12])
		hold(0)
		
	    ######### X-Z plot #############
	    figure(2+figNumOffset)
	    carBox(obcenter1[[1,3]],0,H_tv1/2,L_tv1/2)
		title("X-Z plot")
	    hold(1)
	    carBox(obcenter2[[1,3]],0,H_tv2/2,L_tv2/2)
	    carBox(obcenter3[[1,3]],0,H_tv3/2,L_tv3/2)
	    carBox(obcenter4[[1,3]],0,H_tv4/2,L_tv4/2)
	    carBox(obcenter5[[1,3]],0,H_tv5/2,L_tv5/2)
	    carBox(obcenter6[[1,3]],0,H_tv6/2,L_tv6/2)


	    x0 = [xp[i,1];
	          xp[i,3]]

	    plot(xp[1:i,1],xp[1:i,3],"b")
	    hold(1)
	    quadCircle(x0,R)

	    axis("equal")
	    axis([-2,12,-1,6])
	    hold(0)
		
	    # 3D plots
		figure(3+figNumOffset)
	    
	    x0 = [xp[i,1];
	          xp[i,2];
	          xp[i,3]]

	    plot3D(xp[1:i,1],xp[1:i,2],xp[1:i,3],"b")
		title(disp_title)
	    hold(1)
	    Box3D(obcenter1,L_tv1/2,W_tv1/2,H_tv1/2)
	    Box3D(obcenter2,L_tv2/2,W_tv2/2,H_tv2/2)
	    Box3D(obcenter3,L_tv3/2,W_tv3/2,H_tv3/2)
	    Box3D(obcenter4,L_tv4/2,W_tv4/2,H_tv4/2)
	    Box3D(obcenter5,L_tv5/2,W_tv5/2,H_tv5/2)
	    Box3D(obcenter6,L_tv6/2,W_tv6/2,H_tv6/2)
	    quadBall(x0,R)

	    axis("equal")
	    axis([0,10,0,10])
	    zlim([0,5])
	    hold(0)
	    
	    sleep(0.001)
	end

	# for i = 1:1:N
# 	    figure(3+figNumOffset)
#
# 	    x0 = [xp[i,1];
# 	          xp[i,2];
# 	          xp[i,3]]
#
# 	    plot3D(xp[1:i,1],xp[1:i,2],xp[1:i,3],"b")
# 		title(disp_title)
# 	    hold(1)
# 	    Box3D(obcenter1,L_tv1/2,W_tv1/2,H_tv1/2)
# 	    Box3D(obcenter2,L_tv2/2,W_tv2/2,H_tv2/2)
# 	    Box3D(obcenter3,L_tv3/2,W_tv3/2,H_tv3/2)
# 	    Box3D(obcenter4,L_tv4/2,W_tv4/2,H_tv4/2)
# 	    Box3D(obcenter5,L_tv5/2,W_tv5/2,H_tv5/2)
# 	    Box3D(obcenter6,L_tv6/2,W_tv6/2,H_tv6/2)
# 	    quadBall(x0,R)
#
# 	    axis("equal")
# 	    axis([0,10,0,10])
# 	    zlim([0,5])
# 	    hold(0)
#
# 	    sleep(0.01)
# 	end


end


function carBox(x0,phi,w,l)

    car1 = x0[1:2] + [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car2 = x0[1:2] + [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];
    car3 = x0[1:2] - [cos(phi)*l;sin(phi)*l] + [sin(phi)*w;-cos(phi)*w];
    car4 = x0[1:2] - [cos(phi)*l;sin(phi)*l] - [sin(phi)*w;-cos(phi)*w];

    plot([car1[1],car2[1],car4[1],car3[1],car1[1]],[car1[2],car2[2],car4[2],car3[2],car1[2]],"k")
    
end

function Box3D(x0,l,w,h)

	X = x0[1] + [l,l,-l,-l,l]
	Y = x0[2] + [-w,w,w,-w,-w]
	Z = x0[3] + [-h,-h,-h,-h,-h]

    plot3D(X,Y,Z,"k")

    Z = x0[3] + [h,h,h,h,h]
    plot3D(X,Y,Z,"k")

    X = x0[1] + [l,l]
	Y = x0[2] + [-w,-w]
	Z = x0[3] + [-h, h]
	plot3D(X,Y,Z,"k")

	X = x0[1] + [l,l]
	Y = x0[2] + [w,w]
	Z = x0[3] + [-h, h]
	plot3D(X,Y,Z,"k")

	X = x0[1] + [-l,-l]
	Y = x0[2] + [-w,-w]
	Z = x0[3] + [-h, h]
	plot3D(X,Y,Z,"k")

	X = x0[1] + [-l,-l]
	Y = x0[2] + [w,w]
	Z = x0[3] + [-h, h]
	plot3D(X,Y,Z,"k")
    
end

function quadCircle(x0,R)
	phi = linspace(0,2*pi,30);
	X = x0[1] + R*cos(phi)
	Y = x0[2] + R*sin(phi)
	plot(X,Y,"k")

end

function quadBall(x0,R)
	phi = linspace(0,2*pi,30);

	X = x0[1] + R*cos(phi)
	Y = x0[2] + R*sin(phi)
	Z = x0[3]
	plot3D(X,Y,Z,"k")

	X = x0[1] + R*cos(phi)
	Y = x0[2] + zeros(30,1)
	Z = x0[3] + R*sin(phi)
	plot3D(X,Y,Z,"k")

	X = x0[1] + zeros(30,1)
	Y = x0[2] + R*cos(phi)
	Z = x0[3] + R*sin(phi)
	plot3D(X,Y,Z,"k")

end