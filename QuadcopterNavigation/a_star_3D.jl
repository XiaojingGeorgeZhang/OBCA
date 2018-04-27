###############
# OBCA: Optimization-based Collision Avoidance - a path planner for autonomous parking
# Copyright (C) 2018
# Atsushi SAKAI [atsushisakai@global.komatsu; Komatsu Ltd / MPC Lab]
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

module a_star	# new scope, good for defining global variables

using NearestNeighbors, JuMP, Ipopt
using PyPlot

const VEHICLE_RADIUS = 2.5# const GRID_RESOLUTION = 1.0 #[m], def 2.0
const H_WEIGHT = 1.1	# weight for heuristic function

type Node
    x::Int64  #x index
    y::Int64  #y index
	z::Int64  #z index (added)	
    cost::Float64 # cost
    pind::Int64 # parent index
end

# Only thing you need to do is that using calc_astar_path() with inputs.
# sx, sy, sz: start point
# gx, gy, gz: goal point
# ox, oy, oz: obstacle position list
# reso: grid resolution of A*.

# (sx, sy, sz, gx, gy, gz, ox, oy, oz, xmin, ymin, zmin, xmax, ymax, zmax, GRID_RESOLUTION)

function calc_astar_path(sx::Float64, sy::Float64, sz::Float64, gx::Float64, gy::Float64, gz::Float64,
                         ox::Array{Float64}, oy::Array{Float64}, oz::Array{Float64}, 
						 xmin::Float64, ymin::Float64, zmin::Float64,
						 xmax::Float64, ymax::Float64, zmax::Float64,
						 reso::Float64)
    """
    sx: start x position [m]
    sy: start y position [m]
	sz: start z position [m]
    gx: goal x position [m]
    gy: goal y position [m]
	gz: goal z position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
	oz: z position list of Obstacles [m]
    reso: grid resolution [m]
    """
    tic()
	
    nstart = Node(Int(round(sx/reso)),Int(round(sy/reso)),Int(round(sz/reso)),0.0, -1)
    ngoal = Node(Int(round(gx/reso)),Int(round(gy/reso)),Int(round(gz/reso)),0.0, -1)
	

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]
	oz = [ioz/reso for ioz in oz]
	

    obmap, minx, miny, minz, maxx, maxy, maxz, xw, yw, zw = calc_obstacle_map(ox, oy, oz, xmin, ymin, zmin, xmax, ymax, zmax, reso)
    miniTime = toq();
    # print("MiniTime ",miniTime,"\n")
    tic()
    #open, closed set
    open, closed = Dict{Int64, Node}(), Dict{Int64, Node}()

    pqOpen = Collections.PriorityQueue(Int64,Float64)
	
    open[calc_index(nstart, xw, zw, minx, miny, minz)] = nstart	# ??????; weird if start is minx miny minz -> zero indexing!
    Collections.enqueue!(pqOpen,calc_index(nstart, xw, zw, minx, miny, minz),nstart.cost+H_WEIGHT*h(nstart.x - ngoal.x, nstart.y - ngoal.y, nstart.z-ngoal.z))


    motion = get_motion_model()
    nmotion = length(motion[:,1])
	
	tmpCounter = 1
    while true
        if length(open) == 0
			println("Error: No open set")
			break
		end

		c_id = Collections.dequeue!(pqOpen)
		current = open[c_id]


        if current.x == ngoal.x && current.y == ngoal.y && current.z == ngoal.z # check goal
            # println("Path found by A star!!")
            closed[c_id] = current
            break
        end

       	delete!(open, c_id)
        closed[c_id] = current

        for i in 1:nmotion # expand search grid based on motion model
            node = Node(current.x+motion[i,1], current.y+motion[i,2], current.z+motion[i,3], current.cost+motion[i,4], c_id)

            # check boundary
            if (node.x - minx) >= xw continue end
            if (node.x - minx) <= 0 continue end
            if (node.y - miny) >= yw continue end
            if (node.y - miny) <= 0 continue end
            if (node.z - minz) >= zw continue end
            if (node.z - minz) <= 0 continue end
			
			#collision check
            if obmap[node.x-minx+1, node.y-miny+1, node.z-minz+1] continue end 

            node_ind = calc_index(node, xw, zw, minx, miny, minz)

            # If it is already in the closed set, skip it
            if haskey(closed,node_ind)  continue end

            if haskey(open, node_ind) 	# check if in open set
                if open[node_ind].cost > node.cost
                    # If so, update the node to have a new parent
                    open[node_ind].cost = node.cost
                    open[node_ind].pind = c_id
                    pqOpen[node_ind] = node.cost+H_WEIGHT*h(node.x - ngoal.x, node.y - ngoal.y, node.z-ngoal.z)

                end
            else # add to open set
                open[node_ind] = node
                Collections.enqueue!(pqOpen,node_ind,node.cost+H_WEIGHT*h(node.x - ngoal.x, node.y - ngoal.y, node.z-ngoal.z))
            end
        end		# end nmotion

	
	tmpCounter = tmpCounter + 1
	
    end
	runTime = toq()
    rx, ry, rz = get_final_path(closed, ngoal, nstart, xw, zw, minx, miny, minz, reso)

    return rx, ry, rz, runTime
end


function get_motion_model()
	# 			dx,  dy,  dz,   cost
	motion=[ 	-1   -1   -1    sqrt(3);
				-1	 -1    0	sqrt(2);
				-1   -1    1	sqrt(3);
				-1    0   -1	sqrt(2);
				-1    0    0    1;
				-1    0    1	sqrt(2);
				-1    1   -1	sqrt(3);
				-1    1    0	sqrt(2);
				-1    1    1	sqrt(3);
				 0   -1   -1	sqrt(2);
				 0   -1    0	1;
				 0   -1    1	sqrt(2);
				 0    0   -1	1;
				 0    0    1	1;
				 0    1   -1	sqrt(2);
				 0    1    0	1;
				 0    1    1 	sqrt(2);
      		     1   -1   -1	sqrt(3);
				 1   -1    0	sqrt(2);
				 1   -1    1	sqrt(3);
				 1    0   -1	sqrt(2);
				 1    0    0	1;
				 1    0    1	sqrt(2);
				 1    1   -1	sqrt(3);
				 1    1    0	sqrt(2);
				 1    1    1	sqrt(3)	]

    return motion
end

function calc_index(node::Node, xwidth::Int, zwidth::Int, xmin::Int, ymin::Int64, zmin::Int64)
	return (node.y - ymin)*xwidth*zwidth + (node.x - xmin)*zwidth + (node.z-zmin)
end

function calc_obstacle_map(	ox::Array{Float64}, oy::Array{Float64}, oz::Array{Float64},
							xmin::Float64, ymin::Float64, zmin::Float64,
							xmax::Float64, ymax::Float64, zmax::Float64,   reso::Float64)
	# for easier handling
	push!(ox,xmin,xmax)
	push!(oy,ymin,ymax)
	push!(oz,zmin,zmax)
	
    minx = Int(round(minimum(ox)))
    miny = Int(round(minimum(oy)))
	minz = Int(round(minimum(oz))) 
	maxx = Int(round(maximum(ox)))
    maxy = Int(round(maximum(oy)))
	maxz = Int(round(maximum(oz)))
	
    xwidth = Int(maxx - minx)
    ywidth = Int(maxy - miny)
	zwidth = Int(maxz - minz)

    obmap = fill(false, (xwidth,ywidth,zwidth))

    kdtree = KDTree(hcat(ox, oy, oz)')
    for ix in 1:xwidth 
        x = (ix-1) + minx
        for iy in 1:ywidth 
            y = (iy-1) + miny
			for iz in 1:zwidth
				z = (iz-1) + minz
				
				idxs, onedist = knn(kdtree, [x, y, z] , 1)
            	if onedist[1] <= VEHICLE_RADIUS/reso 
                	obmap[ix,iy,iz] = true
            	end
			end
        end
    end

    return obmap, minx, miny, minz, maxx, maxy, maxz, xwidth, ywidth, zwidth
end

function get_final_path(closed::Dict{Int64, Node},
                        ngoal::Node,
                        nstart::Node,
                        xw::Int64,
						zw::Int64,	# new
                        minx::Int64,
                        miny::Int64,
						minz::Int64,	# new
                        reso::Float64)

    rx, ry ,rz = [ngoal.x],[ngoal.y], [ngoal.z]
	
    nid = calc_index(ngoal, xw, zw, minx, miny, minz)
    while true
		n = closed[nid]
        push!(rx, n.x)
        push!(ry, n.y)
		push!(rz, n.z)
        nid = n.pind

        if rx[end] == nstart.x && ry[end] == nstart.y && rz[end] == nstart.z
            # println("done reconstructing path")
            break
        end
    end

    rx = reverse(rx) .* reso
    ry = reverse(ry) .* reso
	rz = reverse(rz) .* reso

    return rx, ry, rz
end


function search_min_cost_node(open::Dict{Int64, Node}, ngoal::Node,Hmat)
    mnode = nothing
    mcost = Inf
	
	# find best node in open set
    for n in values(open)
        # println("candidate node:", n)
        cost = n.cost + H_WEIGHT*Hmat[Int(n.x+1), Int(n.y+1), Int(n.z+1)]	# compute gScore + hScore (cost from start to n + heuristics)
        if mcost > cost
            mnode = n 
            mcost = cost 
        end
    end

    return mnode
end


function h(x::Int, y::Int, z::Int)
    """
    Heuristic cost function
    """
    return sqrt(x^2 + y^2 + z^2);
end


# Only thing you need to do is that using calc_astar_path() with inputs.
# sx, sy is start point
# gx, gy is a goal point
# ox, oy is obstacle position lists
# and reso means grid resolution of A*.

function main()
	close("all")
    println(PROGRAM_FILE," start A-star!!")
	i = 0
	horizonLengths = ones(100,1)
	for yy = 10 : 10 : 10		# 90
		for zz = 10 : 10 : 10 	# 40
			i = i+1
			
			# all FLOAT for performance
			# everthing in [m] for convenience
			xmin = 0.0
			ymin = 0.0
			zmin = 0.0
			xmax = 105.0
			ymax = 105.0
			zmax = 55.0
	
		    sx = 10.0  # [m]
		    sy = 10.0  # [m]
			sz = 30.0  # [m]
	
		    gx = 90.0  # [m]
		    gy = 80.0  # [m]
			gy = Float64(yy)
			gz = 40.0  # [m]
			gz = Float64(zz)
			# build obstacles
	
			println("gy: ", gy)
			println("gz: ", gz)
	
		    ox = Float64[]
		    oy = Float64[]
			oz = Float64[]

			# first obstacle
			for xx in 20 : 25
				for yy in 0 : 105
					for zz in 6 : 55
						push!(ox,Float64(xx))
						push!(oy,Float64(yy))
						push!(oz,Float64(zz))	
					end
				end	
			end

			# second obstacle
		    ox1 = Float64[]
		    oy1 = Float64[]
			oz1 = Float64[]
	
			for xx = 70 : 75
				# left piece
				for yy = 0 : 40
					for zz = 0 : 55
						push!(ox, Float64(xx))
						push!(oy, Float64(yy))
						push!(oz, Float64(zz))
			
						push!(ox1, Float64(xx))
						push!(oy1, Float64(yy))
						push!(oz1, Float64(zz))
					end
				end
				# right piece
				for yy = 50 : 105
					for zz = 0 : 55
						push!(ox, Float64(xx))
						push!(oy,Float64(yy))
						push!(oz,Float64(zz))
			
						push!(ox1, Float64(xx))
						push!(oy1,Float64(yy))
						push!(oz1,Float64(zz))
					end
				end
				# top piece
				for yy = 40 : 50
					for zz = 30 : 55
						push!(ox, Float64(xx))
						push!(oy,Float64(yy))
						push!(oz,Float64(zz))
			
						push!(ox1, Float64(xx))
						push!(oy1,Float64(yy))
						push!(oz1,Float64(zz))
					end
				end
				# right piece
				for yy = 40 : 50
					for zz = 0 : 20
						push!(ox, Float64(xx))
						push!(oy,Float64(yy))
						push!(oz,Float64(zz))
			
						push!(ox1, Float64(xx))
						push!(oy1,Float64(yy))
						push!(oz1,Float64(zz))
					end
				end
			end
			
		    rx, ry, rz = calc_astar_path(	sx, sy, sz, 		# start
												gx, gy, gz, 		# goal
												ox, oy, oz, 		# list of obstacles
												xmin, ymin, zmin, 	# box constraint
												xmax, ymax, zmax, 	# box constraint
												1.0	)	# other relevant arguments

			# plot problem setup
			fig = figure()
			hold(1)
			title("Test")
			ax = gca(projection="3d")
			plot3D(ox,oy,oz,".b")
			plot3D(ox1,oy1,oz1,".k")
			plot3D([sx],[sy],[sz],"xr")
			plot3D([gx],[gy],[gz],"xb")
			plot3D(rx,ry,rz,"--g")
			xlim([xmin, xmax])
			ylim([ymin, ymax])
			zlim([zmin, zmax])
			xlabel("X [m]")
			ylabel("Y [m]")
			zlabel("Z [m]")
			
			rx_smooth, ry_smooth, rz_smooth = smoothenPath(rx,ry,rz)
			
			
		end # end for-zz
	end	# end for-xx
	# println("*** horizonLengths: ", horizonLengths)
	# println("*** min horizon: ", minimum(horizonLengths[1:i]))
	# println("*** max horizon: ", maximum(horizonLengths[1:i]))

    println(PROGRAM_FILE," Done!!")
end

if length(PROGRAM_FILE)!=0 &&
    contains(@__FILE__, PROGRAM_FILE)

    main()
end


# if contains(@__FILE__, PROGRAM_FILE)
#     main()
# end


end #module

