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
# 	X. Zhang, A. Liniger and F. Borrelli; "Optimization-Based Collision Avoidance"; Technical Report, 2017
###############

###############
# Grid based A* shorest path planning
###############

module a_star

using PyPlot
using NearestNeighbors
using DataStructures 

const VEHICLE_RADIUS = 5.0 #[m]
const GRID_RESOLUTION = 1.0 #[m]


type Node
    x::Int64 #x index
    y::Int64 #y index
    cost::Float64 # cost
    pind::Int64 # parent index
end


function calc_dist_policy(gx::Float64, gy::Float64,
                     ox::Array{Float64}, oy::Array{Float64},
                     reso::Float64, vr::Float64)
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    vr: vehicle radius[m]
    """

    ngoal = Node(round(Int64, gx/reso),round(Int64, gy/reso),0.0, -1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)

    #open, closed set
    open, closed = Dict{Int64, Node}(), Dict{Int64, Node}()
    open[calc_index(ngoal, xw, minx, miny)] = ngoal

    motion = get_motion_model()
    nmotion = length(motion[:,1])
    pq = PriorityQueue()
    enqueue!(pq, calc_index(ngoal, xw, minx, miny), ngoal.cost)

    while true
        if length(open) == 0
			# println("Finish Search")
			break
		end

        c_id = dequeue!(pq)
        current = open[c_id]

        delete!(open, c_id)
        closed[c_id] = current

        for i in 1:nmotion # expand search grid based on motion model
            node = Node(current.x+motion[i,1], current.y+motion[i,2], current.cost+motion[i,3], c_id)

            if !verify_node(node, minx, miny, xw, yw, obmap)
                continue
            end

            node_ind = calc_index(node, xw, minx, miny)

            # If it is already in the closed set, skip it
            if haskey(closed,node_ind)  continue end

            if haskey(open, node_ind)
                if open[node_ind].cost > node.cost
                    # If so, update the node to have a new parent
                    open[node_ind].cost = node.cost
                    open[node_ind].pind = c_id
                end
            else # add to open set
                open[node_ind] = node
                enqueue!(pq, calc_index(node, xw, minx, miny), node.cost)
            end
        end
    end

    pmap = calc_policy_map(closed, xw, yw, minx, miny)

    return pmap
end


function calc_policy_map(closed, xw, yw, minx, miny)

    pmap = fill(Inf, (xw,yw))

    for n in values(closed)
        pmap[n.x-minx, n.y-miny] = n.cost
    end
    # println(pmap)

    return pmap
end


function calc_astar_path(sx::Float64, sy::Float64, gx::Float64, gy::Float64,
                         ox::Array{Float64}, oy::Array{Float64}, reso::Float64, vr::Float64)
    """
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    """

    nstart = Node(round(Int64,sx/reso),round(Int64, sy/reso),0.0, -1)
    ngoal = Node(round(Int64, gx/reso),round(Int64, gy/reso),0.0, -1)

    ox = [iox/reso for iox in ox]
    oy = [ioy/reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, vr)

    #open, closed set
    open, closed = Dict{Int64, Node}(), Dict{Int64, Node}()
    open[calc_index(nstart, xw, minx, miny)] = nstart

    motion = get_motion_model()
    nmotion = length(motion[:,1])
    pq = PriorityQueue()
    enqueue!(pq, calc_index(nstart, xw, minx, miny), calc_cost(nstart, ngoal))

    while true
        if length(open) == 0
			println("Error: No open set")
			break
		end

        c_id = dequeue!(pq)
        current = open[c_id]

        if current.x == ngoal.x && current.y == ngoal.y # check goal
            # println("Goal!!")
            closed[c_id] = current
            break
        end

        delete!(open, c_id)
        closed[c_id] = current

        for i in 1:nmotion # expand search grid based on motion model
            node = Node(current.x+motion[i,1], current.y+motion[i,2], current.cost+motion[i,3], c_id)

            if !verify_node(node, minx, miny, xw, yw, obmap)
                continue
            end

            node_ind = calc_index(node, xw, minx, miny)

            # If it is already in the closed set, skip it
            if haskey(closed,node_ind)  continue end

            if haskey(open, node_ind)
                if open[node_ind].cost > node.cost
                    # If so, update the node to have a new parent
                    open[node_ind].cost = node.cost
                    open[node_ind].pind = c_id
                end
            else # add to open set
                open[node_ind] = node
                enqueue!(pq, calc_index(node, xw, minx, miny), calc_cost(node, ngoal))
            end
        end
    end

    rx, ry = get_final_path(closed, ngoal, nstart, xw, minx, miny, reso)

    return rx, ry
end


function verify_node(node::Node, minx::Int64, miny::Int64, xw::Int64, yw::Int64, obmap::Array{Bool,2})

    if (node.x - minx) >= xw 
        return false
    elseif (node.x - minx) <= 0 
        return false
    end
    if (node.y - miny) >= yw
        return false
    elseif (node.y - miny) <= 0 
        return false
    end

    #collision check
    if obmap[node.x-minx, node.y-miny]
        return false
    end 

    return true
end


function calc_cost(n::Node, ngoal::Node)
    return (n.cost + h(n.x - ngoal.x, n.y - ngoal.y))
end


function get_motion_model()
    # dx, dy, cost
    motion=[1 0 1;
          0 1 1;
          -1 0 1;
          0 -1 1;
          -1 -1 sqrt(2);
          -1 1 sqrt(2);
          1 -1 sqrt(2);
          1 1 sqrt(2);]

    return motion
end


function calc_index(node::Node, xwidth::Int64, xmin::Int64, ymin::Int64)
    return (node.y - ymin)*xwidth + (node.x - xmin)
end


function calc_obstacle_map(ox::Array{Float64}, oy::Array{Float64}, reso::Float64, vr::Float64)

    minx = round(Int64, minimum(ox))
    miny = round(Int64, minimum(oy))
    maxx = round(Int64, maximum(ox))
    maxy = round(Int64, maximum(oy))

    xwidth = round(Int64, maxx - minx)
    ywidth = round(Int64, maxy - miny)

    obmap = fill(false, (xwidth,ywidth))

    kdtree = KDTree(hcat(ox, oy)')
    for ix in 1:xwidth 
        x = ix + minx
        for iy in 1:ywidth 
            y = iy + miny
            idxs, onedist = knn(kdtree, [x, y] , 1)
            if onedist[1] <= vr/reso 
                obmap[ix,iy] = true
            end
        end
    end

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth
end


function get_final_path(closed::Dict{Int64, Node},
                        ngoal::Node,
                        nstart::Node,
                        xw::Int64,
                        minx::Int64,
                        miny::Int64,
                        reso::Float64)

    rx, ry = [ngoal.x],[ngoal.y]
    nid = calc_index(ngoal, xw, minx, miny)
    while true
        n = closed[nid]
        push!(rx, n.x)
        push!(ry, n.y)
        nid = n.pind

        if rx[end] == nstart.x && ry[end] == nstart.y
            # println("done")
            break
        end
    end

    rx = reverse(rx) .* reso
    ry = reverse(ry) .* reso

    return rx, ry
end


function search_min_cost_node(open::Dict{Int64, Node}, ngoal::Node)
    mnode = nothing
    mcost = Inf
    for n in values(open)
        # println(n)
        cost = n.cost + h(n.x - ngoal.x, n.y - ngoal.y)
        if mcost > cost
            mnode = n 
            mcost = cost 
        end
    end
    # println("minnode:", mnode)

    return mnode
end


function h(x::Int64, y::Int64)
    """
    Heuristic cost function
    """
    return sqrt(x^2+y^2);
end


function main()
    # println(PROGRAM_FILE," start!!")

    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]

    ox = Float64[]
    oy = Float64[]

    for i in 0:60
        push!(ox, Float64(i))
        push!(oy, 0.0)
    end
    for i in 0:60
        push!(ox, 60.0)
        push!(oy, Float64(i))
    end
    for i in 0:60
        push!(ox, Float64(i))
        push!(oy, 60.0)
    end
    for i in 0:60
        push!(ox, 0.0)
        push!(oy, Float64(i))
    end
    for i in 0:40
        push!(ox, 20.0)
        push!(oy, Float64(i))
    end
    for i in 0:40
        push!(ox, 40.0)
        push!(oy, 60.0-Float64(i))
    end

    @time rx, ry = calc_astar_path(sx, sy, gx, gy, ox, oy, GRID_RESOLUTION, VEHICLE_RADIUS)

    plot(ox, oy, ".k",label="obstacles")
    plot(sx, sy, "xr",label="start")
    plot(gx, gy, "xb",label="goal")
    plot(rx, ry, "-r",label="A* path")
    legend()
    grid(true)
    axis("equal")
    show()

    # println(PROGRAM_FILE," Done!!")
end


if length(PROGRAM_FILE)!=0 &&
    contains(@__FILE__, PROGRAM_FILE)
 
    main()
end


end #module

