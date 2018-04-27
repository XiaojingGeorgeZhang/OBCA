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
#   X. Zhang, A. Liniger, A. Sakai and F. Borrelli; "Autonomous  Parking  using  Optimization-Based  Collision  Avoidance"; Technical Report, 2018 [add URL]
###############

###############
# Hybrid A star: Julia implementation of Hybrid A* algorithm
###############

module hybrid_a_star

using PyPlot
using DataFrames
using NearestNeighbors
using DataStructures 

include("./reeds_shepp.jl")
include("./a_star.jl")
include("./collision_check.jl")


const VEHICLE_RADIUS = 1.0 #[m]; radius of rear ball; 7.0 
const BUBBLE_DIST = 1.7 #[m]; distance to "forward bubble"; 7.0

##### Fast Comp Time values from Alex Liniger ######
const OB_MAP_RESOLUTION = 0.1 #[m]; obstacle resolution
const YAW_GRID_RESOLUTION = deg2rad(5.0) #[m]; 10.0 /// 5.0
const N_STEER = 5.0 # number of steer command; 10.0 seems OK /// 5
## For Backwards Parking
# const XY_GRID_RESOLUTION = 1. #[m];
# const MOTION_RESOLUTION = 0.3 #[m];
## For Parallel Parking
const XY_GRID_RESOLUTION = 0.3 #[m];
const MOTION_RESOLUTION = 0.1 #[m];
###################################################

const USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC = true
const USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC = false

const SB_COST = 10.0 # switch back penalty cost
const BACK_COST = 0.0 # backward penalty cost
const STEER_CHANGE_COST = 10.0 # steer angle change penalty cost
const STEER_COST = 0.0  # steer angle  penalty cost
const H_COST = 1. # Heuristic cost; higher -> heuristic; 1.0
 
const WB = 2.7 #[m]; 7.0
const MAX_STEER = 0.6#deg2rad(35.0) #[rad]

type Node
    xind::Int64 #x index
    yind::Int64 #y index
    yawind::Int64 #yaw index
    direction::Bool # moving direction forword:true, backword:false
    x::Array{Float64} # x position [m]
    y::Array{Float64} # y position [m]
    yaw::Array{Float64} # yaw angle [rad]
    steer::Float64 # steer input
    cost::Float64 # cost
    pind::Int64 # parent index
end

type Config
    minx::Int64
    miny::Int64
    minyaw::Int64
    maxx::Int64
    maxy::Int64
    maxyaw::Int64
    xw::Int64
    yw::Int64
    yaww::Int64
    xyreso::Float64
    yawreso::Float64
    obminx::Int64
    obminy::Int64
    obmaxx::Int64
    obmaxy::Int64
    obxw::Int64
    obyw::Int64
    obreso::Float64
end


function calc_hybrid_astar_path(sx::Float64, sy::Float64, syaw::Float64,
                                gx::Float64, gy::Float64, gyaw::Float64,
                                ox::Array{Float64}, oy::Array{Float64},
                                xyreso::Float64, yawreso::Float64,
                                obreso::Float64)
    """
    Calc hybrid astar path
    sx: start x position [m]
    sy: start y position [m]
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """

    syaw, gyaw = pi_2_pi(syaw), pi_2_pi(gyaw)

    const c = calc_config(ox, oy, xyreso, yawreso, obreso)
    kdtree = KDTree(hcat(ox, oy)')
    obmap, gkdtree = calc_obstacle_map(ox, oy, c)
    nstart = Node(round(Int64,sx/xyreso), round(Int64,sy/xyreso), round(Int64, syaw/yawreso),true,[sx],[sy],[syaw],0.0,0.0, -1)
    ngoal = Node(round(Int64,gx/xyreso), round(Int64,gy/xyreso), round(Int64,gyaw/yawreso),true,[gx],[gy],[gyaw],0.0,0.0, -1)

    if USE_HOLONOMIC_WITH_OBSTACLE_HEURISTIC
        h_dp = calc_holonomic_with_obstacle_heuristic(ngoal, ox, oy, xyreso)
    else
        h_dp = Array{Float64}()
    end
    if USE_NONHOLONOMIC_WITHOUT_OBSTACLE_HEURISTIC
        h_rs = calc_nonholonomic_without_obstacle_heuristic(ngoal, c)
    else
        h_rs = Array{Float64}()
    end

    open, closed = Dict{Int64, Node}(), Dict{Int64, Node}()
    open[calc_index(nstart, c)] = nstart
    pq = PriorityQueue()
    enqueue!(pq, calc_index(nstart, c), calc_cost(nstart, h_rs, h_dp, ngoal, c))

    u, d = calc_motion_inputs()
    nmotion = length(u)

    while true
        if length(open) == 0
            println("Error: Cannot find path, No open set")
            return nothing, nothing, nothing
        end

        c_id = dequeue!(pq)
        current = open[c_id]

        isupdated, current = update_node_with_analystic_expantion(current, ngoal, obmap, c, kdtree, ox, oy)
        if isupdated
            closed[calc_index(ngoal, c)] = current
            break #goal
        end

        #move current node from open to closed
        delete!(open, c_id)
        closed[c_id] = current

        for i in 1:nmotion
            node = calc_next_node(current, c_id, u[i], d[i], c, gkdtree)

            if !verify_index(node, obmap, c, kdtree, ox, oy) continue end

            node_ind = calc_index(node, c)

            # If it is already in the closed set, skip it
            if haskey(closed, node_ind)  continue end

            if !haskey(open, node_ind)
                open[node_ind] = node
                enqueue!(pq, calc_index(node, c), calc_cost(node, h_rs, h_dp, ngoal, c))
            end
        end

    end

    # println("final expand node:", length(open) + length(closed))

    rx, ry, ryaw = get_final_path(closed, ngoal, nstart, c)

    return rx, ry, ryaw
end


function update_node_with_analystic_expantion(current::Node,
                                             ngoal::Node,
                                             obmap::Array{Bool,2},
                                             c::Config,
                                             kdtree::NearestNeighbors.KDTree,
                                             ox::Array{Float64},
                                             oy::Array{Float64}
                                            )

    apath = analystic_expantion(current, ngoal, obmap, c, kdtree, ox, oy)
    if apath != nothing
        # println("Find path! with analystic_expantion")
        current.x = vcat(current.x, apath.x[2:end-1])
        current.y = vcat(current.y, apath.y[2:end-1])
        current.yaw = vcat(current.yaw, apath.yaw[2:end-1])
        current.cost += calc_rs_path_cost(apath)
        return true, current
    end

    return false, current #no update
end


function calc_rs_path_cost(rspath::hybrid_a_star.reeds_shepp.Path)

    cost = 0.0
    for l in rspath.lengths
        if l >= 0 # forward
            cost += l
        else # back
            cost += abs(l) * BACK_COST
        end
    end

    # swich back penalty
    for i in 1:length(rspath.lengths) - 1
        if rspath.lengths[i] * rspath.lengths[i+1] < 0.0 # switch back
            cost += SB_COST
        end
    end

    # steer penalyty
    for ctype in rspath.ctypes
        if ctype != "S" # curve
            cost += STEER_COST*abs(MAX_STEER)
        end
    end

    # ==steer change penalty
    # calc steer profile
    nctypes = length(rspath.ctypes)
    ulist = fill(0.0, nctypes)
    for i in 1:nctypes
        if rspath.ctypes[i] == "R" 
            ulist[i] = - MAX_STEER
        elseif rspath.ctypes[i] == "L"
            ulist[i] = MAX_STEER
        end
    end
 
    for i in 1:length(rspath.ctypes) - 1
        cost += STEER_CHANGE_COST*abs(ulist[i+1] - ulist[i])
    end

    # println("RS cost is ", cost)
    return cost
end


function analystic_expantion(n::Node, ngoal::Node, obmap::Array{Bool,2}, c::Config,
                             kdtree::NearestNeighbors.KDTree,
                             ox::Array{Float64},
                             oy::Array{Float64}
                            )

    sx = n.x[end]
    sy = n.y[end]
    syaw = n.yaw[end]

    max_curvature = tan(MAX_STEER)/WB
    path = reeds_shepp.calc_shortest_path(sx, sy, syaw, 
                                           ngoal.x[end], ngoal.y[end], ngoal.yaw[end],
                                           max_curvature, step_size=MOTION_RESOLUTION)

    if path == nothing
        return nothing
    end

    if !collision_check.check_collision(path.x, path.y, path.yaw, kdtree, ox, oy)
        return nothing
    end

    # println(paths)
    return path # find good path
end


function calc_motion_inputs()

    up = [i for i in MAX_STEER/N_STEER:MAX_STEER/N_STEER:MAX_STEER]
    u = vcat([0.0], [i for i in up], [-i for i in up]) 
    d = vcat([1.0 for i in 1:length(u)], [-1.0 for i in 1:length(u)]) 
    u = vcat(u,u)

    return u, d
end


function verify_index(node::Node, obmap::Array{Bool,2}, c::Config,
                      kdtree::NearestNeighbors.KDTree,
                      ox::Array{Float64},
                      oy::Array{Float64}
                     )::Bool

    # overflow map
    if (node.xind - c.minx) >= c.xw
        return false
    elseif (node.xind - c.minx) <= 0
        return false
    end
    if (node.yind - c.miny) >= c.yw
        return false
    elseif (node.yind - c.miny) <= 0
        return false
    end

    # check collisiton
    # rectangle check
    if !collision_check.check_collision(node.x, node.y,node.yaw, kdtree, ox, oy)
        return false
    end

    return true #index is ok"
end


function pi_2_pi(iangle::Float64)
    while (iangle > pi)
        iangle -= 2.0 * pi
    end
    while (iangle < -pi)
        iangle += 2.0 * pi
    end

    return iangle
end


function calc_next_node(current::Node, c_id::Int64,
                        u::Float64, d::Float64, 
                        c::Config,
                        gkdtree::NearestNeighbors.KDTree)


    arc_l = XY_GRID_RESOLUTION

    nlist = round(Int64, arc_l/MOTION_RESOLUTION)+1
    xlist = fill(0.0, nlist)
    ylist = fill(0.0, nlist)
    yawlist = fill(0.0, nlist)
    xlist[1] = current.x[end] + d * MOTION_RESOLUTION*cos(current.yaw[end])
    ylist[1] = current.y[end] + d * MOTION_RESOLUTION*sin(current.yaw[end])
    yawlist[1] = pi_2_pi(current.yaw[end] + d*MOTION_RESOLUTION/WB * tan(u))
 

    for i in 1:(nlist - 1)
        xlist[i+1] = xlist[i] + d * MOTION_RESOLUTION*cos(yawlist[i])
        ylist[i+1] = ylist[i] + d * MOTION_RESOLUTION*sin(yawlist[i])
        yawlist[i+1] = pi_2_pi(yawlist[i] + d*MOTION_RESOLUTION/WB * tan(u))
    end
 
    xind = round(Int64, xlist[end]/c.xyreso)
    yind = round(Int64, ylist[end]/c.xyreso)
    yawind = round(Int64, yawlist[end]/c.yawreso)

    addedcost = 0.0
    if d > 0
        direction = true
        addedcost += abs(arc_l)
    else
        direction = false
        addedcost += abs(arc_l) * BACK_COST
    end

    # swich back penalty
    if direction != current.direction # switch back penalty
        addedcost += SB_COST
    end

    # steer penalyty
    addedcost += STEER_COST*abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current.steer - u)

    cost = current.cost + addedcost 
    node = Node(xind, yind, yawind, direction, xlist, ylist, yawlist, u, cost, c_id)
    # println(node)

    return node
end


function is_same_grid(node1::Node,node2::Node)

    if node1.xind != node2.xind
        return false
    end
    if node1.yind != node2.yind
        return false
    end
    if node1.yawind != node2.yawind
        return false
    end

    return true

end


function calc_index(node::Node, c::Config)
    ind = (node.yawind - c.minyaw)*c.xw*c.yw+(node.yind - c.miny)*c.xw + (node.xind - c.minx)
    if ind <= 0
        println("Error(calc_index):", ind)
    end
    return ind
end


function calc_holonomic_with_obstacle_heuristic(gnode::Node, ox::Array{Float64}, oy::Array{Float64}, xyreso::Float64)
    # println("Calc distance policy")
    h_dp = a_star.calc_dist_policy(gnode.x[end], gnode.y[end], ox, oy, xyreso, VEHICLE_RADIUS)
    return h_dp
end


function calc_nonholonomic_without_obstacle_heuristic(ngoal::Node,
                                                      c::Config)

    h_rs = fill(0.0, (c.xw,c.yw,c.yaww))
    max_curvature = tan(MAX_STEER)/WB

    for ix in 1:c.xw
        for iy in 1:c.yw
            for iyaw in 1:c.yaww
                sx = (ix + c.minx)*c.xyreso
                sy = (iy + c.miny)*c.xyreso
                syaw = pi_2_pi((iyaw + c.minyaw)*c.yawreso)
                L = reeds_shepp.calc_shortest_path_length(sx, sy, syaw, 
                                                          ngoal.x[end], ngoal.y[end], ngoal.yaw[end],
                                                          max_curvature, step_size=MOTION_RESOLUTION)
                h_rs[ix, iy, iyaw] = L
            end
        end
    end

    # println(h_rs[:,:,1])

    return h_rs
end


function calc_config(ox::Array{Float64}, oy::Array{Float64}, xyreso::Float64, yawreso::Float64, obreso::Float64)
    minx = round(Int64, minimum(ox)/xyreso)
    miny = round(Int64, minimum(oy)/xyreso)
    maxx = round(Int64, maximum(ox)/xyreso)
    maxy = round(Int64, maximum(oy)/xyreso)
    obminx = round(Int64, minimum(ox)/obreso)
    obminy = round(Int64, minimum(oy)/obreso)
    obmaxx = round(Int64, maximum(ox)/obreso)
    obmaxy = round(Int64, maximum(oy)/obreso)

    xw = round(Int64,(maxx - minx))
    yw = round(Int64,(maxy - miny))
    obxw = round(Int64,(obmaxx - obminx))
    obyw = round(Int64,(obmaxy - obminy))

    minyaw = round(Int64, - pi/yawreso) - 1
    maxyaw = round(Int64, pi/yawreso)
    yaww = round(Int64,(maxyaw - minyaw))

    config = Config(minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww,
                    xyreso, yawreso, obminx, obminy, obmaxx, obmaxy, obxw, obyw, obreso)

    return config
end


function calc_obstacle_map(ox::Array{Float64},
                           oy::Array{Float64},
                           c::Config)

    ox = [iox/c.obreso for iox in ox]
    oy = [ioy/c.obreso for ioy in oy]

    obmap = fill(false, (c.obxw, c.obyw))

    gkdtree = KDTree(hcat(ox, oy)')
    for ix in 1:c.obxw 
        x = ix + c.obminx
        for iy in 1:c.obyw 
            y = iy + c.obminy
            idxs, onedist = knn(gkdtree, [x, y] , 1)
            if onedist[1] <= VEHICLE_RADIUS/c.obreso 
                obmap[ix,iy] = true
            end
        end
    end

    return obmap, gkdtree
end


function get_final_path(closed::Dict{Int64, Node},
                        ngoal::Node,
                        nstart::Node,
                        c::Config)

    rx, ry, ryaw = Array{Float64}(ngoal.x),Array{Float64}(ngoal.y),Array{Float64}(ngoal.yaw)
    nid = calc_index(ngoal, c)
    # println("Fianl cost is ", closed[nid].cost)
    while true
        n = closed[nid]
        rx = vcat(rx, reverse(n.x))
        ry = vcat(ry, reverse(n.y))
        ryaw = vcat(ryaw, reverse(n.yaw))
        nid = n.pind
        if is_same_grid(n, nstart)
            # println("done")
            break
        end
    end

    rx = reverse(rx)
    ry = reverse(ry)
    ryaw = reverse(ryaw)

    dist = sum([sqrt(idx^2+idy^2) for (idx,idy) in zip(diff(rx), diff(ry))])
    # println("Fianl path distance is ", dist)

    return rx, ry, ryaw
end


function calc_cost(n::Node, h_rs::Array{Float64}, h_dp::Array{Float64}, ngoal::Node, c::Config)

    if length(h_rs) > 1 && length(h_dp) > 1  # Both heuristic cost are activated
        c_h_dp = h_dp[n.xind - c.minx, n.yind - c.miny]
        c_h_rs = h_rs[n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw]
        return (n.cost + H_COST*max(c_h_dp, c_h_rs))
    elseif length(h_dp) > 1 # Distance policy heuristics is activated
        return (n.cost + H_COST*h_dp[n.xind - c.minx, n.yind - c.miny])
    elseif length(h_rs) > 1 # Reed Sheep path heuristics is activated
        return (n.cost + H_COST*h_rs[n.xind - c.minx, n.yind - c.miny, n.yawind - c.minyaw])
    end

    return (n.cost + H_COST*calc_euclid_dist(n.x[end] - ngoal.x[end],n.y[end] - ngoal.y[end], n.yaw[end] - ngoal.yaw[end]))
end


function calc_euclid_dist(x::Float64, y::Float64, yaw::Float64)
    """
    Heuristic cost function
    """
    if yaw >= pi
        yaw -= pi
    else yaw <= -pi
        yaw += pi
    end
    return sqrt(x^2+y^2+yaw^2)
end


function main()
    # println(PROGRAM_FILE," start!!")

    sx = 20.0  # [m]
    sy = 20.0  # [m]
    syaw = deg2rad(90.0)
    gx = 180.0  # [m]
    gy = 100.0  # [m]
    gyaw = deg2rad(-90.0)

    ox = Float64[]
    oy = Float64[]

    for i in 0:200
        push!(ox, Float64(i))
        push!(oy, 0.0)
    end
    for i in 0:120
        push!(ox, 200.0)
        push!(oy, Float64(i))
    end
    for i in 0:200
        push!(ox, Float64(i))
        push!(oy, 120.0)
    end
    for i in 0:120
        push!(ox, 0.0)
        push!(oy, Float64(i))
    end
    for i in 0:80
        push!(ox, 40.0)
        push!(oy, Float64(i))
    end
    for i in 0:80
        push!(ox, 80.0)
        push!(oy, 120.0-Float64(i))
    end
    for i in 0:40
        push!(ox, 120.0)
        push!(oy, 120.0-Float64(i))
        push!(ox, 120.0)
        push!(oy, Float64(i))
    end
    for i in 0:80
        push!(ox, 160.0)
        push!(oy, 120.0-Float64(i))
    end

    @time rx, ry, ryaw = calc_hybrid_astar_path(sx, sy, syaw, gx, gy, gyaw, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION, OB_MAP_RESOLUTION)

    plot(ox, oy, ".k",label="obstacles")
    if rx != nothing
        plot(rx, ry, "-r",label="Hybrid A* path")
    end
 
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

