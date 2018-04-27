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
# Reeds Shepp path planner
###############


module reeds_shepp

using PyPlot

const STEP_SIZE = 0.1

type Path
    lengths::Array{Float64} #lengths of each part of the path +: forward, -: backward
    ctypes::Array{String} # type of each part of the path
    L::Float64 # total path length
    x::Array{Float64} # final x positions [m]
    y::Array{Float64} # final y positions [m]
    yaw::Array{Float64} # final yaw angles [rad]
    directions::Array{Int8} # forward:1, backward:-1 
end

function pi_2_pi(iangle::Float64)::Float64
    while (iangle > pi)
        iangle -= 2.0 * pi
    end
    while (iangle < -pi)
        iangle += 2.0 * pi
    end

    return iangle
end


function calc_shortest_path(sx::Float64, sy::Float64, syaw::Float64, 
                            gx::Float64, gy::Float64, gyaw::Float64,
                            maxc::Float64;
                            step_size::Float64 = STEP_SIZE)
    # println("Find Shortest Path")
    paths = calc_paths(sx,sy,syaw,gx,gy,gyaw,maxc,step_size=step_size)

    minL = Inf
    best_path_index = -1
    for i in 1:length(paths)
        if paths[i].L <= minL 
            minL = paths[i].L
            best_path_index = i
        end
    end

    return paths[best_path_index]
end


function calc_shortest_path_length(sx::Float64, sy::Float64, syaw::Float64, 
                            gx::Float64, gy::Float64, gyaw::Float64,
                            maxc::Float64;
                            step_size::Float64 = STEP_SIZE)
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]
    paths = generate_path(q0, q1, maxc)

    minL = Inf
    for i in 1:length(paths)
        L = paths[i].L/maxc
        if L <= minL 
            minL = L
        end
    end

    return minL
end


function calc_paths(sx::Float64, sy::Float64, syaw::Float64, 
                    gx::Float64, gy::Float64, gyaw::Float64,
                    maxc::Float64; step_size::Float64 = STEP_SIZE)::Array{Path}
    q0 = [sx, sy, syaw]
    q1 = [gx, gy, gyaw]

    paths = generate_path(q0, q1, maxc)
    for path in paths
        x, y, yaw, directions = generate_local_course(path.L, path.lengths, path.ctypes, maxc, step_size*maxc)

        # convert global coordinate
        path.x = [cos(-q0[3]) * ix + sin(-q0[3]) * iy + q0[1] for (ix, iy) in zip(x, y)]
        path.y = [-sin(-q0[3]) * ix + cos(-q0[3]) * iy + q0[2] for (ix, iy) in zip(x, y)]
        path.yaw = pi_2_pi.([iyaw + q0[3] for iyaw in yaw])
        path.directions = directions
        path.lengths = [l/maxc for l in path.lengths]
        path.L = path.L/maxc

    end

    return paths
end


function get_label(path::Path)
    label =""

    for (m,l) in zip(path.ctypes, path.lengths)
        label = string(label, m)
        if l > 0.0
            label = string(label, "+")
        else
            label = string(label, "-")
        end
    end

    return label
end


function polar(x::Float64, y::Float64)
    r = sqrt(x^2+y^2)
    theta = atan2(y, x)
    return r, theta
end


function mod2pi(x::Float64)
    v = mod(x, 2.0*pi)
    if v < -pi
        v += 2.0*pi;
    else
        if v > pi
            v -= 2.0*pi
        end
    end
    return v
end
 

function LSL(x::Float64, y::Float64, phi::Float64)
    u, t = polar(x - sin(phi), y - 1.0 + cos(phi))
    if t >= 0.0
        v = mod2pi(phi - t)
        if (v >= 0.0)
            return true, t, u, v
       end
    end

    return false, 0.0, 0.0, 0.0
end


function LSR(x::Float64, y::Float64, phi::Float64)
    u1, t1 = polar(x + sin(phi), y - 1.0 - cos(phi))
    u1 = u1^2;
    if u1 >= 4.0
        u = sqrt(u1 - 4.0)
        theta = atan2(2.0, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)

        if t >= 0.0 && v >= 0.0
            return true, t, u, v
        end
    end

    return false, 0.0, 0.0, 0.0
end


function LRL(x::Float64, y::Float64, phi::Float64)
    u1, t1 = polar(x - sin(phi), y - 1.0 + cos(phi))

    if u1 <= 4.0
        u = -2.0*asin(0.25 * u1)
        t = mod2pi(t1 + 0.5 * u + pi);
        v = mod2pi(phi - t + u);

        if t >= 0.0 && u <= 0.0
            return true, t, u, v
        end
    end

    return false, 0.0, 0.0, 0.0
end
 

function set_path(paths::Array{Path}, lengths::Array{Float64}, ctypes::Array{String})

    path = Path([],[],0.0,[],[],[],[])
    path.ctypes = ctypes
    path.lengths = lengths

    # check same path exist
    for tpath in paths
        typeissame = (tpath.ctypes == path.ctypes)
        if typeissame
            if sum(tpath.lengths - path.lengths) <= 0.01
                return paths # not insert path
            end
        end
    end

    path.L = sum([abs(i) for i in lengths])

    Base.Test.@test path.L >= 0.01

    push!(paths, path)

    return paths
end


function SCS(x::Float64, y::Float64, phi::Float64, paths::Array{Path})::Array{Path}
    flag, t, u, v = SLS(x, y, phi) 
    if flag
        # println("SCS1")
        paths = set_path(paths, [t, u, v], ["S","L","S"])
    end
    flag, t, u, v = SLS(x, -y, -phi) 
    if flag
        # println("SCS2")
        paths = set_path(paths, [t, u, v], ["S","R","S"])
    end

    return paths
end


function SLS(x::Float64, y::Float64, phi::Float64)
    # println(x,",", y,",", phi, ",", mod2pi(phi))
    phi = mod2pi(phi)
    if y > 0.0 && phi > 0.0 && phi < pi*0.99
        xd = - y/tan(phi) + x
        t =  xd - tan(phi/2.0)
        u = phi
        v = sqrt((x-xd)^2+y^2)-tan(phi/2.0)
        # println("1,",t,",",u,",",v)
        return true, t, u, v
    elseif y < 0.0 && phi > 0.0 && phi < pi*0.99
        xd = - y/tan(phi) + x
        t =  xd - tan(phi/2.0)
        u = phi
        v = -sqrt((x-xd)^2+y^2)-tan(phi/2.0)
        # println("2,",t,",",u,",",v)
        return true, t, u, v
    end

    return false, 0.0, 0.0, 0.0
end


function CSC(x::Float64, y::Float64, phi::Float64, paths::Array{Path})
    flag, t, u, v = LSL(x, y, phi) 
    if flag
        # println("CSC1")
        paths = set_path(paths, [t, u, v], ["L","S","L"])
    end
    flag, t, u, v = LSL(-x, y, -phi) 
    if flag
        # println("CSC2")
        paths = set_path(paths, [-t, -u, -v], ["L","S","L"])
    end
    flag, t, u, v = LSL(x, -y, -phi) 
    if flag
        # println("CSC3")
        paths = set_path(paths, [t, u, v], ["R","S","R"])
    end
    flag, t, u, v = LSL(-x, -y, phi) 
    if flag
        # println("CSC4")
        paths = set_path(paths, [-t, -u, -v], ["R","S","R"])
    end
    flag, t, u, v = LSR(x, y, phi) 
    if flag
        # println("CSC5")
        paths = set_path(paths, [t, u, v], ["L","S","R"])
    end
    flag, t, u, v = LSR(-x, y, -phi) 
    if flag
        # println("CSC6")
        paths = set_path(paths, [-t, -u, -v], ["L","S","R"])
    end
    flag, t, u, v = LSR(x, -y, -phi) 
    if flag
        # println("CSC7")
        paths = set_path(paths, [t, u, v], ["R","S","L"])
    end
    flag, t, u, v = LSR(-x, -y, phi) 
    if flag
        # println("CSC8")
        paths = set_path(paths, [-t, -u, -v], ["R","S","L"])
    end

    return paths
end


function CCC(x::Float64, y::Float64, phi::Float64, paths::Array{Path})

    flag, t, u, v = LRL(x, y, phi) 
    if flag
        # println("CCC1")
        paths = set_path(paths, [t, u, v], ["L","R","L"])
    end
    flag, t, u, v = LRL(-x, y, -phi) 
    if flag
        # println("CCC2")
        paths = set_path(paths, [-t, -u, -v], ["L","R","L"])
    end
    flag, t, u, v = LRL(x, -y, -phi) 
    if flag
        # println("CCC3")
        paths = set_path(paths, [t, u, v], ["R","L","R"])
    end
    flag, t, u, v = LRL(-x, -y, phi) 
    if flag
        # println("CCC4")
        paths = set_path(paths, [-t, -u, -v], ["R","L","R"])
    end

    # backwards
    xb = x*cos(phi) + y*sin(phi)
    yb = x*sin(phi) - y*cos(phi)
    # println(xb, ",", yb,",",x,",",y)

    flag, t, u, v = LRL(xb, yb, phi)
    if flag
        # println("CCC5")
        paths = set_path(paths, [v, u, t], ["L","R","L"])
    end
    flag, t, u, v = LRL(-xb, yb, -phi)
    if flag
        # println("CCC6")
        paths = set_path(paths, [-v, -u, -t], ["L","R","L"])
    end
    flag, t, u, v = LRL(xb, -yb, -phi)
    if flag
        # println("CCC7")
        paths = set_path(paths, [v, u, t], ["R","L","R"])
    end
    flag, t, u, v = LRL(-xb, -yb, phi)
    if flag
        # println("CCC8")
        paths = set_path(paths, [-v, -u, -t], ["R","L","R"])
    end
 
    return paths
end


function calc_tauOmega(u::Float64, v::Float64, xi::Float64, eta::Float64, phi::Float64)
    delta = mod2pi(u-v)
    A = sin(u) - sin(delta)
    B = cos(u) - cos(delta) - 1.0

    t1 = atan2(eta*A - xi*B, xi*A + eta*B)
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    if t2 < 0
        tau = mod2pi(t1+pi)
    else
        tau = mod2pi(t1)
    end
    omega = mod2pi(tau - u + v - phi)

    return tau, omega
end


function LRLRn(x::Float64, y::Float64, phi::Float64)
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho = 0.25 * (2.0 + sqrt(xi*xi + eta*eta))

    if rho <= 1.0
        u = acos(rho)
        t, v = calc_tauOmega(u, -u, xi, eta, phi);
        if t >= 0.0 && v <= 0.0
            return true, t, u, v
        end
    end

    return false, 0.0, 0.0, 0.0
end


function LRLRp(x::Float64, y::Float64, phi::Float64)
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho = (20.0 - xi*xi - eta*eta) / 16.0;
    # println(xi,",",eta,",",rho)

    if (rho>=0.0 && rho<=1.0)
        u = -acos(rho);
        if (u >= -0.5 * pi)
            t, v = calc_tauOmega(u, u, xi, eta, phi);
            if t >= 0.0 && v >= 0.0
                return true, t, u, v
            end
        end
    end

    return false, 0.0, 0.0, 0.0
end


function CCCC(x::Float64, y::Float64, phi::Float64, paths::Array{Path})

    flag, t, u, v = LRLRn(x, y, phi) 
    if flag
        # println("CCCC1")
        paths = set_path(paths, [t, u, -u, v], ["L","R","L","R"])
    end

    flag, t, u, v = LRLRn(-x, y, -phi) 
    if flag
        # println("CCCC2")
        paths = set_path(paths, [-t, -u, u, -v], ["L","R","L","R"])
    end
 
    flag, t, u, v = LRLRn(x, -y, -phi) 
    if flag
        # println("CCCC3")
        paths = set_path(paths, [t, u, -u, v], ["R","L","R","L"])
    end
 
    flag, t, u, v = LRLRn(-x, -y, phi) 
    if flag
        # println("CCCC4")
        paths = set_path(paths, [-t, -u, u, -v], ["R","L","R","L"])
    end

    flag, t, u, v = LRLRp(x, y, phi) 
    if flag
        # println("CCCC5")
        paths = set_path(paths, [t, u, u, v], ["L","R","L","R"])
    end

    flag, t, u, v = LRLRp(-x, y, -phi) 
    if flag
        # println("CCCC6")
        paths = set_path(paths, [-t, -u, -u, -v], ["L","R","L","R"])
    end

    flag, t, u, v = LRLRp(x, -y, -phi) 
    if flag
        # println("CCCC7")
        paths = set_path(paths, [t, u, u, v], ["R","L","R","L"])
    end

    flag, t, u, v = LRLRp(-x, -y, phi) 
    if flag
        # println("CCCC8")
        paths = set_path(paths, [-t, -u, -u, -v], ["R","L","R","L"])
    end

    return paths
end


function LRSR(x::Float64, y::Float64, phi::Float64)

    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho, theta = polar(-eta, xi)

    if rho >= 2.0
        t = theta
        u = 2.0 - rho
        v = mod2pi(t + 0.5*pi - phi)
        if t >= 0.0 && u <= 0.0 && v <=0.0
            return true, t, u, v
        end
    end

    return false, 0.0, 0.0, 0.0
end


function LRSL(x::Float64, y::Float64, phi::Float64)
    xi = x - sin(phi)
    eta = y - 1.0 + cos(phi)
    rho, theta = polar(xi, eta)

    if rho >= 2.0
        r = sqrt(rho*rho - 4.0);
        u = 2.0 - r;
        t = mod2pi(theta + atan2(r, -2.0));
        v = mod2pi(phi - 0.5*pi - t);
        if t >= 0.0 && u<=0.0 && v<=0.0
            return true, t, u, v
        end
    end

    return false, 0.0, 0.0, 0.0
end


function CCSC(x::Float64, y::Float64, phi::Float64, paths::Array{Path})

    flag, t, u, v = LRSL(x, y, phi) 
    if flag
        # println("CCSC1")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","L"])
    end

    flag, t, u, v = LRSL(-x, y, -phi) 
    if flag
        # println("CCSC2")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","L"])
    end

    flag, t, u, v = LRSL(x, -y, -phi) 
    if flag
        # println("CCSC3")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","R"])
    end

    flag, t, u, v = LRSL(-x, -y, phi) 
    if flag
        # println("CCSC4")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","R"])
    end

    flag, t, u, v = LRSR(x, y, phi) 
    if flag
        # println("CCSC5")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["L","R","S","R"])
    end

    flag, t, u, v = LRSR(-x, y, -phi) 
    if flag
        # println("CCSC6")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["L","R","S","R"])
    end

    flag, t, u, v = LRSR(x, -y, -phi) 
    if flag
        # println("CCSC7")
        paths = set_path(paths, [t, -0.5*pi, u, v], ["R","L","S","L"])
    end

    flag, t, u, v = LRSR(-x, -y, phi) 
    if flag
        # println("CCSC8")
        paths = set_path(paths, [-t, 0.5*pi, -u, -v], ["R","L","S","L"])
    end

    # backwards
    xb = x*cos(phi) + y*sin(phi)
    yb = x*sin(phi) - y*cos(phi)
    flag, t, u, v = LRSL(xb, yb, phi) 
    if flag
        # println("CCSC9")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","R","L"])
    end

    flag, t, u, v = LRSL(-xb, yb, -phi) 
    if flag
        # println("CCSC10")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","R","L"])
    end

    flag, t, u, v = LRSL(xb, -yb, -phi) 
    if flag
        # println("CCSC11")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","L","R"])
    end

    flag, t, u, v = LRSL(-xb, -yb, phi) 
    if flag
        # println("CCSC12")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","L","R"])
    end

    flag, t, u, v = LRSR(xb, yb, phi) 
    if flag
        # println("CCSC13")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["R","S","R","L"])
    end

    flag, t, u, v = LRSR(-xb, yb, -phi) 
    if flag
        # println("CCSC14")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["R","S","R","L"])
    end

    flag, t, u, v = LRSR(xb, -yb, -phi) 
    if flag
        # println("CCSC15")
        paths = set_path(paths, [v, u, -0.5*pi, t], ["L","S","L","R"])
    end

    flag, t, u, v = LRSR(-xb, -yb, phi) 
    if flag
        # println("CCSC16")
        paths = set_path(paths, [-v, -u, 0.5*pi, -t], ["L","S","L","R"])
    end

    return paths
end


function LRSLR(x::Float64, y::Float64, phi::Float64)
    # formula 8.11 *** TYPO IN PAPER ***
    xi = x + sin(phi)
    eta = y - 1.0 - cos(phi)
    rho, theta = polar(xi, eta)
    if rho >= 2.0
        u = 4.0 - sqrt(rho*rho - 4.0)
        if u <= 0.0
            t = mod2pi(atan2((4.0-u)*xi -2.0*eta, -2.0*xi + (u-4.0)*eta));
            v = mod2pi(t - phi);

            if t >= 0.0 && v >=0.0
                return true, t, u, v
            end
        end
    end

    return false, 0.0, 0.0, 0.0
end
  

function CCSCC(x::Float64, y::Float64, phi::Float64, paths::Array{Path})
    flag, t, u, v = LRSLR(x, y, phi) 
    if flag
        # println("CCSCC1")
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["L","R","S","L","R"])
    end
    flag, t, u, v = LRSLR(-x, y, -phi) 
    if flag
        # println("CCSCC2")
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["L","R","S","L","R"])
    end

    flag, t, u, v = LRSLR(x, -y, -phi) 
    if flag
        # println("CCSCC3")
        paths = set_path(paths, [t, -0.5*pi, u, -0.5*pi, v], ["R","L","S","R","L"])
    end

    flag, t, u, v = LRSLR(-x, -y, phi) 
    if flag
        # println("CCSCC4")
        paths = set_path(paths, [-t, 0.5*pi, -u, 0.5*pi, -v], ["R","L","S","R","L"])
    end

    return paths
end


function generate_local_course(L::Float64,
                               lengths::Array{Float64},
                               mode::Array{String},
                               maxc::Float64,
                               step_size::Float64)
    npoint = trunc(Int64, L/step_size) + length(lengths)+3
    # println(npoint, ",", L, ",", step_size, ",", L/step_size)

    px = fill(0.0, npoint)
    py = fill(0.0, npoint)
    pyaw = fill(0.0, npoint)
    directions = fill(0, npoint)
    ind = 2

    if lengths[1] > 0.0
        directions[1] = 1
    else
        directions[1] = -1
    end

    if lengths[1] > 0.0
        d = step_size
    else
        d = -step_size
    end

    pd = d
    ll = 0.0

    for (m, l, i) in zip(mode, lengths, 1:length(mode))

        if l > 0.0
            d = step_size
        else
            d = -step_size
        end

        # set prigin state
        ox, oy, oyaw = px[ind], py[ind], pyaw[ind]

        ind -= 1
        if i >= 2 && (lengths[i-1]*lengths[i])>0
            pd = - d - ll
        else
            pd = d - ll
        end

        while abs(pd) <= abs(l)
            ind += 1
            px, py, pyaw, directions = interpolate(ind, pd, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
            pd += d
        end

        ll = l - pd - d # calc remain length

        ind += 1
        px, py, pyaw, directions = interpolate(ind, l, m, maxc, ox, oy, oyaw, px, py, pyaw, directions)
    end

    #remove unused data
    while px[end] == 0.0
        pop!(px)
        pop!(py)
        pop!(pyaw)
        pop!(directions)
    end

    return px, py, pyaw, directions
end


function interpolate(ind::Int64, l::Float64, m::String, maxc::Float64,
                     ox::Float64, oy::Float64, oyaw::Float64,
                     px::Array{Float64}, py::Array{Float64}, pyaw::Array{Float64}, 
                     directions::Array{Int64})

    if m == "S"
        px[ind] = ox + l / maxc * cos(oyaw)
        py[ind] = oy + l / maxc * sin(oyaw)
        pyaw[ind] =  oyaw
    else # curve
        ldx = sin(l) / maxc
        if m == "L"  # left turn
            ldy = (1.0 - cos(l)) / maxc
        elseif m == "R"  # right turn
            ldy = (1.0 - cos(l)) / -maxc 
        end
        gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy 
        gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy
        px[ind] = ox + gdx
        py[ind] = oy + gdy
    end

    if m == "L"  # left turn
        pyaw[ind] = oyaw + l
    elseif m == "R"  # right turn
        pyaw[ind] = oyaw - l
    end

    if l > 0.0
        directions[ind] = 1
    else
        directions[ind] = -1
    end

    return px, py, pyaw, directions
end


function generate_path(q0::Array{Float64}, q1::Array{Float64}, maxc::Float64)::Array{Path}
    dx = q1[1] - q0[1]
    dy = q1[2] - q0[2]
    dth = q1[3] - q0[3]
    c = cos(q0[3])
    s = sin(q0[3]);
    x = (c*dx + s*dy)*maxc
    y = (-s*dx + c*dy)*maxc

    paths = Path[]
    paths = SCS(x, y, dth, paths)
    paths = CSC(x, y, dth, paths)
    paths = CCC(x, y, dth, paths)
    paths = CCCC(x, y, dth, paths)
    paths = CCSC(x, y, dth, paths)
    paths = CCSCC(x, y, dth, paths)

    return paths
end


function calc_curvature(x,y,yaw, directions)

    c = Float64[]
    ds = Float64[]

    for i in 2:length(x)-1
        dxn = x[i]-x[i-1]
        dxp = x[i+1]-x[i]
        dyn = y[i]-y[i-1]
        dyp = y[i+1]-y[i]
        dn =sqrt(dxn^2.0+dyn^2.0)
        dp =sqrt(dxp^2.0+dyp^2.0)
        dx = 1.0/(dn+dp)*(dp/dn*dxn+dn/dp*dxp)
        ddx = 2.0/(dn+dp)*(dxp/dp-dxn/dn)
        dy = 1.0/(dn+dp)*(dp/dn*dyn+dn/dp*dyp)
        ddy = 2.0/(dn+dp)*(dyp/dp-dyn/dn)
        curvature = (ddy*dx-ddx*dy)/(dx^2+dy^2)
        d = (dn+dp)/2.0

        if isnan(curvature)
            curvature = 0.0
        end

        if directions[i] <= 0.0
            curvature = -curvature
        end

        if length(c) == 0
            push!(ds, d)
            push!(c, curvature)
        end

        push!(ds, d)
        push!(c, curvature)
    end

    push!(ds, ds[end])
    push!(c, c[end] )

    return c, ds
end


function check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)
    # println("Test")
    # println(start_x,",", start_y, "," ,start_yaw, ",", max_curvature)
    paths = calc_paths(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    Base.Test.@test length(paths) >= 1

    for path in paths
        Base.Test.@test abs(path.x[1] - start_x) <= 0.01
        Base.Test.@test abs(path.y[1] - start_y) <= 0.01
        Base.Test.@test abs(path.yaw[1] - start_yaw) <= 0.01
        Base.Test.@test abs(path.x[end] - end_x) <= 0.01
        Base.Test.@test abs(path.y[end] - end_y) <= 0.01
        Base.Test.@test abs(path.yaw[end] - end_yaw) <= 0.01

        #course distance check
        d = [sqrt(dx^2+dy^2) for (dx, dy) in zip(diff(path.x[1:end-1]), diff(path.y[1:end-1]))] 

        for i in length(d)
            Base.Test.@test abs(d[i] - STEP_SIZE) <= 0.001
        end
    end

end

function test()
    println("Test1")
    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    start_yaw = deg2rad(10.0)  # [rad]
    end_x = 7.0  # [m]
    end_y = -8.0  # [m]
    end_yaw = deg2rad(50.0)  # [rad]
    max_curvature = 2.0

    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    start_x = 0.0  # [m]
    start_y = 0.0  # [m]
    start_yaw = deg2rad(10.0)  # [rad]
    end_x = 7.0  # [m]
    end_y = -8.0  # [m]
    end_yaw = deg2rad(-50.0)  # [rad]
    max_curvature = 2.0

    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    start_x = 0.0  # [m]
    start_y = 10.0  # [m]
    start_yaw = deg2rad(-10.0)  # [rad]
    end_x = -7.0  # [m]
    end_y = -8.0  # [m]
    end_yaw = deg2rad(-50.0)  # [rad]
    max_curvature = 2.0

    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    start_x = 0.0  # [m]
    start_y = 10.0  # [m]
    start_yaw = deg2rad(-10.0)  # [rad]
    end_x = -7.0  # [m]
    end_y = -8.0  # [m]
    end_yaw = deg2rad(150.0)  # [rad]
    max_curvature = 1.0

    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    start_x = 0.0  # [m]
    start_y = 10.0  # [m]
    start_yaw = deg2rad(-10.0)  # [rad]
    end_x = 7.0  # [m]
    end_y = 8.0  # [m]
    end_yaw = deg2rad(150.0)  # [rad]
    max_curvature = 2.0
    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    start_x = -40.0  # [m]
    start_y = 549.0  # [m]
    start_yaw = 2.44346 # [rad]
    end_x =  36.0 # [m]
    end_y =  446.0 # [m]
    end_yaw = -0.698132
    max_curvature = 0.05890904077226434
    check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    # Random test
    for i in 1:100
        start_x = rand()*100.0 - 50.0
        start_y = rand()*100.0 - 50.0
        start_yaw = deg2rad(rand()*360.0 - 180.0)
        end_x = rand()*100.0 - 50.0
        end_y = rand()*100.0 - 50.0
        end_yaw = deg2rad(rand()*360.0 - 180.0)
        max_curvature = rand()/10.0
        # println(i, ",", start_x, ",", start_y,",", start_yaw,",",end_x,",",end_y,",", end_yaw)
        check_path(start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)
    end
end


function main()
    println(PROGRAM_FILE," start!!")
    test()

    start_x = 3.0  # [m]
    start_y = 10.0  # [m]
    start_yaw = deg2rad(40.0)  # [rad]
    end_x = 0.0  # [m]
    end_y = 1.0  # [m]
    end_yaw = deg2rad(0.0)  # [rad]
    max_curvature = 0.1

    @time bpath = calc_shortest_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, max_curvature)

    rc, rds = calc_curvature(bpath.x, bpath.y, bpath.yaw, bpath.directions)

    subplots(1)
    plot(bpath.x, bpath.y,"-r", label=get_label(bpath))

    plot(start_x, start_y)
    plot(end_x, end_y)

    legend()
    grid(true)
    axis("equal")

    subplots(1)
    plot(rc, ".r", label="reeds shepp")
    grid(true)
    title("Curvature")

    show()

    println(PROGRAM_FILE," Done!!")
end


if length(PROGRAM_FILE)!=0 &&
    contains(@__FILE__, PROGRAM_FILE)
 
    main()
end

end #module

