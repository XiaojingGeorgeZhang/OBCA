###############
# H-OBCA: Hierarchical Optimization-based Collision Avoidance - a path planner for autonomous parking
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
#   X. Zhang, A. Liniger, A. Sakai and F. Borrelli; "Autonomous  Parking  using  Optimization-Based  Collision  Avoidance"; Technical Report, 2018 [add URL]
###############

module collision_check

using NearestNeighbors
using PyPlot

const B = 1.0 #[m] distance from rear to vehicle back end
const C = 3.7 #[m] distance from rear to vehicle front end
const I = 2.0 #[m] width of vehicle
const WBUBBLE_DIST = (B+C)/2.0-B #[m] distance from rear and the center of whole bubble
const WBUBBLE_R = (B+C)/2.0 #[m] whole bubble radius

const vrx = [C, C, -B, -B, C ]
const vry = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]

function check_collision(x, y, yaw, kdtree, ox, oy)

    for (ix, iy, iyaw) in zip(x, y, yaw)
        cx = ix + WBUBBLE_DIST*cos(iyaw)
        cy = iy + WBUBBLE_DIST*sin(iyaw)

        # Whole bubble check
        ids = inrange(kdtree, [cx, cy], WBUBBLE_R, true)
        if length(ids) == 0 continue end

        if !rect_check(ix, iy, iyaw, ox[ids], oy[ids])
            return false #collision
        end
    end

    return true #OK

end


function rect_check(ix, iy, iyaw, ox, oy)

    c = cos(-iyaw)
    s = sin(-iyaw)

    for (iox, ioy) in zip(ox, oy)
        tx = iox - ix
        ty = ioy - iy
        lx = (c*tx - s*ty)
        ly = (s*tx + c*ty)

        sumangle = 0.0
        for i in 1:length(vrx)-1
            x1 = vrx[i] - lx
            y1 = vry[i] - ly
            x2 = vrx[i+1] - lx
            y2 = vry[i+1] - ly
            d1 = hypot(x1,y1)
            d2 = hypot(x2,y2)
            theta1 = atan2(y1,x1)
            tty = (-sin(theta1)*x2 + cos(theta1)*y2)

            tmp = (x1*x2+y1*y2)/(d1*d2)
            if tmp >= 1.0 tmp = 1.0 end

            if tty >= 0.0
                sumangle += acos(tmp)
            else
                sumangle -= acos(tmp)
            end
        end

        if sumangle >= pi
            return false
        end
    end

    return true #OK
end


function main()

    ox = rand(3)*30.0 - 30.0/2.0
    oy = rand(3)*30.0 - 30.0/2.0

    kdtree = KDTree(hcat(ox, oy)')

    x = [10.0, 5.0]
    y = [10.0, 5.0]
    yaw = [deg2rad(10.0), deg2rad(0.0)]

    flag = check_collision(x, y, yaw, kdtree, ox, oy)
    if flag 
        # println("OK")
    else
        # println("Collision")
    end

    plot(ox, oy, ".r")
    grid(true)
    axis("equal")
    show()

end


if length(PROGRAM_FILE)!=0 &&
    contains(@__FILE__, PROGRAM_FILE)
 
    @time main()
end


end #module


