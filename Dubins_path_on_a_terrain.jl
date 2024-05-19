using LinearAlgebra, Dubins, Rotations, StaticArrays

""" Constants"""
R = 1.0   # Minimum turning radius
#       X    Y    Z  theta    since the terrain has 3 coordinates, hence defined by X, Y, Z
#qi = [0.0, 0.0, 1.0, 0.0]
#qf = [2.0, 1.0, 1.0, 0.0]

"""
Check if points (initial and final locations) lie on z = max{0, x − y, 2x + 3y + 1}
"""

function check_point_on_plane(q)
    z = q[4]
    if z == max(0, q[1]-q[2], 2*q[1] + 3*q[2] + 1)
        if z == 0
            return 1                        # returns 1 if the point lies on plane z = 0
        elseif z == q[1]-q[2]
            return 2                        # returns 2 if the point lies on plane z = x - y
        elseif z == 2*q[1] + 3*q[2] + 1
            return 3                        # returns 3 if the point lies on plane z = 2x + 3y + 1
        end
    else
        return "None"
    end
end

"""
Compute the point after rotating the plane it lies on about its intersection with z = 0
"""
function rotate_planes(q)
    n0 = [0.0, 0.0, 1.0]
    n0 = n0/ norm(n0)
    
    plane_check = check_point_on_plane(q)
    if plane_check == 1
        return q

    elseif plane_check == 2
        n1 = [1.0, -1.0, -1.0]
        n1 = n1/ norm(n1)
        n_norm = cross(n1, n0)
        n_norm = n_norm/ norm(n_norm)                  
        theta = acos(dot(n1, n0))
        RM = RM = [cos(theta) + n_norm[1]^2 * (1 - cos(theta))    n_norm[1] * n_norm[2] * (1 - cos(theta)) - n_norm[3] * sin(theta)   n_norm[1] * n_norm[3] * (1 - cos(theta)) + n_norm[2] * sin(theta);
        n_norm[2] * n_norm[1] * (1 - cos(theta)) + n_norm[3] * sin(theta)   cos(theta) + n_norm[2]^2 * (1 - cos(theta))    n_norm[2] * n_norm[3] * (1 - cos(theta)) - n_norm[1] * sin(theta);
        n_norm[3] * n_norm[1] * (1 - cos(theta)) - n_norm[2] * sin(theta)   n_norm[3] * n_norm[2] * (1 - cos(theta)) + n_norm[1] * sin(theta)   cos(theta) + n_norm[3]^2 * (1 - cos(theta))] 
        q_t = [q[1], q[2], q[4]]  
        qt_r = RM * q_t                                                                             # Computing rotated point
        qr = [qt_r[1], qt_r[2], q[3]]
        return qr

    elseif plane_check == 3
        n1 = [2.0, 3.0, -1.0]                                                             # Normal for z = 2x + 3y + 1
        n1 = n1/ norm(n1)
        n_norm = cross(n1, n0)
        n_norm = n_norm/ norm(n_norm)                  
        theta = -acos(dot(n1, n0))
        RM = RM = [cos(theta) + n_norm[1]^2 * (1 - cos(theta))    n_norm[1] * n_norm[2] * (1 - cos(theta)) - n_norm[3] * sin(theta)   n_norm[1] * n_norm[3] * (1 - cos(theta)) + n_norm[2] * sin(theta);
        n_norm[2] * n_norm[1] * (1 - cos(theta)) + n_norm[3] * sin(theta)   cos(theta) + n_norm[2]^2 * (1 - cos(theta))    n_norm[2] * n_norm[3] * (1 - cos(theta)) - n_norm[1] * sin(theta);
        n_norm[3] * n_norm[1] * (1 - cos(theta)) - n_norm[2] * sin(theta)   n_norm[3] * n_norm[2] * (1 - cos(theta)) + n_norm[1] * sin(theta)   cos(theta) + n_norm[3]^2 * (1 - cos(theta))] 
        q_t = [q[1], q[2], q[4]]       
        qt_r = RM * q_t                                                                             # Computing rotated point
        qr = [qt_r[1], qt_r[2], q[3]]
        return qr

    else 
        return zeros(3)
    end
end

"""
Compute the Dubins shortest path and path length using rotated points
"""

function path_terrain(q_i, q_f, R)
    qi_r = rotate_planes(q_i)
    qf_r = rotate_planes(q_f)
    errcode, path = dubins_shortest_path(qi_r, qf_r, R)
    path_length = dubins_path_length(path)

    return (path_length, qi_r, qf_r)
end


"""
Main function to calculate path_length from qi = [0.0, 0.0, 1.0, 0.0] qf = [2.0, 1.0, 1.0, 0.0]
"""
function main()
    q1 = [1.0, 1.0, pi/2, 6.0]
    q2 = [10.0, 1.0, pi, 9.0]
    (path_length_1, q_initial_r, q_final_r) = path_terrain(q1, q2, R)
    println("Path length:", path_length_1)
    println("Initial config:", q_initial_r)
    println("Final config:", q_final_r)
end

main()
