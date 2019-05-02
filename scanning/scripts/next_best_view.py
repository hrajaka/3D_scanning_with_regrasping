import matplotlib.pyplot as plt
import numpy as np
import autolab_core
import trimesh
from mpl_toolkits.mplot3d import Axes3D
import scipy.spatial
import matplotlib.path

from kin_func_skeleton import rotation_3d


####


if __name__ == '__main__':

    ## Defining the transform between the box and the camera ##

    rotation = np.array([[1,       0,       0], # 'random' values, they will need to be changed for the final setup
                         [0, -0.1979,  0.9802],
                         [0, -0.9802, -0.1979]])
    translation = np.array([0.2, -0.8, 0.4])
    rigid_transfo = autolab_core.RigidTransform(rotation=rotation, translation=translation, from_frame='camera', to_frame='world')

    pawn = trimesh.exchange.load.load('pawn.obj')
    pawn.fix_normals()

    xs = []
    ys = []
    zs = []

    ## Plot the object in 3D ##

    for vertex in pawn.vertices:
        pos_in_world = np.matmul(rigid_transfo.matrix, np.array([vertex[0], vertex[1], vertex[2], 1]))

        xs.append(pos_in_world[0])
        ys.append(pos_in_world[1])
        zs.append(pos_in_world[2])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.scatter(xs, ys, zs, c='r', marker='o', s=0.1)

    camera_center = np.matmul(rigid_transfo.matrix, np.array([0, 0, 0, 1]))[:3]
    # ax.scatter([camera_center[0]], [camera_center[1]], [camera_center[2]])
    # ax.plot3D([camera_center[0], xs[0]], [camera_center[1], ys[0]], [camera_center[2], zs[0]])


    ## Draw the edges of the box ##

    box_size_x = 20 # WARNING: in centimeter
    box_size_y = 20
    box_size_z = 20
    box = -1 * np.ones((box_size_x, box_size_y, box_size_z)) # one tile per centimeter
    box_corners = [[0,              0,              0             ],
                   [box_size_x/100., 0,              0             ],
                   [0,              box_size_y/100., 0             ],
                   [0,              0,              box_size_z/100.],
                   [box_size_x/100., box_size_y/100., 0             ],
                   [box_size_x/100., 0,              box_size_z/100.],
                   [0,              box_size_y/100., box_size_z/100.],
                   [box_size_x/100., box_size_y/100., box_size_z/100.]]

    ax.plot3D([0,               0,               0,               0, 0, box_size_x/100., box_size_x/100., box_size_x/100., box_size_x/100., box_size_x/100., box_size_x/100.,               0,               0, box_size_x/100., box_size_x/100.,               0],
              [0,               0, box_size_y/100., box_size_y/100., 0,               0,               0, box_size_y/100., box_size_y/100.,               0, box_size_y/100., box_size_y/100., box_size_y/100., box_size_y/100.,               0,               0],
              [0, box_size_z/100., box_size_z/100.,               0, 0,               0, box_size_z/100., box_size_z/100.,               0,               0,               0,               0, box_size_z/100., box_size_z/100., box_size_z/100., box_size_z/100.])


    ## Taking care of the points that are on the mesh ##
    print("## Updating the occupancy grid for the points on the mesh ##")

    list_of_unit_vectors = []

    for i, vertex in enumerate(pawn.vertices):
        pos_in_world = np.array([xs[i], ys[i], zs[i]])
        # points_for_convex_hull.append(pos_in_world)

        vector = (pos_in_world - camera_center)
        list_of_unit_vectors.append(vector/np.linalg.norm(vector))

        # we do N steps in the direction of the line until we hit the object, and say the space is free
        for step_size in np.linspace(0, 1, 200):
            current_point = camera_center + step_size*vector
            if current_point[0] >= 0 and current_point[0] <= float(box_size_x)/100 and current_point[1] >= 0 and current_point[1] <= float(box_size_y)/100 and current_point[2] >= 0 and current_point[2] <= float(box_size_z)/100:
                id_x = int(current_point[0]*100)
                id_y = int(current_point[1]*100)
                id_z = int(current_point[2]*100)

                if box[id_x, id_y, id_z] == -1:
                    box[id_x-1:id_x+2, id_y-1:id_y+2, id_z-1:id_z+2] = 0

        # we say the space where the object is is not free
        id_x = int(pos_in_world[0]*100)
        id_y = int(pos_in_world[1]*100)
        id_z = int(pos_in_world[2]*100)

        if box[id_x, id_y, id_z] == -1 or box[id_x, id_y, id_z] == 0:
            box[id_x, id_y, id_z] = 1


    ## Now we loop on all points of the box to take care of the ones not aligned with the mesh ##

    print("## Updating the occupancy grid for the points that are not aligned with the mesh ##")

    for k in range(box_size_x):
        for i in range(box_size_y):
            for j in range(box_size_z):
                if box[k, i, j] == -1:
                    vector = np.array([float(k)/100, float(i)/100, float(j)/100] - camera_center)
                    unit_vector = vector / np.linalg.norm(vector)

                    dot_products = []
                    for vec in list_of_unit_vectors:
                        dot_products.append(np.dot(unit_vector, vec))

                    if np.max(dot_products) < 0.9999:
                        # then it means the point is not aligned with the part
                        box[k, i, j] = 0
                    else:
                        index_of_best = np.argmax(dot_products)
                        if np.linalg.norm(vector) < np.linalg.norm(np.array([xs[index_of_best], ys[index_of_best], zs[index_of_best]]) - camera_center):
                            # ie if the point is between the camera and the object
                            box[k, i, j] = 0

    ## Plotting the occupancy grid ##

    print("## Plotting the occupancy grid ##")
    for k in range(box_size_x):
        for i in range(box_size_y):
            for j in range(box_size_z):
                if box[k, i, j] == 1:
                    ax.scatter(k/100., i/100., j/100., c='k', marker='o', s=0.5)
                if box[k, i, j] == -1:
                    ax.scatter(k/100., i/100., j/100., c='gray', marker='o', s=0.5)



    ## Choosing the next best view ##
    print("## Computing the score for N rotations of the occupancy grid ##")

    for rotation_number in [1, 2, 3, 4]:
        ## choosing a rotation about the world's z axis ##

        rotation_angle = np.pi / rotation_number # add some randomness
        rotation_matrix = rotation_3d(np.array([0,0,1]), rotation_angle)
        camera_center_in_cube_center_frame = camera_center - np.array([0.5*box_size_x/100., 0.5*box_size_y/100., 0])
        camera_center_in_cube_center_frame_after_rotation = np.matmul(rotation_matrix, camera_center_in_cube_center_frame)

        # ax.scatter([camera_center_in_cube_center_frame_after_rotation[0]], [camera_center_in_cube_center_frame_after_rotation[1]], [camera_center_in_cube_center_frame_after_rotation[2]])

        unknown_points_we_can_probably_see = 0
        occupied_points_we_can_probably_see = 0
        score_dict = {}

        ## Plotting the occupancy grid ##
        for k in range(box_size_x):
            for i in range(box_size_y):
                for j in range(box_size_z):
                    if box[k, i, j] == -1:
                        # we want to check if the camera can see it now
                        point_pos = np.array([k/100., i/100., j/100.])
                        vector = camera_center_in_cube_center_frame_after_rotation - point_pos
                        intersects_with_object = False

                        # we do N steps in the direction of the line until we hit the object, and say the space is free
                        for step_size in np.linspace(0, 1, 500):
                            current_point = point_pos + step_size*vector

                            if current_point[0] >= 0 and current_point[0] <= float(box_size_x)/100 and current_point[1] >= 0 and current_point[1] <= float(box_size_y)/100 and current_point[2] >= 0 and current_point[2] <= float(box_size_z)/100:

                                id_x = int(current_point[0]*100)
                                id_y = int(current_point[1]*100)
                                id_z = int(current_point[2]*100)
                                if box[id_x, id_y, id_z] == 1:
                                    intersects_with_object = True
                                    break
                        if not intersects_with_object:
                            ax.scatter([point_pos[0]], [point_pos[1]], [point_pos[2]], s=0.5, c='g')
                            unknown_points_we_can_probably_see += 1

                    if box[k, i, j] == 1:
                        # we want to check if the camera can see it now
                        point_pos = np.array([k/100., i/100., j/100.])
                        vector = camera_center_in_cube_center_frame_after_rotation - point_pos
                        intersects_with_object = False

                        # we do N steps in the direction of the line until we hit the object, and say the space is free
                        for step_size in np.linspace(0, 1, 500):
                            current_point = point_pos + step_size*vector

                            if current_point[0] >= 0 and current_point[0] <= float(box_size_x)/100 and current_point[1] >= 0 and current_point[1] <= float(box_size_y)/100 and current_point[2] >= 0 and current_point[2] <= float(box_size_z)/100:
                                id_x = int(current_point[0]*100)
                                id_y = int(current_point[1]*100)
                                id_z = int(current_point[2]*100)
                                if box[id_x, id_y, id_z] == 1 and id_x != k and id_y != i and id_z != j:
                                    intersects_with_object = True
                                    break
                        if not intersects_with_object:
                            ax.scatter([point_pos[0]], [point_pos[1]], [point_pos[2]], s=0.5, c='y')
                            occupied_points_we_can_probably_see += 1

        # print("Unknown points we can probably see: {}".format(unknown_points_we_can_probably_see))
        # print("Occupied points we can probably see: {}".format(occupied_points_we_can_probably_see))
        # print("Score: {}".format(float(occupied_points_we_can_probably_see) / (occupied_points_we_can_probably_see + unknown_points_we_can_probably_see)))
        score_dict[rotation_angle] = float(occupied_points_we_can_probably_see) / (occupied_points_we_can_probably_see + unknown_points_we_can_probably_see)

    target_score = 1/3 # ie we want one third of points we already know in the points that we can see next time
    best_angle = score_dict.keys()[0]
    distance_to_target = abs(score_dict[best_angle] - target_score)
    for key in score_dict.keys():
        current_distance_to_target = abs(score_dict[key] - target_score)
        if current_distance_to_target < distance_to_target:
            best_angle = score_dict[key]

    print("Rotation that gives the best score: {}".format(best_angle))

    ax.view_init(elev=0.1, azim=0)
    plt.show()





#
