import matplotlib.pyplot as plt
import numpy as np
import autolab_core
import trimesh
from mpl_toolkits.mplot3d import Axes3D
import scipy.spatial
import matplotlib.path



####


if __name__ == '__main__':

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
                   [box_size_x/100, 0,              0             ],
                   [0,              box_size_y/100, 0             ],
                   [0,              0,              box_size_z/100],
                   [box_size_x/100, box_size_y/100, 0             ],
                   [box_size_x/100, 0,              box_size_z/100],
                   [0,              box_size_y/100, box_size_z/100],
                   [box_size_x/100, box_size_y/100, box_size_z/100]]

    ax.plot3D([0,              0,              0,              0, 0, box_size_x/100, box_size_x/100, box_size_x/100, box_size_x/100, box_size_x/100, box_size_x/100,              0,              0, box_size_x/100, box_size_x/100,             0],
              [0,              0, box_size_y/100, box_size_y/100, 0,              0,              0, box_size_y/100, box_size_y/100,              0, box_size_y/100, box_size_y/100, box_size_y/100, box_size_y/100,              0,             0],
              [0, box_size_z/100, box_size_z/100,              0, 0,              0, box_size_z/100, box_size_z/100,              0,              0,              0,              0, box_size_z/100, box_size_z/100, box_size_z/100, box_size_z/100])


    ## Taking care of the points that are on the mesh ##

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
    for k in range(box_size_x):
        print(k)
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
    for k in range(box_size_x):
        print(k)
        for i in range(box_size_y):
            for j in range(box_size_z):
                if box[k, i, j] == 1:
                    ax.scatter(k/100, i/100, j/100, c='k', marker='o', s=0.5)
                if box[k, i, j] == -1:
                    ax.scatter(k/100, i/100, j/100, c='gray', marker='o', s=0.5)


    ax.view_init(elev=0.1, azim=0)
    plt.show()

    ## Choosing the next best view ##

    # TODO





#
