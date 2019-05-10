#!/home/cc/ee106b/sp19/class/ee106b-aai/virtualenvironment/my_new_app/bin/python
#!/home/cc/ee106b/sp19/class/ee106b-abj/python-virtual-environments/env/bin/python

"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import scipy
import sys
import argparse
# AutoLab imports
from autolab_core import RigidTransform
import trimesh
import warnings
import utils
import policies
warnings.simplefilter("ignore", DeprecationWarning)
from visualization import Visualizer3D as vis3d

# 106B lab imports
from policies import GraspingPolicy
try:
    import rospy
    import tf
    from baxter_interface import gripper as baxter_gripper
    #from path_planner import PathPlanner
    from geometry_msgs.msg import PoseStamped
    ros_enabled = True
except:
    print 'Couldn\'t import ROS.  I assume you\'re running this on your laptop'
    ros_enabled = False



MAX_HAND_DISTANCE = 0.085
MIN_HAND_DISTANCE = 0.045
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1
FINGER_LENGTH = 0.1









def RigidTransformToPoseStamped(G):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "gripper_pose"

        translation = G.translation

        pose.pose.position.x = translation[0]
        pose.pose.position.y = translation[1]
        pose.pose.position.z = translation[2]

        quaternion = G.quaternion
        pose.pose.orientation.w = quaternion[0]
        pose.pose.orientation.x = quaternion[1]
        pose.pose.orientation.y = quaternion[2]
        pose.pose.orientation.z = quaternion[3]

        return pose

def matrixToPoseStamped(mat):

        rot_mat = mat[:3, :3]
        trans = mat[:3,3]

        rigid_transform = RigidTransform(rot_mat, trans, from_frame='from', to_frame='to')

        pose = RigidTransformToPoseStamped(rigid_transform)

        return pose

def lookup_transform(to_frame, from_frame='base'):
    """
    Returns the AR tag position in world coordinates

    Parameters
    ----------
    to_frame : string
        examples are: ar_marker_7, gearbox, pawn, ar_marker_3, etc
    from_frame : string
        lets be real, you're probably only going to use 'base'

    Returns
    -------
    :obj:`autolab_core.RigidTransform` AR tag position or object in world coordinates
    """
    tag_rot = None
    tag_pos = None

    print('CALLING lookup_transform')
    print('to_frame: {}, from_frame: {}'.format(to_frame, from_frame))
    if not ros_enabled:
        print 'I am the lookup transform function!  ' \
            + 'You\'re not using ROS, so I\'m returning the Identity Matrix.'
        return RigidTransform(to_frame=from_frame, from_frame=to_frame)
    print('initializing transformlistener')
    listener = tf.TransformListener()
    attempts, max_attempts, rate = 0, 10, rospy.Rate(1.0)
    while attempts < max_attempts:
        print('attempt {}'.format(attempts))
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
        except Exception as e:
            print(e)
            rate.sleep()
        attempts += 1
    tag_rot = np.array([tag_rot[3], tag_rot[0], tag_rot[1], tag_rot[2]])
    rot = RigidTransform.rotation_from_quaternion(tag_rot)
    return RigidTransform(rot, tag_pos, to_frame=to_frame, from_frame=from_frame)

def execute_grasp(T_world_grasp, planner, gripper):
    """
    takes in the desired hand position relative to the object, finds the desired
    hand position in world coordinates.  Then moves the gripper from its starting
    orientation to some distance BEHIND the object, then move to the  hand pose
    in world coordinates, closes the gripper, then moves up.

    Parameters
    ----------
    T_grasp_world : :obj:`autolab_core.RigidTransform`
        desired position of gripper relative to the world frame
    """

    def close_gripper():
        """closes the gripper"""
        gripper.close(block=True)
        rospy.sleep(1.0)

    def open_gripper():
        """opens the gripper"""
        gripper.open(block=True)
        rospy.sleep(1.0)

    inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
    if inp == "exit":
        return

    ## opening gripper ##
    print('---------- OPENING GRIPPER ----------')
    open_gripper()

    print('---------- MOVING TO INTERMEDIATE POSITION ----------')
    temp_matrix = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -0.1],
                            [0, 0, 0, 1]])
    T1 = np.matmul(T_world_grasp.matrix, temp_matrix)
    pose_stamped_1 = matrixToPoseStamped(T1)

    plan_1 = planner.plan_to_pose(pose_stamped_1)

    planner.execute_plan(plan_1)

    print('---------- MOVING TO FINAL POSITION ----------')
    pose_stamped_2 = RigidTransformToPoseStamped(T_world_grasp)

    plan_2 = planner.plan_to_pose(pose_stamped_2)

    planner.execute_plan(plan_2)

    print('----------CLOSING GRIPPER----------')
    close_gripper()

    print('---------- MOVING TO UP POSITION ----------')
    pose_stamped_3 = RigidTransformToPoseStamped(T_world_grasp)
    pose_stamped_3.pose.position.z = pose_stamped_3.pose.position.z + 0.3

    plan_3 = planner.plan_to_pose(pose_stamped_3)

    planner.execute_plan(plan_3)

    print('---------- MOVING TO LATERAL POSITION ----------')
    pose_stamped_4 = RigidTransformToPoseStamped(T_world_grasp)
    pose_stamped_4.pose.position.y = pose_stamped_4.pose.position.y + 0.2

    plan_4 = planner.plan_to_pose(pose_stamped_4)

    planner.execute_plan(plan_4)

    print('----------OPENING GRIPPER----------')
    open_gripper()

    print('---------- MOVING TO UP POSITION NBR 2 ----------')
    pose_stamped_5 = pose_stamped_4
    pose_stamped_5.pose.position.z = pose_stamped_5.pose.position.z + 0.3
    pose_stamped_5.pose.position.y = pose_stamped_5.pose.position.y - 0.2

    plan_5 = planner.plan_to_pose(pose_stamped_5)

    planner.execute_plan(plan_5)

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-obj', type=str, default='pawn', help=
        """Which Object you\'re trying to pick up.  Options: gearbox, nozzle, pawn.
        Default: gearbox"""
    )
    parser.add_argument('-n_vert', type=int, default=10, help=
        'How many vertices you want to sample on the object surface.  Default: 1000'
    )
    parser.add_argument('-n_facets', type=int, default=32, help=
        """You will approximate the friction cone as a set of n_facets vectors along
        the surface.  This way, to check if a vector is within the friction cone, all
        you have to do is check if that vector can be represented by a POSITIVE
        linear combination of the n_facets vectors.  Default: 32"""
    )
    parser.add_argument('-n_grasps', type=int, default=500, help=
        'How many grasps you want to sample.  Default: 500')
    parser.add_argument('-n_execute', type=int, default=5, help=
        'How many grasps you want to execute.  Default: 5')
    parser.add_argument('-metric', '-m', type=str, default='compute_force_closure', help=
        """Which grasp metric in grasp_metrics.py to use.
        Options: compute_force_closure, compute_gravity_resistance, compute_custom_metric"""
    )
    parser.add_argument('-arm', '-a', type=str, default='right', help=
        'Options: left, right.  Default: right'
    )
    parser.add_argument('--baxter', action='store_true', help=
        """If you don\'t use this flag, you will only visualize the grasps.  This is
        so you can run this on your laptop"""
    )
    parser.add_argument('--debug', action='store_true', help=
        'Whether or not to use a random seed'
    )
    return parser.parse_args()

def compute_approach_direction(mesh, grasp_vertices, grasp_quality, grasp_normals):

    ## initalizing stuff ##
    visualize = True
    nb_directions_to_test = 6
    normal_scale = 0.01
    plane_normal = utils.normalize(grasp_vertices[0] - grasp_vertices[1])

    midpoint = (grasp_vertices[0] + grasp_vertices[1]) / 2

    ## generating a certain number of approach directions ##
    theta = np.pi / nb_directions_to_test
    rot_mat = utils.rotation_3d(-plane_normal, theta)

    horizontal_direction = utils.normalize(np.cross(plane_normal, np.array([0, 0, 1])))
    directions_to_test = [horizontal_direction] #these are vectors
    approach_directions = [np.array([midpoint, midpoint + horizontal_direction * normal_scale])] #these are two points for visualization

    for i in range(nb_directions_to_test-1):
        directions_to_test.append(utils.normalize(np.matmul(rot_mat, directions_to_test[-1])))
        approach_directions.append(np.array([midpoint, midpoint + directions_to_test[-1] * normal_scale]) )

    ## computing the palm position for each approach direction ##
    palm_positions = []
    for i in range(nb_directions_to_test):
        palm_positions.append(midpoint + FINGER_LENGTH * directions_to_test[i])


    if visualize:
        ## plotting the whole mesh ##
        vis3d.mesh(mesh, style='wireframe')

        ## computing and plotting midpoint and gripper position ##
        dirs = (grasp_vertices[0] - grasp_vertices[1]) / np.linalg.norm(grasp_vertices[0] - grasp_vertices[1])
        grasp_endpoints = np.zeros(grasp_vertices.shape)
        grasp_endpoints[0] = midpoint + dirs*MAX_HAND_DISTANCE/2
        grasp_endpoints[1] = midpoint - dirs*MAX_HAND_DISTANCE/2

        color = [min(1, 2*(1-grasp_quality)), min(1, 2*grasp_quality), 0, 1]
        vis3d.plot3d(grasp_endpoints, color=color, tube_radius=.001)
        vis3d.points(midpoint, scale=0.003)

        ## computing and plotting normals at contact points ##
        n0 = np.zeros(grasp_endpoints.shape)
        n1 = np.zeros(grasp_endpoints.shape)
        n0[0] = grasp_vertices[0]
        n0[1] = grasp_vertices[0] + normal_scale * grasp_normals[0]
        n1[0] = grasp_vertices[1]
        n1[1] = grasp_vertices[1] + normal_scale * grasp_normals[1]
        vis3d.plot3d(n0, color=(0, 0, 0), tube_radius=.002)
        vis3d.plot3d(n1, color=(0, 0, 0), tube_radius=.002)

        ## plotting normals the palm positions for each potential approach direction ##
        for i in range(nb_directions_to_test):
            vis3d.points(palm_positions[i], scale=0.003)

        vis3d.show()

    directions_to_test = [directions_to_test[3], directions_to_test[2], directions_to_test[4], directions_to_test[1], directions_to_test[5], directions_to_test[0]]
    palm_positions = [palm_positions[3], palm_positions[2], palm_positions[4], palm_positions[1], palm_positions[5], palm_positions[0]]

    ## checking if some approach direction is valid ##
    for i in range(nb_directions_to_test):
        if len(trimesh.intersections.mesh_plane(mesh, directions_to_test[i], palm_positions[i])) == 0:
            # it means the palm won't bump with part
            return directions_to_test[i]
    
    # it means all approach directions will bump with part 
    return -1

def vertices_to_baxter_hand_pose(grasp_vertices, approach_direction, obj_name):
    """
    takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
    BE CAREFUL ABOUT THE FROM FRAME AND TO FRAME.  the RigidTransform class' frames are
    weird.

    Parameters
    ----------
    grasp_vertices : 2x3 :obj:`numpy.ndarray`
        position of the fingers in object frame
    approach_direction : 3x' :obj:`numpy.ndarray`
        there are multiple grasps that go through contact1 and contact2.  This describes which
        orientation the hand should be in

    Returns
    -------
    :obj:`autolab_core:RigidTransform` Hand pose in the object frame
    """
    # parameters required to create a autolab_core:RigidTransform:
    # - rotation (aka 3x3 rotation matrix)
    # - translation (aka 3x1 vector)
    # - from_frame (aka str)
    # - to_frame (aka str)
    
    midpoint = (grasp_vertices[0] + grasp_vertices[1]) / 2

    gripper_half_width = MAX_HAND_DISTANCE / 2
    
    z = utils.normalize(approach_direction)
    y = utils.normalize(grasp_vertices[0] - grasp_vertices[1])
    x = np.cross(y, z)

    rot_mat_opposite = np.array([x, y, z]).T
    p_opposite = midpoint

    rot_mat = rot_mat_opposite.T
    p = - np.matmul(rot_mat_opposite.T, p_opposite)

    rigid_trans = RigidTransform(rot_mat_opposite, p_opposite, to_frame='right_gripper', from_frame=obj_name) 

    return rigid_trans

def vis_transform(mesh, G_transform, vertices):
        """
        Pass in any grasp and its associated grasp quality.  this function will plot
        each grasp on the object and plot the grasps as a bar between the points, with
        colored dots on the line endpoints representing the grasp quality associated
        with each grasp

        Parameters
        ----------
        mesh : :obj:`Trimesh`
        grasp_vertices : mx2x3 :obj:`numpy.ndarray`
            m grasps.  Each grasp containts two contact points.  Each contact point
            is a 3 dimensional vector, hence the shape mx2x3
        grasp_qualities : mx' :obj:`numpy.ndarray`
            vector of grasp qualities for each grasp
        """
        L = MAX_HAND_DISTANCE / 2 # gripper half width

        # transform from gripper to contact 1
        G_gc1 = np.array([[1,  0,  0,    0],
                         [0,  0,  1, -1*L],
                         [0, -1,  0,    0],
                         [0,  0,  0,    1]])

        # transform from gripper to contact 2
        G_gc2 = np.array([[1,  0,  0,    0],
                         [0,  0, -1,    L],
                         [0,  1,  0,    0],
                         [0,  0,  0,    1]])

        G = G_transform.matrix

        print('G')
        print(G)

        G_oc1 = np.matmul(G, G_gc1)
        G_oc2 = np.matmul(G, G_gc2)


        scale = 0.01
        o = np.array([0, 0, 0, 1])
        x = np.array([scale, 0, 0, 1])
        y = np.array([0, scale, 0, 1])
        z = np.array([0, 0, scale, 1])

        ot = np.matmul(G, o)
        xt = np.matmul(G, x)
        yt = np.matmul(G, y)
        zt = np.matmul(G, z)


        o1 = np.matmul(G_oc1, o)
        x1 = np.matmul(G_oc1, x)
        y1 = np.matmul(G_oc1, y)
        z1 = np.matmul(G_oc1, z)

        o2 = np.matmul(G_oc2, o)
        x2 = np.matmul(G_oc2, x)
        y2 = np.matmul(G_oc2, y)
        z2 = np.matmul(G_oc2, z)

        vis3d.mesh(mesh, style='wireframe')


        
        #Plot origin axes
        x_axis = np.array([o, x])[:, :3]
        y_axis = np.array([o, y])[:, :3]
        z_axis = np.array([o, z])[:, :3]

        x_axis_t = np.array([ot, xt])[:, :3]
        y_axis_t = np.array([ot, yt])[:, :3]
        z_axis_t = np.array([ot, zt])[:, :3]

        x_axis_1 = np.array([o1, x1])[:, :3]
        y_axis_1 = np.array([o1, y1])[:, :3]
        z_axis_1 = np.array([o1, z1])[:, :3]

        x_axis_2 = np.array([o2, x2])[:, :3]
        y_axis_2 = np.array([o2, y2])[:, :3]
        z_axis_2 = np.array([o2, z2])[:, :3]

        center_of_obj = RigidTransform(np.array([[ 1., 0., 0.], [ 0., 1., 0.], [ 0., 0., 1.]]), mesh.centroid)
        vis3d.pose(center_of_obj, alpha=0.01, tube_radius=0.001, center_scale=0.002)


        vis3d.plot3d(x_axis, color=(0.5,0,0), tube_radius=0.001)
        vis3d.plot3d(y_axis, color=(0,0.5,0), tube_radius=0.001)
        vis3d.plot3d(z_axis, color=(0,0,0.5), tube_radius=0.001)

        vis3d.plot3d(x_axis_t, color=(255,0,0), tube_radius=0.001)
        vis3d.plot3d(y_axis_t, color=(0,255,0), tube_radius=0.001)
        vis3d.plot3d(z_axis_t, color=(0,0,255), tube_radius=0.001)

        vis3d.plot3d(x_axis_1, color=(255,0,0), tube_radius=0.001)
        vis3d.plot3d(y_axis_1, color=(0,255,0), tube_radius=0.001)
        vis3d.plot3d(z_axis_1, color=(0,0,255), tube_radius=0.001)

        vis3d.plot3d(x_axis_2, color=(255,0,0), tube_radius=0.001)
        vis3d.plot3d(y_axis_2, color=(0,255,0), tube_radius=0.001)
        vis3d.plot3d(z_axis_2, color=(0,0,255), tube_radius=0.001)

        vis3d.points(vertices[0], scale=0.003)
        vis3d.points(vertices[1], scale=0.003)

        vis3d.show()

if __name__ == '__main__':
    print('\n--------------------------------------')
    print('starting main.py')
    print('--------------------------------------\n')

    try:
        args = parse_args()

        if args.debug:
            np.random.seed(0)

        rospy.init_node('main_node')

        ## Load and pre-process mesh ##
        filename = 'obj/{}.obj'.format(args.obj)
        print('Loading mesh {}...\n'.format(filename))

        ## TODO: put the mesh in the object frame --> remove the cenroid for the translation, ang use the ar tag to find the rotation

        mesh = trimesh.load_mesh(filename)
        mesh.fix_normals()

        ## Visualize the mesh ##
        print('Visualizing mesh (close visualizer window to continue)...\n')
        utils.visualize_mesh(mesh)

<<<<<<< HEAD

        ## Sample the vertices ##
        print('Sampling vertices...\n')
        vertices, ids = trimesh.sample.sample_surface_even(mesh, 10)
=======
       # grasping policies
        
        print('initializing grasping policy...\n')
        grasping_policy = GraspingPolicy(
            args.n_vert,
            args.n_grasps,
            args.n_execute,
        )
        

        # sample the vertices
        print('sampling vertices...\n')
        '''
        vertices, ids = trimesh.sample.sample_surface_even(mesh, 100)
>>>>>>> 3cd2eea8de2d18658e82d08b75280b255ec6ca8b
        normals = mesh.face_normals[ids]
        normals = -1 * normals;
        '''
        vertices, normals = grasping_policy.sample_normals(mesh)
        utils.visualize_normals(mesh, vertices, normals)

        metrics = grasping_policy.compute_metrics(
            mesh, vertices, normals)
        utils.visualize_metrics(mesh, vertices, normals, metrics)
        
        ## Picking vertex with best metric ##
        print('Picking the vertex with best metric...\n')

        metrics = np.abs(np.asarray(metrics))
        best_vertex = vertices[np.argmin(metrics)]
        best_normal = normals[np.argmin(metrics)]

        offset = 0.02
        contact_point1 = best_vertex - offset * best_normal
        contact_point2 = best_vertex + (MAX_HAND_DISTANCE-offset) * best_normal
        contact_vertices = np.array([contact_point1, contact_point2])
        contact_normals = np.array([best_normal, -best_normal])

        ## Compute approach direction ang generate gripper pose in object frame ##
        print('Computing approach direction ang generating gripper pose in object frame...\n')
        approach_direction = compute_approach_direction(mesh, contact_vertices, metrics[np.argmin(metrics)], contact_normals)
        T_obj_grasp = vertices_to_baxter_hand_pose(contact_vertices, approach_direction, args.obj)
        vis_transform(mesh, T_obj_grasp, contact_vertices)
     

        ## Generate gripper pose in world frame ##
        print('Generating gripper pose in world frame...\n')
        T_world_obj = lookup_transform(args.obj)
        T_wo = T_world_obj.matrix
        T_og = T_obj_grasp.matrix
        T_wg = np.matmul(T_wo, T_og)

        T_world_grasp = RigidTransform(T_wg[:3, :3], T_wg[:3, 3], 'world', 'gripper')

        execute_grasp(T_world_grasp, planner, gripper)


        grasp_vertices = grasping_policy.grasp_vertices(
            vertices, normals)
        utils.visualize_grasps(mesh, grasp_vertices, metrics)

        approach_dirs = np.empty((vertices.shape[0], 3))
        for i, g in enumerate(grasp_vertices):
            a = grasping_policy.compute_approach_direction(mesh, g)
            approach_dirs[i] = a

    except rospy.ROSInterruptException:
        pass

    '''
    #mesh.apply_transform(T_world_obj.matrix)
    mesh.fix_normals()


    # This policy takes a mesh and returns the best actions to execute on the robot
    grasping_policy = GraspingPolicy(
        args.n_vert,
        args.n_grasps,
        args.n_execute,
        args.n_facets,
        args.metric
    )

    # Each grasp is represented by T_grasp_world, a RigidTransform defining the
    # position of the end effector

    # Execute each grasp on the baxter / sawyer
    if args.baxter:
        T_world_obj = lookup_transform(args.obj)
        print('T_world_obj')
        print(T_world_obj)
        print('')
        gripper = baxter_gripper.Gripper(args.arm)
        gripper.calibrate()
        planner = PathPlanner('{}_arm'.format(args.arm))

        T_obj_grasps = grasping_policy.top_n_actions(mesh, args.obj)
        T_world_grasps = []

        for i, Tog in enumerate(T_obj_grasps):

            ## computing the final positions ##
            T_wo = T_world_obj.matrix
            T_og = Tog.matrix
            T_wg = np.matmul(T_wo, T_og)

            T_world_grasps.append(RigidTransform(T_wg[:3, :3], T_wg[:3, 3], 'world', 'gripper'))


        for T_world_grasp in T_world_grasps:
            repeat = True

            while repeat:
                execute_grasp(T_world_grasp, planner, gripper)
                repeat = raw_input("repeat? [y|n] ") == 'y'

    else:
        T_grasp_worlds = grasping_policy.top_n_actions(mesh, args.obj)
    '''
