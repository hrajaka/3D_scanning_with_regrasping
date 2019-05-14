#!/home/cc/ee106b/sp19/class/ee106b-aai/virtualenvironment/my_new_app/bin/python
#!/home/cc/ee106b/sp19/class/ee106b-abj/python-virtual-environments/env/bin/python

#!/home/hasithr/virtualenv/env/bin/python



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
    from path_planner import PathPlanner
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

    if attempts >= max_attempts:
        print('Did not find transform')
        return None
    else:
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
    parser.add_argument('-n_vert', type=int, default=100, help=
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


if __name__ == '__main__':
    print('\n--------------------------------------')
    print('starting main.py')
    print('--------------------------------------\n')

    try:
        args = parse_args()

        if args.debug:
            np.random.seed(0)

        rospy.init_node('main_node')

        print('Getting transform from robot to AR tag...\n')
        T_world_ar = lookup_transform('ar_marker_0', 'base')
        if not T_world_ar:
            print('Did not find TF, using offline version')
            rot = np.array([[ 0.99746769, -0.03630581,  0.06115626],
                            [ 0.03882606,  0.99842334, -0.0405384 ],
                            [-0.05958806,  0.0428102 ,  0.99730464]])
            tra = np.array( [ 0.48215306,  0.16572939, -0.26271284])
            T_world_ar = RigidTransform(rot, tra, 'ar_marker_0', 'base')

        print('Getting transform from camera to AR tag...\n')
        T_ar_cam = lookup_transform('camera_depth_optical_frame', 'ar_marker_2_0')
        if not T_ar_cam:
            print('Did not find TF, using offline version')
            rot = np.array([[ 0.96164055, -0.15495092,  0.2263574 ],
                            [-0.27416062, -0.57037451,  0.77427959],
                            [ 0.00913315, -0.80663693, -0.59097669]])
            tra = np.array( [-0.14501467, -1.161184  ,  0.3797466 ])
            T_ar_cam = RigidTransform(rot, tra, 'ar_marker_2_0', 'base')
        T_ar_cam.from_frame = 'ar_marker_0'
        print(T_ar_cam)

        ## Load and pre-process mesh ##
        filename = '{}.obj'.format(args.obj)
        print('Loading mesh {}...\n'.format(filename))

        mesh = trimesh.load_mesh(filename)
        mesh.fix_normals()

        T_obj_cam = RigidTransform(T_ar_cam.rotation, mesh.centroid)

        ## Visualize the mesh ##
        print('Visualizing mesh (close visualizer window to continue)...\n')
        utils.visualize_mesh(mesh, T_world_ar, T_ar_cam, T_obj_cam)

        print('centroid: ')
        print(mesh.centroid)
        mesh.apply_transform(T_obj_cam.inverse().matrix)
        print('centroid again: ')
        print(mesh.centroid)

       # grasping policies
        print('initializing grasping policy...\n')
        grasping_policy = GraspingPolicy(
            args.n_vert,
            args.n_grasps,
            args.n_execute,
        )
        
        # sample the vertices
        print('sampling vertices...\n')
        vertices, normals = grasping_policy.sample_normals(mesh)
        #utils.visualize_normals(mesh, vertices, normals)
        raw_input('Press ENTER to visualize grasp')
        metrics = grasping_policy.compute_metrics(mesh, vertices, normals)
        utils.visualize_metrics(mesh, vertices, normals, metrics)
        
        ## Picking vertex with best metric ##
        print('Picking the vertex with best metric...\n')

        metrics = np.abs(np.asarray(metrics))
        best_vertex = vertices[np.argmin(metrics)]
        best_normal = normals[np.argmin(metrics)]

        contact_vertices = grasping_policy.grasp_vertices(best_vertex, best_normal)

        contact_normals = np.array([best_normal, -best_normal])

        ## Compute approach direction ang generate gripper pose in object frame ##
        print('Computing approach direction ang generating gripper pose in object frame...\n')
        #approach_direction = compute_approach_direction(mesh, contact_vertices, metrics[np.argmin(metrics)], contact_normals)

        approach_direction = grasping_policy.compute_approach_direction(mesh, contact_vertices)


        approach_direction = - approach_direction
        print('approach direction:')
        print(approach_direction.shape)
        print(approach_direction)
        
        T_obj_grasp = grasping_policy.vertices_to_baxter_hand_pose(contact_vertices, approach_direction, args.obj)
        grasping_policy.vis_transform(mesh, T_obj_grasp, contact_vertices)
        

        ## Generate gripper pose in world frame ##
        print('Generating gripper pose in world frame...\n')
        T_wo = np.matmul(np.matmul(T_world_ar.matrix, T_ar_cam.matrix), T_obj_cam.inverse().matrix) 
        T_og = T_obj_grasp.matrix
        T_wg = np.matmul(T_wo, T_og)
        print(T_wg)

        T_world_obj = RigidTransform(T_wo[:3, :3], T_wo[:3, 3], 'world', 'object')
        T_world_grasp = RigidTransform(T_wg[:3, :3], T_wg[:3, 3], 'world', 'gripper')

        T_obj_world = T_world_obj.inverse()
        T_grasp_world = T_world_grasp.inverse()
        utils.visualize_plan(mesh, T_obj_world, T_grasp_world)
        print(T_world_grasp)

        ## 

        gripper = baxter_gripper.Gripper('right')
        gripper.calibrate()
        planner = PathPlanner('{}_arm'.format('right'))

        
        
        execute_grasp(T_world_grasp, planner, gripper)
        
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
