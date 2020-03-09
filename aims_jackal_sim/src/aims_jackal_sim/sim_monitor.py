#!/usr/bin/env python

from aims_jackal_sim.srv import SimReset, SimResetResponse
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_msgs.msg import *
from conf import *
import rospy
import os.path
import numpy as np

class SimMonitor(object):
    def __init__(self):
        rospy.init_node('sim_reset_server')
        s = rospy.Service('sim_reset', SimReset, self.sim_reset)
        self.request_pub = rospy.Publisher('obs/requests', String, queue_size=10)
        self.removed_pub = rospy.Publisher('obs/removed', String, queue_size=10)
        self.qr_codes_sub = rospy.Subscriber("qr_codes", String, self.store_qr)

        # initialise vars
        self.rubble_being_removed = None
        self.rubble_start_removal = None
        self.rubble_recent_obs = None
        self.targets_observed = []
        self.resetting = False
        self.sim_start = rospy.get_time()
        self.last_print = rospy.get_time()
        self.run_completed = False

        # constants
        self.close = 1.8 # distance in metres to request removal
        self.max_sim_time = get_time_lim()

        while not rospy.is_shutdown():
            self.check_run_ended()
            rospy.sleep(0.5)

    def sim_reset(self, req):
        """ configures new doors, targets, and resets robot position """

        # reset vars
        self.resetting = True
        self.doors_to_remove = {}
        self.targets_observed = []
        self.run_completed = False

        # check if the robot is in the correct starting location
        robot_at_start = self.check_robot_location()
        if not robot_at_start:
            print('Robot not in starting location. Not resetting sim.')
            return SimResetResponse(False)

        # remove objects and spawn new ones
        self.delete_all_objects()
        rubbles, targets, robot_loc, cylinder_loc = get_new_config(req.seed)
        self.spawn_rubbles(rubbles)
        self.spawn_targets(targets)
        #self.spawn_cylinder(cylinder_loc)

        # start timer for this run
        print('\n\nSimulation reset, starting timer. \n')
        self.resetting = False
        self.sim_start = rospy.get_time()
        return SimResetResponse(True)

    def check_robot_location(self):
        """ checks if the robot is in the starting or ending region """

        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        world_srv = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # if the jackal is not spawned yet return
        world = world_srv()
        if 'jackal' not in world.model_names:
            return False
        else:
            resp = get_state_srv('jackal', 'world')
            jackal_pose = resp.pose

        region = get_starting_loc()
        in_region = True

        if jackal_pose.position.x < region['min_x']:
            in_region = False
        if jackal_pose.position.x > region['max_x']:
            in_region = False
        if jackal_pose.position.y < region['min_y']:
            in_region = False
        if jackal_pose.position.y > region['max_y']:
            in_region = False
        return in_region

    def store_qr(self, data):
        """ keep track of which targets are seen """
        if self.resetting or self.run_completed:
            return

        qr = data.data
        if 'target' in qr and qr not in self.targets_observed:
            self.targets_observed.append(data.data)
            print(data.data + ' observed.')

        if "rubble" in qr:
            self.track_rubble_removal(qr)

    def track_rubble_removal(self, qr):
        """ keeps track of when it is appropriate to remove the rubble piles.
        a rubble pile is removed if the qr code is continuously viewed and
        the jackal is continuosly near the rubble pile. """

        # if the jackal is not close to the qr then don't do anything
        is_close = self.is_jackal_close(qr)
        is_still = self.is_jackal_still()
        if not (is_close and is_still):
            return

        # if we are not currently removing a rubble set it to this one
        if self.rubble_being_removed is None:
            print('\nstarting to remove ' + qr)
            self.rubble_being_removed = qr
            self.rubble_start_removal = rospy.get_time()
            self.rubble_recent_obs = rospy.get_time()

        # otherwise we are observing a rubble pile again
        else:
            time_now = rospy.get_time()

            # if we haven't observed the rubble pile in two seconds, reset
            if time_now - self.rubble_recent_obs > 1.5:
                print("no longer removing rubble pile.")
                self.rubble_being_removed = None

            # if this is not the rubble pile we are removing return
            elif self.rubble_being_removed != qr:
                return

            # check if its time to remove this rubble
            elif time_now - self.rubble_start_removal > get_removal_time(qr):
                self.remove_rubble(qr)
                print(qr + ' removed.\n')
                self.rubble_being_removed = None

            # otherwise update observation
            else:
                self.rubble_recent_obs = time_now

                if time_now - self.last_print > 2.:
                    self.last_print = time_now
                    print('removing ' + qr + '...')


    def spawn_targets(self, targets):
        """ spawns targets of victims with AR codes """

        for targ in targets:
            self.spawn_target(targ)

    def spawn_target(self, target):
        """ spawns a single target """

        # load the sdf from the model
        my_path = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(my_path, "../../models/" + target['model'] + "/model.sdf")
        f = open(model_path,'r')
        sdf = f.read()
        f.close()

        # spawn target in correct position
        pose = Pose()
        pose.position.x = target['x']
        pose.position.y = target['y']
        pose.position.z = 0.2
        pose.orientation.z = np.sin(target['angle']/2)
        pose.orientation.w = np.cos(target['angle']/2)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(target['model'], sdf, "simulation", pose, "world")

    def spawn_cylinder(self, loc):
        """ spawns the cylinder """

        # load the sdf from the model
        my_path = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(my_path, "../../models/" + "cylinder" + "/model.sdf")
        f = open(model_path,'r')
        sdf = f.read()
        f.close()

        # spawn target in correct position
        pose = Pose()
        pose.position.x = loc['x']
        pose.position.y =loc['y']
        pose.position.z = 0.2
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox('cylinder', sdf, "simulation", pose, "world")

    def check_run_ended(self):
        """ checks if the simulation run is ended. the two criterion are if
        the time limit has expired or if the robot is near the start location
        after more than 30 seconds."""

        # if the world is being reset skip this
        if self.resetting or self.run_completed:
            return

        # if in the starting region and seen all targets
        if self.check_robot_location() and len(self.targets_observed) == get_num_targets():
            print('Mission complete.')
            self.run_completed = True
            self.print_score()

        # if run out of time
        elif rospy.get_time() - self.sim_start > self.max_sim_time:
            print('Run completed due to time limit expired.')
            self.run_completed = True

    def print_score(self):
        """ computes the score for this run """

        points = get_points()
        dt = rospy.get_time() - self.sim_start
        score = points['mission_complete'] + dt*points['per_second']
        print('Mission complete, %.1f points' % (points['mission_complete']))
        print('%.1f seconds elapsed, %.1f points' % (dt, dt*points['per_second']))
        print('Total score: %.1f' % (score))

    def is_jackal_close(self, qr):
        """ checks  if jackal is sufficiently close to a rubble pile.
        to be considered removing that pile. """

        # if the world is being reset skip this
        if self.resetting or self.run_completed:
            return

        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        world_srv = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # if the jackal is not spawned yet return
        world = world_srv()
        if 'jackal' not in world.model_names:
            return
        else:
            resp = get_state_srv('jackal', 'world')
            jackal_pose = resp.pose

        # check for the appropriate rubble pile
        for name in world.model_names:
            if qr == name:
                resp = get_state_srv(name, 'world')
                rubble_pose = resp.pose
                dist = self.dist_two_poses(jackal_pose, rubble_pose)

                # check if jackal visits a door which is not scheduled for removal
                if dist < self.close:
                    return True
        return False

    def is_jackal_still(self):
        """ checks if jackal is not moving, necessary to remove rubble piles """

        # if the world is being reset skip this
        if self.resetting or self.run_completed:
            return

        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        world_srv = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        get_state_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # if the jackal is not spawned yet return
        world = world_srv()
        if 'jackal' not in world.model_names:
            return
        else:
            resp = get_state_srv('jackal', 'world')
            jackal_vel = resp.twist

        if abs(jackal_vel.linear.x) > 0.03 or abs(jackal_vel.angular.z) > 0.03:
            return False
        else:
            return True

    def remove_rubble(self, name):
        """ remove door if removal time has passed """

        rospy.wait_for_service('/gazebo/delete_model')
        delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_srv(name)
        delete_srv('plank_' + name)

    def dist_two_poses(self, pose1, pose2):
        """ calculates the distance in the xy plane between two poses """

        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dist = np.sqrt(dx**2 + dy**2)
        return dist

    def move_robot(self, loc):
        """ moves the robot back to default starting location """

        state = ModelState()
        state.model_name = 'jackal'
        state.pose.position.x = loc['x']
        state.pose.position.y = loc['y']
        state.pose.position.z = 0.1
        state.pose.orientation.z = np.sin(loc['angle']/2)
        state.pose.orientation.w = np.cos(loc['angle']/2)

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state)

    def delete_all_objects(self):
        """ deletes all of the door and target objects from the simulation """

        rospy.wait_for_service('/gazebo/get_world_properties')
        world_srv = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        world = world_srv()

        rospy.wait_for_service('/gazebo/delete_model')
        delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        for name in world.model_names:
            if 'rubble' in name or 'person' in name or 'cylinder' in name or 'plank' in name:
                delete_srv(name)

    def spawn_rubbles(self, rubbles):
        """ spawns each of the doors """

        for rubble in rubbles:
            self.spawn_rubble(rubble)

    def spawn_rubble(self, rubble):
        """ spawns rubble pile found in models directory of this package. """

        name = 'rubble_' + rubble['size'] + str(rubble['loc'])

        # load the sdf from the model
        my_path = os.path.abspath(os.path.dirname(__file__))
        model_path = os.path.join(my_path, "../../models/reduced_" + name + "/model.sdf")
        f = open(model_path,'r')
        sdf = f.read()
        f.close()

        # spawn door in correct position
        pose = Pose()
        pose.position.x = rubble['x']
        pose.position.y = rubble['y']
        pose.position.z = 0.1
        pose.orientation.z = np.sin(rubble['angle']/2)
        pose.orientation.w = np.cos(rubble['angle']/2)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(name, sdf, "simulation", pose, "world")

        # load the sdf from the model
        model_path = os.path.join(my_path, "../../models/plank/model.sdf")
        f = open(model_path,'r')
        sdf = f.read()
        f.close()

        # spawn plank below in correct position
        pose = Pose()
        pose.position.x = rubble['x']
        pose.position.y = rubble['y']
        pose.position.z = 0.05
        pose.orientation.z = np.sin(rubble['angle']/2)
        pose.orientation.w = np.cos(rubble['angle']/2)

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox('plank_'+name, sdf, "simulation", pose, "world")

if __name__ == "__main__":
    sim_monitor = SimMonitor()
