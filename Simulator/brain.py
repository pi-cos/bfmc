#!/usr/bin/python3
SIMULATOR_FLAG = True
SHOW_IMGS = True

import numpy as np
import cv2 as cv
from time import time, sleep
from numpy.linalg import norm
from collections import deque
from names_and_constants import *

if not SIMULATOR_FLAG:
    from control.automobile_data_interface import Automobile_Data
else:
    from automobile_data_interface import Automobile_Data
from helper_functions import *
from PathPlanning4 import PathPlanning
from controller3 import Controller
from controllerSP import ControllerSpeed
from detection import NO_SIGN, Detection

from helper_functions import *

CHECKPOINTS = [86,99,116]
CHECKPOINTS = [86,99,112,337,463,240]
SPEED_CHALLENGE = False

class State():
    def __init__(self, name=None, method=None, activated=False):
        self.name = name
        self.method = method
        self.active = activated
        self.start_time = None
        self.start_position = None
        self.start_distance = None
        self.just_switched = False
        self.interrupted = False
        #variables specific to state, can be freely assigned
        self.var1 = None
        self.var2 = None
        self.var3 = None
        self.var4 = None
    def __str__(self):
        return self.name.upper() if self.name is not None else 'None'
    def run(self):
        self.method()

ALWAYS_ON_ROUTINES = [UPDATE_STATE,CONTROL_FOR_SIGNS]

class Routine():
    def __init__(self, name, method, activated=False):
        self.name = name
        self.method = method
        self.active = activated
        self.start_time = None
        self.start_position = None
        self.start_distance = None
        self.var1 = None
        self.var2 = None
        self.var3 = None
    def __str__(self):
        return self.name
    def run(self):
        self.method()

EVENT_TYPES = [INTERSECTION_STOP_EVENT, INTERSECTION_TRAFFIC_LIGHT_EVENT, INTERSECTION_PRIORITY_EVENT,
                JUNCTION_EVENT, ROUNDABOUT_EVENT, CROSSWALK_EVENT, PARKING_EVENT, HIGHWAY_EXIT_EVENT]

class Event:
    def __init__(self, name=None, dist=None, point=None, yaw_stopline=None, path_ahead=None, length_path_ahead=None, curvature=None):
        self.name = name                # name/type of the event
        self.dist = dist                # distance of event from start of path
        self.point = point              # [x,y] position on the map of the event
        self.yaw_stopline = yaw_stopline# yaw of the stop line at the event
        self.path_ahead = path_ahead    # sequence of points after the event, only for intersections or roundabouts
        self.length_path_ahead = length_path_ahead   # length of the path after the event, only for intersections or roundabouts
        self.curvature = curvature      # curvature of the path ahead of the event
    def __str__(self):
        return self.name.upper() if self.name is not None else 'None'

CONDITIONS = {
    CAN_OVERTAKE:             True,     #if true, the car is in presence of a dotted line and is allowed to overtake it
    HIGHWAY:                  False,    # if true, the car is in a highway, the speed should be higher on highway, 
    TRUST_GPS:                True,     # if true the car will trust the gps, for example in expecting a sign or traffic light
                                        # can be set as false if there is a lot of package loss or the car has not received signal in a while
    CAR_ON_PATH:              True,     # if true, the car is on the path, if the gps is trusted and the position is too far from the path it will be set to false
    REROUTING:                True      # if true, the car is rerouting, for example at the beginning or after a roadblock
}

ACHIEVEMENTS = {
    PARK_ACHIEVED: False
}

#==============================================================
#========================= PARAMTERS ==========================
#==============================================================
SIGN_DIST_THRESHOLD = 0.5
YAW_GLOBAL_OFFSET = 0.0 #global offset of the yaw angle between the real track and the simulator map
#STOPLINES
USE_ADVANCED_NETWORK_FOR_STOPLINES = True
STOP_LINE_APPROACH_DISTANCE = 0.45 if USE_ADVANCED_NETWORK_FOR_STOPLINES else 0.4
STOP_LINE_STOP_DISTANCE = 0.1 if not SPEED_CHALLENGE else 0.2 #0.05
GPS_STOPLINE_APPROACH_DISTANCE = 0.7
GPS_STOPLINE_STOP_DISTANCE = 0.5 if not SPEED_CHALLENGE else 0.6 #0.55
assert STOP_LINE_STOP_DISTANCE <= STOP_LINE_APPROACH_DISTANCE
assert GPS_STOPLINE_STOP_DISTANCE <= GPS_STOPLINE_APPROACH_DISTANCE

STOP_WAIT_TIME = 0.5 if not SPEED_CHALLENGE else 0.0 #3.0
OPEN_LOOP_PERCENTAGE_OF_PATH_AHEAD = 0.6 #0.6
STOP_LINE_DISTANCE_THRESHOLD = 0.2 #distance from previous stop_line from which is possible to start detecting a stop line again
POINT_AHEAD_DISTANCE_LOCAL_TRACKING = 0.3 #0.3
USE_LOCAL_TRACKING_FOR_INTERSECTIONS = True

ACCELERATION_CONST = 1.5 #multiplier for desired speed, used to regulate highway speed
SLOW_DOWN_CONST = 0.3 if not SPEED_CHALLENGE else 1.0#0.3 #multiplier, used when approaching lines
STRAIGHT_DIST_TO_EXIT_HIGHWAY = 0.8 #[m] go straight for this distance in orther to exit the hihgway

#GPS
ALWAYS_TRUST_GPS = False  # if true the car will always trust the gps (bypass)
ALWAYS_DISTRUST_GPS = False #if true, the car will always distrust the gps (bypass)
assert not(ALWAYS_TRUST_GPS and ALWAYS_DISTRUST_GPS), 'ALWAYS_TRUST_GPS and ALWAYS_DISTRUST_GPS cannot be both True'
#Rerouting
GPS_DISTANCE_THRESHOLD_FOR_CONVERGENCE = 0.05 #distance between 2 consecutive measure of the gps for the kalmann filter to be considered converged
GPS_SAMPLE_TIME = 0.25 #[s] time between 2 consecutive gps measurements
GPS_CONVERGENCE_PATIANCE = 2 #iterations to consider the gps converged
GPS_TIMEOUT = 5.0 #[s] time to wait to have gps signal

#PARKING
PARKING_DISTANCE_SLOW_DOWN_THRESHOLD = 1.0
PARKING_DISTANCE_STOP_THRESHOLD = 0.1
SUBPATH_LENGTH_FOR_PARKING = 300 # length in samples of the path to consider around the parking position, max
ALWAYS_USE_GPS_FOR_PARKING = False #debug
ALWAYS_USE_SIGN_FOR_PARKING = False #debug
DEFAULT_PARKING_METHOD = 'gps' #'gps' or 'sign' 
assert not (ALWAYS_USE_GPS_FOR_PARKING and ALWAYS_USE_SIGN_FOR_PARKING)
PARK_MAX_SECONDS_W8_GPS = 10.0 #[s] max seconds to wait for gps to be available 
MAX_PARK_SEARCH_DIST = 2.0 #[m] max distance to search for parking
IDX_OFFSET_FROM_SAVED_PARK_POSITION = 56 +13 #index offset from the saved parking position
PARK_SIGN_DETETCTION_PATIENCE = 8.0 #[s] max seconds to wait for a sign to be available
PARK_SEARCH_SPEED = 0.1 #[m/s] speed to search for parking
PARK_MANOUVER_SPEED = 0.15 #[m/s] speed to perform the parking manouver
DIST_SIGN_FIRST_T_SPOT = 0.75 #[m] distance from the sign to the first parking spot
DIST_T_SPOTS = 0.45 #[m] distance from the sign to the second parking spot
DIST_SIGN_FIRST_S_SPOT = 0.9 #[m] distance from the sign to the first parking spot
DIST_S_SPOTS = 0.7 #[m] distance from the sign to the second parking spot
FURTHER_DIST_S = 0.69 #[m] distance to proceed further in order to perform the s manouver
FURTHER_DIST_T = 0.64 #[m] distance to proceed further in order to perform the t manouver
T_ANGLE = 27.0 #[deg] angle to perform the t manouver
S_ANGLE = 27.0 #[deg] angle to perform the s manouver
DIST_2T = 0.8 #[m] distance to perform the 2nd part of t manouver
DIST_3T = 0.1 #[m] distance to perform the 3rd part of t manouver 
DIST_2S = 0.38 #[m] distance to perform the 2nd part of s manouver
DIST_4S = 0.05 #[m] distance to perform the 4th part of s manouver
STEER_ACTUATION_DELAY_PARK = 0.5 #[s] delay to perform the steering manouver
SLEEP_AFTER_STOPPING = 0.2 #[s] WARNING: this stops the state machine. So be careful increasing it
STEER_ACTUATION_DELAY = 0.3 #[s] delay to perform the steering manouver

# OBSTACLES
OBSTACLE_IS_ALWAYS_PEDESTRIAN = False
OBSTACLE_IS_ALWAYS_CAR = False
OBSTACLE_IS_ALWAYS_ROADBLOCK = False
MIN_DIST_BETWEEN_OBSTACLES = 0.5 #dont detect obstacle for this distance after detecting one of them
assert OBSTACLE_IS_ALWAYS_PEDESTRIAN ^ OBSTACLE_IS_ALWAYS_CAR ^ OBSTACLE_IS_ALWAYS_ROADBLOCK or not (OBSTACLE_IS_ALWAYS_PEDESTRIAN or OBSTACLE_IS_ALWAYS_CAR or OBSTACLE_IS_ALWAYS_ROADBLOCK)
OBSTACLE_DISTANCE_THRESHOLD = 0.5 #[m] distance from the obstacle to consider it as an obstacle
OBSTACLE_CONTROL_DISTANCE = 0.25 #distance to where to stop wrt the obstacle
OBSTACLE_IMGS_CAPTURE_START_DISTANCE = 0.45 #dist from where we capture imgs 
OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE = 0.28 #dist up to we capture imgs
assert OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE > OBSTACLE_CONTROL_DISTANCE
PEDESTRIAN_CONTROL_DISTANCE = 0.35 #[m] distance to keep from the pedestrian
PEDESTRIAN_TIMEOUT = 2.0 #[s] time to w8 after the pedestrian cleared the road
TAILING_DISTANCE = 0.3 #[m] distance to keep from the vehicle while tailing
OVERTAKE_STEER_ANGLE = 27.0 #[deg]
OVERTAKE_STATIC_CAR_SPEED = 0.2  #[m/s]
OT_STATIC_SWITCH_1 = 0.3
OT_STATIC_SWITCH_2 = 0.4
OT_STATIC_LANE_FOLLOW = 0.45
AVOID_ROADBLOCK_ANGLE = 27.0 #[deg]
AVOID_ROADBLOCK_SPEED = 0.2 #[m/s]
AVOID_ROADBLOCK_DISTANCE = 0.6 #[m]

#CHECKS
MAX_DIST_AWAY_FROM_LANE = 0.8 #[m] max distance from the lane to trip the state checker
MAX_ERROR_ON_LOCAL_DIST = 0.05 #[m] max error on the local distance

#==============================================================
#=========================== BRAIN ============================
#==============================================================
class Brain:
    def __init__(self, car, controller, controller_sp, detection, path_planner, checkpoints=None, desired_speed=0.3, debug=True):
        print("Initialize brain")
        self.car = Automobile_Data() #not needed, just to import he methods in visual studio
        self.car = car
        assert isinstance(self.car, Automobile_Data)
        self.controller = Controller() #again, not needed
        self.controller = controller
        self.controller_sp = ControllerSpeed() #again, not needed
        self.controller_sp = controller_sp
        assert isinstance(self.controller, Controller)
        self.detect = Detection() #again, not needed
        self.detect = detection
        assert isinstance(self.detect, Detection)
        self.path_planner = PathPlanning(None)
        self.path_planner = path_planner
        assert isinstance(self.path_planner, PathPlanning)
        
        #navigation instruction is a list of tuples:
        # ()
        self.navigation_instructions = []
        # events are an ordered list of tuples: (type , distance from start, x y position)
        self.events = []
        self.checkpoints = checkpoints if checkpoints is not None else CHECKPOINTS
        self.checkpoint_idx = 0
        self.desired_speed = desired_speed
        self.parking_method = DEFAULT_PARKING_METHOD 
        if ALWAYS_USE_GPS_FOR_PARKING:
            self.parking_method = 'gps'
        if ALWAYS_USE_SIGN_FOR_PARKING:
            self.parking_method = 'sign'

        #current and previous states (class State)
        self.curr_state = State()
        self.prev_state = State()
        #previous and next event (class Event)
        self.prev_event = Event()
        self.next_event = Event()
        self.event_idx = 0

        #stop line with higher precision
        self.stop_line_distance_median = 1.0
        self.car_dist_on_path = 0

        #debug
        self.debug = debug
        if self.debug and SHOW_IMGS:
            cv.namedWindow('brain_debug', cv.WINDOW_NORMAL)
            self.debug_frame = None

        self.conditions = CONDITIONS
        self.achievements = ACHIEVEMENTS

        # INITIALIZE STATES

        self.states = { 
            START_STATE:              State(START_STATE, self.start_state),
            END_STATE:                State(END_STATE, self.end_state),
            DOING_NOTHING:            State(DOING_NOTHING, self.doing_nothing),
            # lane following, between intersections or roundabouts
            LANE_FOLLOWING:           State(LANE_FOLLOWING, self.lane_following),
            # intersection navigation, further divided into the possible directions [left, right, straight]
            APPROACHING_STOP_LINE:    State(APPROACHING_STOP_LINE, self.approaching_stop_line), 
            INTERSECTION_NAVIGATION:  State(INTERSECTION_NAVIGATION, self.intersection_navigation),
            GOING_STRAIGHT:           State(GOING_STRAIGHT, self.going_straight),
            TRACKING_LOCAL_PATH:      State(TRACKING_LOCAL_PATH, self.tracking_local_path),
            #roundabout, for roundabouts the car will track the local path
            ROUNDABOUT_NAVIGATION:    State(ROUNDABOUT_NAVIGATION, self.roundabout_navigation),
            #waiting states
            WAITING_FOR_PEDESTRIAN:   State(WAITING_FOR_PEDESTRIAN, self.waiting_for_pedestrian),
            WAITING_FOR_GREEN:        State(WAITING_FOR_GREEN, self.waiting_for_green),
            WAITING_AT_STOPLINE:      State(WAITING_AT_STOPLINE, self.waiting_at_stopline),
            WAITING_FOR_REROUTING:    State(WAITING_FOR_REROUTING, self.waiting_for_rerouting),
            #overtaking manouver
            OVERTAKING_STATIC_CAR:    State(OVERTAKING_STATIC_CAR, self.overtaking_static_car),
            OVERTAKING_MOVING_CAR:    State(OVERTAKING_MOVING_CAR, self.overtaking_moving_car),
            TAILING_CAR:              State(TAILING_CAR, self.tailing_car),
            AVOIDING_ROADBLOCK:       State(AVOIDING_ROADBLOCK, self.avoiding_roadblock),
            #parking
            PARKING:                  State(PARKING, self.parking),
            #crosswalk navigation
            CROSSWALK_NAVIGATION:     State(CROSSWALK_NAVIGATION, self.crosswalk_navigation),
            CLASSIFYING_OBSTACLE:     State(CLASSIFYING_OBSTACLE, self.classifying_obstacle)
        }

        # INITIALIZE ROUTINES
        self.routines = {
            FOLLOW_LANE:              Routine(FOLLOW_LANE,  self.follow_lane),      
            DETECT_STOP_LINE:         Routine(DETECT_STOP_LINE,  self.detect_stop_line),   
            SLOW_DOWN:                Routine(SLOW_DOWN,  self.slow_down),     
            ACCELERATE:               Routine(ACCELERATE,  self.accelerate),         
            CONTROL_FOR_SIGNS:        Routine(CONTROL_FOR_SIGNS,  self.control_for_signs),         
            CONTROL_FOR_SEMAPHORE:    Routine(CONTROL_FOR_SEMAPHORE,  self.control_for_semaphore), 
            CONTROL_FOR_PEDESTRIANS:  Routine(CONTROL_FOR_PEDESTRIANS,  self.control_for_pedestrians), 
            CONTROL_FOR_VEHICLES:     Routine(CONTROL_FOR_VEHICLES,  self.control_for_roadblocks),   
            CONTROL_FOR_ROADBLOCKS:   Routine(CONTROL_FOR_ROADBLOCKS,  self.control_for_roadblocks),
            CONTROL_FOR_OBSTACLES:    Routine(CONTROL_FOR_OBSTACLES,  self.control_for_obstacles),
            UPDATE_STATE:             Routine(UPDATE_STATE, self.update_state)  
        }
        self.active_routines_names = []

        self.sign_points = np.load('data/sign_points.npy')
        self.sign_types = np.load('data/sign_types.npy').astype(int)
        assert len(self.sign_points) == len(self.sign_types)
        self.sign_seen = np.zeros(len(self.sign_types))
        self.curr_sign = NO_SIGN

        self.frame_for_stopline_angle = None



        self.last_run_call = time()
        print('Brain initialized')
        self.switch_to_state(START_STATE)
        # self.switch_to_state(DOING_NOTHING)

    #=============== STATES ===============#
    def start_state(self):
        if self.curr_state.just_switched:
            self.conditions[REROUTING] = True
            self.car.drive_speed(0.0) #stop
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (np.array([0.0,0.0]), 0) #(position, counter)
            self.curr_state.var2 = time()
            self.curr_state.just_switched = False
        
        #localize the car and go to the first checkpoint
        #for now we will assume to be in the correct position
        can_generate_route = False
        curr_time = time()
        if self.conditions[TRUST_GPS]:
            self.curr_state.var2 = time() #reset timer
            curr_pos = np.array([self.car.x_est, self.car.y_est])
            prev_pos, cnt = self.curr_state.var1
            if norm(curr_pos - prev_pos) < GPS_DISTANCE_THRESHOLD_FOR_CONVERGENCE:
                cnt += 1
                if cnt >= GPS_CONVERGENCE_PATIANCE:
                    can_generate_route = True
                    #get closest node
                    closest_node, distance = self.path_planner.get_closest_node(curr_pos)
                    print(f'GPS converged, starting from node: {closest_node}, distance: {distance:.2f}')
                    self.checkpoints[self.checkpoint_idx] = closest_node
                    if distance > 0.8:
                        print('ERROR: REROUTING: GPS converged, but distance is too large, we are too far from the lane')
                        sleep(3)
                        exit()

                else: print(f'Waiting for GPS convergence... {cnt}/{GPS_CONVERGENCE_PATIANCE}')
            else:
                cnt = 0
                print('GPS not converged yet')
            self.curr_state.var1 = (curr_pos, cnt)
            sleep(GPS_SAMPLE_TIME)
        else:
            start_time = self.curr_state.var2
            print(f'Waiting for gps: {(curr_time-start_time):.1f}/{GPS_TIMEOUT}')
            if curr_time - start_time > GPS_TIMEOUT:
                print('WARNING: ROUTE_GENERATION: No gps signal or GPS not trusted, Starting from the first checkpoint')
                can_generate_route = True
        
        if can_generate_route:
            print('Generating route...')
            #get start and end nodes from the chekpoint list
            assert len(self.checkpoints) >= 2, 'List of checkpoints needs 2 ore more nodes'
            start_node = self.checkpoints[self.checkpoint_idx]
            end_node = self.checkpoints[self.checkpoint_idx+1] #already checked in end_state
            print(f'Start node: {start_node}, End node: {end_node}')
            #calculate path
            self.path_planner.compute_shortest_path(start_node, end_node)
            #initialize the list of events on the path
            print('Augmenting path...')
            events = self.path_planner.augment_path(draw=SHOW_IMGS)
            print(f'Path augmented')
            #add the events to the list of events, increasing it
            self.events = self.create_sequence_of_events(events)
            self.event_idx = 1
            self.next_event = self.events[0]
            self.prev_event.dist = 0.0
            self.car.reset_rel_pose()
            print(f'EVENTS: idx: {self.event_idx}')
            for e in self.events:
                print(e)
            
            #draw the path 
            self.path_planner.draw_path()
            print('Starting...')
            self.conditions[REROUTING] = False
            self.sign_seen = np.zeros_like(self.sign_seen) #reset the signs seen
            # sleep(3)
            # cv.waitKey(0)
            self.switch_to_state(LANE_FOLLOWING)
            self.car.drive_speed(self.desired_speed)

    def end_state(self):
        self.activate_routines([])
        self.car.drive_speed(0.0) #TODO control in position
        self.go_to_next_event()
        #start routing for next checkpoint
        self.next_checkpoint()
        self.switch_to_state(START_STATE)

    def doing_nothing(self):
        self.activate_routines([])

    def lane_following(self): # LANE FOLLOWING ##############################
        #start driving if it's the first time it has been called
        if self.curr_state.just_switched:
            self.car.drive_speed(self.desired_speed)
            self.activate_routines([FOLLOW_LANE, DETECT_STOP_LINE, CONTROL_FOR_OBSTACLES])
            self.curr_state.just_switched = False
        
        #check parking
        if self.next_event.name == PARKING_EVENT:
            self.activate_routines([FOLLOW_LANE, CONTROL_FOR_OBSTACLES]) #we dont need stoplines in parking
            dist_between_events = self.next_event.dist - self.prev_event.dist
            #Relative positioning is reset at every stopline, so we can use that to approximately calculate the distance to the parking spot
            approx_dist_from_parking = dist_between_events - self.car.dist_loc
            print(f'Approx dist from parking: {approx_dist_from_parking}')
            if approx_dist_from_parking < PARKING_DISTANCE_SLOW_DOWN_THRESHOLD: #we are reasonably close to the parking spot
                # self.activate_routines([FOLLOW_LANE, SLOW_DOWN])
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.switch_to_state(PARKING)

        #check highway exit case
        elif self.next_event.name == HIGHWAY_EXIT_EVENT:
            if self.conditions[TRUST_GPS]:
                diff = self.next_event.dist - self.car_dist_on_path 
                if 0.0 < diff:
                    print(f'Driving toward highway exit: exiting in {diff:.2f} [m]')
                elif -0.05 < diff <= 0.0:
                    print(f'Arrived at highway exit, switching to going straight for exiting')
                    self.switch_to_state(GOING_STRAIGHT)
                else:
                    print('ERROR: LANE FOLLOWING: Missed Highway exit')
                    self.car.stop()
                    sleep(3)
                    exit()
            else:
                raise NotImplementedError #TODO implement this case with signs

        # end of current route, go to end state
        elif self.next_event.name == END_EVENT:
            self.activate_routines([FOLLOW_LANE, CONTROL_FOR_OBSTACLES])
            if self.conditions[TRUST_GPS]: #NOTE End is implemented only with gps now, much more robust, but cannot do it without it
                dist_to_end = len(self.path_planner.path)*0.01 - self.car_dist_on_path
                if dist_to_end > 0.2:
                    print(f'Driving toward end: exiting in {dist_to_end:.2f} [m]')
                elif -0.2 < dist_to_end <= 0.2:
                    print(f'Arrived at end, switching to end state')
                    self.switch_to_state(END_STATE)
                else:
                    print('ERROR: LANE FOLLOWING: Missed end')
                    self.car.stop()
                    sleep(3)
                    exit()
            
        #check if we are approaching a stop_line, but only if we are far enough from the previous stop_line
        else:
            if self.conditions[TRUST_GPS]:
                dist_to_stopline = self.next_event.dist - self.car_dist_on_path
                if GPS_STOPLINE_APPROACH_DISTANCE <= dist_to_stopline: 
                    print(f'Stopline is far: {dist_to_stopline-GPS_STOPLINE_APPROACH_DISTANCE:.2f} [m]')
                if 0.0 < dist_to_stopline < GPS_STOPLINE_APPROACH_DISTANCE:
                    print(f'Switching to approaching stopline')
                    self.switch_to_state(APPROACHING_STOP_LINE)
                else:
                    print(f'It seems we passed the stopline, but its probably because the path self intersected, IGNORING ERROR')
            else:
                far_enough_from_prev_stop_line = (self.event_idx == 1) or (self.car.dist_loc > STOP_LINE_DISTANCE_THRESHOLD)
                print(f'stop enough: {self.car.dist_loc}') if self.prev_event.name is not None else None
                if self.detect.est_dist_to_stop_line < STOP_LINE_APPROACH_DISTANCE and far_enough_from_prev_stop_line and self.routines[DETECT_STOP_LINE].active:
                    self.switch_to_state(APPROACHING_STOP_LINE)


        if self.conditions[HIGHWAY]:
            self.add_routines([ACCELERATE])


    def approaching_stop_line(self):
        self.activate_routines([FOLLOW_LANE, SLOW_DOWN, DETECT_STOP_LINE, CONTROL_FOR_OBSTACLES]) #FOLLOW_LANE, SLOW_DOWN, DETECT_STOP_LINE, CONTROL_FOR_OBSTACLES

        if self.conditions[TRUST_GPS] and False:
            dist_to_stopline = self.next_event.dist - self.car_dist_on_path
            if GPS_STOPLINE_APPROACH_DISTANCE <= dist_to_stopline:
                print(f'Switching to lane following')
                self.switch_to_state(LANE_FOLLOWING)
                return
            elif GPS_STOPLINE_STOP_DISTANCE <= dist_to_stopline < GPS_STOPLINE_APPROACH_DISTANCE:
                print(f'Approaching stop line: {dist_to_stopline-GPS_STOPLINE_STOP_DISTANCE:.2f} [m]')
                decide_next_state = False
            elif 0.0 <= dist_to_stopline < GPS_STOPLINE_STOP_DISTANCE:
                print(f'Arrived at stop line')
                decide_next_state = True
            else:
                print('ERROR: APPROACHING STOP LINE: Missed stop line')
                self.car.stop()
                sleep(3)
                exit()
        else:
            dist = self.detect.est_dist_to_stop_line
            # #check if we are here by mistake
            if dist > STOP_LINE_APPROACH_DISTANCE:
                self.switch_to_state(LANE_FOLLOWING)
                return
            if self.stop_line_distance_median is not None and False: #we have a median, => we have an accurate position for the stopline
                print('Driving towards stop line... at distance: ', self.stop_line_distance_median)
                self.activate_routines([SLOW_DOWN]) #FOLLOW_LANE, SLOW_DOWN#deactivate detect stop line
                dist_to_drive = self.stop_line_distance_median - self.car.encoder_distance
                self.car.drive_distance(dist_to_drive)
                if dist_to_drive < STOP_LINE_STOP_DISTANCE: 
                    print(f'Arrievd at stop line. Using median distance: {self.stop_line_distance_median}')
                    print(f'                           encoder distance: { self.car.encoder_distance:.2f}')
                    # sleep(1.0)
                    decide_next_state = True
                else: decide_next_state = False
            else: #alternative, if we don't have a median, we just use the (possibly inaccurate) network estimaiton
                print('WARNING: APPROACHING_STOP_LINE: stop distance may be imprecise')
                if dist < STOP_LINE_STOP_DISTANCE:
                    print('Stopped at stop line. Using network distance: ', self.detect.est_dist_to_stop_line) 
                    decide_next_state = True
                    dist_from_line = dist if self.stop_line_distance_median is None else self.stop_line_distance_median - self.car.encoder_distance
                    assert dist_from_line < 0.5, f'dist_from_line is too large, {dist_from_line:.2f}'
                else: decide_next_state = False

        if decide_next_state:
            print('Deciding next state, based on next event...')
            next_event_name = self.next_event.name
            # Events with stopline
            if next_event_name == INTERSECTION_STOP_EVENT:
                self.switch_to_state(WAITING_AT_STOPLINE) 
            elif next_event_name == INTERSECTION_TRAFFIC_LIGHT_EVENT:
                self.switch_to_state(WAITING_FOR_GREEN)
            elif next_event_name == INTERSECTION_PRIORITY_EVENT:
                self.switch_to_state(INTERSECTION_NAVIGATION)
            elif next_event_name == JUNCTION_EVENT:
                self.switch_to_state(INTERSECTION_NAVIGATION) #TODO: careful with this
            elif next_event_name == ROUNDABOUT_EVENT:
                self.switch_to_state(ROUNDABOUT_NAVIGATION)
            elif next_event_name == CROSSWALK_EVENT:
                self.switch_to_state(CROSSWALK_NAVIGATION) #directly go to lane keeping, the pedestrian will be managed in that state
            # Events without stopline = LOGIC ERROR
            elif next_event_name == PARKING_EVENT:
                print('WARNING: UNEXPECTED STOP LINE FOUND WITH PARKING AS NEXT EVENT')
                self.car.stop()
                sleep(3)
                exit() #TODO: handle this case
            elif next_event_name == HIGHWAY_EXIT_EVENT:
                print('WARNING: UNEXPECTED STOP LINE FOUND WITH HIGHWAY EXIT AS NEXT EVENT')
                self.car.stop()
                sleep(3)
                exit() #TODO: handle this case
            else:
                print('ERROR: UNEXPECTED STOP LINE FOUND WITH UNKNOWN EVENT AS NEXT EVENT')
                self.car.stop()
                sleep(3)
                exit()
            self.activate_routines([]) #deactivate all routines

    def intersection_navigation(self):
        self.activate_routines([])
        if USE_LOCAL_TRACKING_FOR_INTERSECTIONS: #use local tracking, more reliable if local yaw estimaiton and localization are good
            self.switch_to_state(TRACKING_LOCAL_PATH)
        else: # go for precoded manouvers, less precise and less robust, but predictable
            if self.next_event.curvature > 0.5:
                self.switch_to_state(TURNING_RIGHT)
            elif self.next_event.curvature < -0.5:
                self.switch_to_state(TURNING_LEFT)
            else:
                self.switch_to_state(GOING_STRAIGHT)
            self.car.drive_speed(self.desired_speed)
        
    def going_straight(self):
        if self.curr_state.just_switched:
            self.activate_routines([])
            if self.next_event.name == HIGHWAY_EXIT_EVENT:
                distance_to_stop = STRAIGHT_DIST_TO_EXIT_HIGHWAY
            else:
                distance_to_stop = self.curr_state.start_distance+OPEN_LOOP_PERCENTAGE_OF_PATH_AHEAD*self.next_event.length_path_ahead
            self.curr_state.var1 = distance_to_stop + self.car.encoder_distance
            self.curr_state.just_switched = False

        distance_to_stop = self.curr_state.var1
        if self.car.encoder_distance < distance_to_stop: 
            self.car.drive(speed=self.desired_speed, angle=0.0)
        else: #end of the maneuver
            self.switch_to_state(LANE_FOLLOWING)
            self.go_to_next_event()

    def tracking_local_path(self):
        print('State: tracking_local_path') #var1=initial distance from stop_line, #var2=path to follow
        self.activate_routines([])
        if self.curr_state.just_switched:
            stop_line_position = self.next_event.point
            stop_line_yaw = self.next_event.yaw_stopline
            local_path_slf_rot = self.next_event.path_ahead #local path in the stop line frame
    
            if self.conditions[TRUST_GPS] and False:
                USE_PRECISE_LOCATION_AND_YAW = True
                point_car_est = np.array([self.car.x_est, self.car.y_est])
                point_car_path = self.path_planner.path[int(round(self.car_dist_on_path*100))]

                if USE_PRECISE_LOCATION_AND_YAW:
                    angle = self.car.yaw
                    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
                    car_position_slf = point_car_est - stop_line_position
                    car_position_slf = car_position_slf @ rot_matrix
                else:
                    point_ahead = self.path_planner.path[int(round(self.car_dist_on_path*100+10))]
                    point_behind = self.path_planner.path[int(round(self.car_dist_on_path*100-10))]
                    yaw_path = np.arctan2(point_ahead[1]-point_behind[1], point_ahead[0]-point_behind[0])
                    yaw_error_path = np.arctan2(point_car_est[1]-point_car_path[1], point_car_est[0]-point_car_path[0])
                    sign = np.sign(diff_angle(yaw_path, yaw_error_path))
                    x_dist = self.next_event.dist - self.car_dist_on_path 
                    y_dist = sign*norm(point_car_est-point_car_path)*0.0
                    car_position_slf = -np.array([x_dist, y_dist])
                print('Car position in stop line frame: ', car_position_slf)
            else:
                if USE_ADVANCED_NETWORK_FOR_STOPLINES:
                    stopline_x, stopline_y, stopline_angle = self.detect.detect_stop_line2(self.car.frame, show_ROI=True)
                    e2 = stopline_y
                else:
                    self.detect.detect_stop_line(self.car.frame, SHOW_IMGS)
                    e2, _, _ = self.detect.detect_lane(self.car.frame, SHOW_IMGS)
                    e2 = 0.0 # NOTE e2 is usually bad
                if self.stop_line_distance_median is not None and False:
                    print('We HAVE the median, using median estimation')
                    print(len(self.routines[DETECT_STOP_LINE].var2))
                    d = self.stop_line_distance_median - self.car.encoder_distance 
                else: #we do not have an accurate position for the stopline
                    print('We DONT have the median, using simple net estimation')
                    print(len(self.routines[DETECT_STOP_LINE].var2))
                    if self.detect.est_dist_to_stop_line < STOP_LINE_APPROACH_DISTANCE:
                        d = self.detect.est_dist_to_stop_line
                    else: d = 0.0

                car_position_slf = -np.array([+d+0.38, +e2])#-np.array([+d+0.3+0.15, +e2])#np.array([+d+0.2, -e2])

            # get orientation of the car in the stop line frame
            yaw_car = self.car.yaw
            yaw_mult_90 = get_yaw_closest_axis(yaw_car)
            alpha = diff_angle(yaw_car, yaw_mult_90) #get the difference from the closest multiple of 90deg
            alpha_true = alpha
            print(f'alpha true: {np.rad2deg(alpha):.1f}')
            alpha = self.detect.detect_yaw_stopline(self.car.frame, SHOW_IMGS and False) * 0.8
            print(f'alpha est: {np.rad2deg(alpha):.1f}')
            assert np.abs(alpha - alpha_true) < np.deg2rad(5.0), f'Estimated alpha is too different from true alpha'
            assert abs(alpha) < np.pi/6, f'Car orientation wrt stopline is too big, it needs to be better aligned, alpha = {alpha}'
            rot_matrix = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])
            
            # ## get position of the car in the stop line frame
            local_path_cf = local_path_slf_rot 
            local_path_cf = local_path_cf @ rot_matrix #NOTE: rotation first if we ignore the lateral error and consider only the euclidean distance from the line
            local_path_cf = local_path_cf - car_position_slf #cf = car frame
            #rotate from slf to cf
            self.curr_state.var1 = local_path_cf
            #Every time we stop for a stopline, we reset the local frame of reference
            self.car.reset_rel_pose() 
            self.curr_state.just_switched = False

            if SHOW_IMGS:
                img = self.car.frame.copy()
                #project the whole path (true)
                img, _ = project_onto_frame(img, self.car, self.path_planner.path, align_to_car=True, color=(0, 100, 0))
                #project local path (estimated), it should match the true path
                img, _ = project_onto_frame(img, self.car, local_path_cf, align_to_car=False)
                cv.imshow('brain_debug', img)
                cv.waitKey(1)
                self.curr_state.var2 = np.array([self.car.x_true, self.car.y_true]) #var2 hold original position
                true_start_pos_wf = self.curr_state.var2

                alpha = alpha + stop_line_yaw
                rot_matrix = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])

                est_car_pos_slf = car_position_slf
                est_car_pos_slf_rot = est_car_pos_slf @ rot_matrix.T
                est_car_pos_wf = est_car_pos_slf_rot + stop_line_position
                cv.circle(self.path_planner.map, mR2pix(est_car_pos_wf), 25, (255, 0, 255), 5)
                cv.circle(self.path_planner.map, mR2pix(true_start_pos_wf), 30, (0, 255, 0), 5)
                cv.imshow('Path', self.path_planner.map)
                cv.waitKey(1)
                #debug
                # self.car.drive_speed(0.0)
                # sleep(SLEEP_AFTER_STOPPING)
                # sleep(1.0)
                cv.namedWindow('local_path', cv.WINDOW_NORMAL)
                local_map_img = np.zeros_like(self.path_planner.map)
                h = local_map_img.shape[0]
                w = local_map_img.shape[1]
                local_map_img[w//2-2:w//2+2, :] = 255
                local_map_img[:, h//2-2:h//2+2] = 255

                cv.circle(local_map_img, (w//2,h//2), 50, (255, 0, 255), 5)
                for i in range(len(local_path_cf)):
                    if (i%3 == 0):
                        p = local_path_cf[i]
                        pix = mR2pix(p)
                        pix = (int(pix[0]+w//2), int(pix[1]-h//2))
                        cv.circle(local_map_img, pix, 10, (0, 150, 150), -1)
                cv.imshow('local_path', local_map_img)
                cv.waitKey(1)  
                self.curr_state.var3 = local_map_img

        D = POINT_AHEAD_DISTANCE_LOCAL_TRACKING
        #track the local path using simple pure pursuit
        local_path = self.curr_state.var1
        car_pos_loc = np.array([self.car.x_loc, self.car.y_loc])
        local_path_cf = local_path - car_pos_loc
        dist_path = norm(local_path_cf, axis=1)
        #get idx of car position on the path
        idx_car_on_path = np.argmin(dist_path)
        dist_path = dist_path[idx_car_on_path:]
        dist_path = np.abs(dist_path - D)
        #get idx of point ahead
        idx_point_ahead = np.argmin(dist_path) + idx_car_on_path
        # idx_point_ahead = int(100*self.car.dist_loc) # m -> cm -> idx
        print(f'idx_point_ahead: {idx_point_ahead} / {len(local_path_cf)}')

        rot_matrix = np.array([[np.cos(self.car.yaw_loc), -np.sin(self.car.yaw_loc)], [np.sin(self.car.yaw_loc), np.cos(self.car.yaw_loc)]])
        local_path_cf = local_path_cf @ rot_matrix

        if SHOW_IMGS:
            local_map_img = self.curr_state.var3
            h = local_map_img.shape[0]
            w = local_map_img.shape[1]
            angle = self.car.yaw_loc_o # + self.car.yaw_loc
            rot_matrix_w = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            # show car position in the local frame (from the encoder)
            cv.circle(local_map_img, (mR2pix(car_pos_loc)[0]+w//2, mR2pix(car_pos_loc)[1]-h//2), 5, (255, 0, 255), 2)
            # show the true position to check if they match, translated wrt starting position into the local frame
            true_start_pos_wf = self.curr_state.var2
            true_pos_loc = np.array([self.car.x_true, self.car.y_true]) - true_start_pos_wf
            true_pos_loc = true_pos_loc @ rot_matrix_w
            cv.circle(local_map_img, (mR2pix(true_pos_loc)[0]+w//2, mR2pix(true_pos_loc)[1]-h//2), 7, (0, 255, 0), 2)
            cv.imshow('local_path', local_map_img)
            true_start_pos_wf = self.curr_state.var2
            car_pos_loc_rot_wf = car_pos_loc @ rot_matrix_w.T
            car_pos_wf = true_start_pos_wf + car_pos_loc_rot_wf
            # show car position in wf (encoder)
            cv.circle(self.path_planner.map, mR2pix(car_pos_wf), 5, (255, 0, 255), 2)
            # show the true position to check if they match
            true_pos_wf = np.array([self.car.x_true, self.car.y_true])
            cv.circle(self.path_planner.map, mR2pix(true_pos_wf), 7, (0, 255, 0), 2)
            cv.imshow('Path', self.path_planner.map)
            cv.waitKey(1)
        if np.abs(get_curvature(local_path_cf)) < 0.1: #the local path is straight
            print('straight')
            max_idx = len(local_path_cf)-60 #dont follow until the end
        else: #curvy path
            max_idx = len(local_path_cf)-1  #follow until the end
            print('curvy')
        # State exit conditions
        if idx_point_ahead >= max_idx: #we reached the end of the path
            self.switch_to_state(LANE_FOLLOWING)
            self.go_to_next_event()
        else: #we are still on the path
            point_ahead = local_path_cf[idx_point_ahead]
            if SHOW_IMGS:
                img = self.car.frame.copy()
                img, _ = project_onto_frame(img, self.car, local_path_cf, align_to_car=False)
                img, _ = project_onto_frame(img, self.car, point_ahead, align_to_car=False, color=(0,0,255))
                cv.imshow('brain_debug', img)
                cv.waitKey(1)
            gains = [0.0, .0, 1.2, 0.0] #k1,k2,k3,k3D
            e2 = local_path_cf[idx_car_on_path][1] 
            yaw_error = np.arctan2(point_ahead[1], point_ahead[0]) 
            out_speed, out_angle = self.controller.get_control(e2, yaw_error, 0.0, self.desired_speed, gains=gains)
            self.car.drive(out_speed, np.rad2deg(out_angle))

    def roundabout_navigation(self):
        self.switch_to_state(TRACKING_LOCAL_PATH)

    def waiting_for_pedestrian(self):
        self.activate_routines([FOLLOW_LANE])
        dist_ahead = self.car.filtered_sonar_distance
        if self.curr_state.just_switched:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            self.activate_routines([])
            dist_to_keep = dist_ahead - PEDESTRIAN_CONTROL_DISTANCE
            print('dist_ahead: ', dist_ahead)
            print(f'dist_to_keep: {dist_to_keep}')
            self.car.drive_distance(dist_to_keep) #go to a safe dist from the pedestrian and w8 there
            self.curr_state.var1 = time() #last time I saw the pedestrian, initiliaze it
            self.curr_state.just_switched = False
        if dist_ahead < OBSTACLE_DISTANCE_THRESHOLD:
            print('Waiting for pedestrian...')
            self.activate_routines([])
            self.curr_state.var1 = last_seen_pedestrian_time = time()
        else: 
            last_seen_pedestrian_time = self.curr_state.var1
            self.activate_routines([])
            curr_time = time()
            print(f'Pedestrian has cleared the road, keep waiting for: {curr_time - last_seen_pedestrian_time}/{PEDESTRIAN_TIMEOUT}')
            if curr_time - last_seen_pedestrian_time > PEDESTRIAN_TIMEOUT:
                # self.switch_to_state(LANE_FOLLOWING) 
                self.car.stop()
                sleep(SLEEP_AFTER_STOPPING)
                print(f'Swithching back to {self.prev_state}')
                self.switch_to_state(LANE_FOLLOWING) #NOTE
            
    def waiting_for_green(self):
        #temporary fix
        self.switch_to_state(WAITING_AT_STOPLINE)

    def waiting_at_stopline(self):
        self.activate_routines([]) #no routines
        if STOP_WAIT_TIME > 0.0:
            self.car.drive_speed(0.0)
            sleep(SLEEP_AFTER_STOPPING)
            if (time() - self.curr_state.start_time) > STOP_WAIT_TIME:
                self.switch_to_state(INTERSECTION_NAVIGATION)
        else:
            self.switch_to_state(INTERSECTION_NAVIGATION)

    def waiting_for_rerouting(self):
        pass

    def overtaking_static_car(self):
        self.activate_routines([])
        #states
        OT_SWITCHING_LANE = 1
        OT_LANE_FOLLOWING = 2 
        OT_SWITCHING_BACK = 3

        if self.curr_state.just_switched:
            self.curr_state.var1 = (OT_SWITCHING_LANE, True)
            self.curr_state.var2 = self.car.encoder_distance
            self.curr_state.just_switched = False
        sub_state, just_sub_switched = self.curr_state.var1
        dist_prev_manouver = self.curr_state.var2
        if sub_state == OT_SWITCHING_LANE:
            if just_sub_switched:
                self.car.drive_angle(-OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                self.car.drive_speed(OVERTAKE_STATIC_CAR_SPEED)
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver 
            assert dist > -0.05
            print(f'Switching lane: {dist:.2f}/{OT_STATIC_SWITCH_1:.2f}')
            if dist > OT_STATIC_SWITCH_1:
                sub_state, just_sub_switched = OT_LANE_FOLLOWING, True
                dist_prev_manouver = self.car.encoder_distance
        elif sub_state == OT_LANE_FOLLOWING:
            self.activate_routines([FOLLOW_LANE])
            dist = self.car.encoder_distance - dist_prev_manouver
            print(f'Following lane: {dist:.2f}/{OT_STATIC_LANE_FOLLOW:.2f}')
            if dist > OT_STATIC_LANE_FOLLOW:
                sub_state, just_sub_switched = OT_SWITCHING_BACK, True
                dist_prev_manouver = self.car.encoder_distance 
        elif sub_state == OT_SWITCHING_BACK:
            if just_sub_switched:
                self.car.drive_angle(OVERTAKE_STEER_ANGLE)
                dist_prev_manouver = self.car.encoder_distance
                just_sub_switched = False
            dist = self.car.encoder_distance - dist_prev_manouver 
            assert dist > -0.05
            print(f'Switching back: {dist:.2f}/{OT_STATIC_SWITCH_2:.2f}')
            if dist > OT_STATIC_SWITCH_2:
                self.switch_to_state(LANE_FOLLOWING)
        else:
            print('ERROR: OVERTAKE: Wrong substate')
            self.car.stop()
            sleep(3)
            exit()

        self.curr_state.var1 = (sub_state, just_sub_switched)
        self.curr_state.var2 = dist_prev_manouver

    def overtaking_moving_car(self):
        pass

    def tailing_car(self):
        dist = self.car.filtered_sonar_distance
        if dist > OBSTACLE_DISTANCE_THRESHOLD:
            self.switch_to_state(LANE_FOLLOWING)
        else:
            dist_to_drive = dist - TAILING_DISTANCE
            self.car.drive_distance(dist_to_drive)
            if self.conditions[CAN_OVERTAKE]:
                if -0.05 < dist_to_drive < 0.05:
                    self.switch_to_state(OVERTAKING_STATIC_CAR)

    def avoiding_roadblock(self):
        self.activate_routines([])
        AR_WATING_FOR_GPS = 1
        AR_SWITCHING_LANE = 2
        NODES_LEFT = ['16','138','137','136','135','134','7']
        NODES_RIGHT = ['15','143','142','141','140','139','8']
        if self.curr_state.just_switched:
            self.car.drive_speed(0.0) #stop
            sleep(SLEEP_AFTER_STOPPING)
            self.curr_state.var1 = (np.array([0.0,0.0]), 0) #(position, counter)
            self.curr_state.var2 = time()
            self.curr_state.var3 = (AR_WATING_FOR_GPS, True)
            self.curr_state.var4 = False #IN RIGHT LANE  
            self.curr_state.just_switched = False
        
        substate, just_switched_substate = self.curr_state.var3
        print(f'Substate: {substate}, just switched: {just_switched_substate}')
        if substate == AR_WATING_FOR_GPS:
            #localize the car and go to the first checkpoint
            #for now we will assume to be in the correct position
            curr_time = time()
            if self.conditions[TRUST_GPS]:
                self.curr_state.var2 = time() #reset timer
                curr_pos = np.array([self.car.x_est, self.car.y_est])
                prev_pos, cnt = self.curr_state.var1
                if norm(curr_pos - prev_pos) < GPS_DISTANCE_THRESHOLD_FOR_CONVERGENCE:
                    cnt += 1
                    if cnt >= GPS_CONVERGENCE_PATIANCE:
                        self.curr_state.var3 = (AR_SWITCHING_LANE, True)
                        closest_node, distance = self.path_planner.get_closest_node(curr_pos)
                        print(f'GPS converged, node: {closest_node}, distance: {distance:.2f}')
                        if closest_node in NODES_LEFT:
                            self.curr_state.var4 = True
                        elif closest_node in NODES_RIGHT:
                            self.curr_state.var4 = False
                        else:
                            print('ERROR: ROADBLOCK: GPS converged but we are not in a possible node for switching lane')
                            self.car.stop()
                            sleep(3)
                            exit()
                        if distance > 0.8:
                            print('ERROR: REROUTING: GPS converged, but distance is too large, we are too far from the lane')
                            sleep(3)
                            exit()
                    else: print(f'Waiting for GPS convergence... {cnt}/{GPS_CONVERGENCE_PATIANCE}')
                else:
                    cnt = 0
                    print('GPS not converged yet')
                self.curr_state.var1 = (curr_pos, cnt)
                sleep(GPS_SAMPLE_TIME)
            else:
                start_time = self.curr_state.var2
                print(f'Waiting for gps: {(curr_time-start_time):.1f}/{GPS_TIMEOUT}')
                if curr_time - start_time > GPS_TIMEOUT:
                    print('WARNING: ROUTE_GENERATION: No gps signal or GPS not trusted, Starting from the first checkpoint')
                    sleep(3.0)
        elif substate == AR_SWITCHING_LANE:
            in_right_lane = self.curr_state.var4
            if just_switched_substate:
                if in_right_lane:
                    print('Switching to left lane')
                    self.car.drive_angle(-AVOID_ROADBLOCK_ANGLE)
                else:
                    print('Switching to right lane')
                    self.car.drive_angle(AVOID_ROADBLOCK_ANGLE)
                sleep(STEER_ACTUATION_DELAY)
                self.car.drive_speed(AVOID_ROADBLOCK_SPEED)
                self.curr_state.var1 = start_encoder_pos = self.car.encoder_distance
                self.curr_state.var3 = (AR_SWITCHING_LANE, False) #just switched = false
            else:
                start_encoder_pos = self.curr_state.var1
                curr_encoder_dist = self.car.encoder_distance
                if curr_encoder_dist - start_encoder_pos > AVOID_ROADBLOCK_DISTANCE*0.6:
                    self.car.drive_angle(+AVOID_ROADBLOCK_ANGLE) if in_right_lane else self.car.drive_angle(-AVOID_ROADBLOCK_ANGLE)
                if curr_encoder_dist - start_encoder_pos > AVOID_ROADBLOCK_DISTANCE:
                    print('Arrived, switching back to rerouting')
                    self.car.drive_speed(0.0)
                    self.switch_to_state(START_STATE)
        else:
            print('ERROR: AVOIDING_ROADBLOCK: Wrong substate')
            self.car.stop()
            sleep(3)
            exit()
    

    def parking(self):
        #Substates
        LOCALIZING_PARKING_SPOT = 1
        CHECKING_FOR_PARKED_CARS = 2
        STEP0 = 69
        T_STEP1 = 3
        T_STEP2 = 4
        T_STEP3 = 5
        T_STEP4 = 6
        T_STEP5 = 7
        S_STEP1 = 8
        S_STEP2 = 9
        S_STEP3 = 10
        S_STEP4 = 11
        S_STEP5 = 12
        S_STEP6 = 13
        S_STEP7 = 14
        S_STEP8 = 15
        PARK_END = 16
        #park types
        T_PARK = 't'
        S_PARK = 's'
        if self.curr_state.just_switched:
            # We just got in the parking state, we came from lane following, we are reasonably close to the parking spot and we are not moving
            # self.curr_state.var1 will hold the parking substate, the parking type, and if it has just changed state
            park_state = LOCALIZING_PARKING_SPOT
            #find park type with position
            park_pos = self.next_event.point
            s_pos = np.array(self.path_planner.get_coord('177'))
            t_pos = np.array(self.path_planner.get_coord('162'))
            print(park_pos)
            print(s_pos)
            print(t_pos)
            d_s = norm(park_pos - s_pos) #distances from the s parking spot
            d_t = norm(park_pos - t_pos) #distances from the t parking spot
            if d_s < d_t and d_s < 0.2: park_type = S_PARK
            elif d_t <= d_s and d_t < 0.2: park_type = T_PARK
            else:
                print('ERROR: PARKING -> parking spot is not close to expected parking spot position!')
                exit()
            self.curr_state.var1 = (park_state, park_type, True)           
            self.curr_state.just_switched = False
        
        park_pos = self.next_event.point
        park_state, park_type, just_changed = self.curr_state.var1

        #############################################################################################
        #first state: localizing with precision the parking spot
        if park_state == LOCALIZING_PARKING_SPOT:
            print('LOCALIZING_PARKING_SPOT')
            self.activate_routines([FOLLOW_LANE]) 

            if (self.parking_method == 'gps' or ALWAYS_USE_GPS_FOR_PARKING) and not ALWAYS_USE_SIGN_FOR_PARKING: 
                print('Using gps for parking')
                if just_changed:
                    trusted_gps_once = False #this will become true if we trusted the gps at least once. We will use local pos afterward
                    self.curr_state.var2 = trusted_gps_once #var2 
                    self.curr_state.var1 = (park_state, park_type, False) #just_changed = False
                    self.car.reset_rel_pose() 

                trusted_gps_once = self.curr_state.var2
                
                if not self.conditions[TRUST_GPS] and not trusted_gps_once: 
                    self.car.drive_speed(0.0)
                    curr_time = time()
                    passed_time = curr_time - self.curr_state.start_time
                    if passed_time > PARK_MAX_SECONDS_W8_GPS:
                        print('ERROR: GPS Timout!')
                        print('Using sign for parking...')
                        self.curr_state.var1 = (LOCALIZING_PARKING_SPOT, park_type, True)
                        self.parking_method = 'sign'
                    print(f'Parking: GPS not trusted, waiting for GPS to be trusted for {passed_time}/{PARK_MAX_SECONDS_W8_GPS} [s]...')
                else: #gps is trusted or we have already trusted it
                    self.curr_state.var2 = trusted_gps_once = True #we trusted gps once
                    car_est_pos = np.array([self.car.x_est, self.car.y_est])
                    # car_est_pos = np.array([self.car.x_true, self.car.y_true]) # DEBUG ONLY
                    park_index_on_path = int(self.next_event.dist*100) #one sample for every cm in the path
                    path_to_analyze = self.path_planner.path[max(0, park_index_on_path-SUBPATH_LENGTH_FOR_PARKING):
                                        min(park_index_on_path+SUBPATH_LENGTH_FOR_PARKING, len(self.path_planner.path))]
                    car_idx_on_path = np.argmin(norm(path_to_analyze - car_est_pos, axis=1))
                    park_index_on_path = SUBPATH_LENGTH_FOR_PARKING
                    if car_idx_on_path < park_index_on_path and self.car.dist_loc < MAX_PARK_SEARCH_DIST:
                        print('Behind parking spot')
                        self.car.drive_speed(PARK_SEARCH_SPEED)
                        if car_idx_on_path > park_index_on_path - IDX_OFFSET_FROM_SAVED_PARK_POSITION:
                            print('We arrived at the parking spot')
                            self.car.drive_speed(0.0)
                            self.curr_state.var1 = (CHECKING_FOR_PARKED_CARS, park_type, True)
                        else: print(f'getting closer...  dist: {self.car.dist_loc:.2f}/{MAX_PARK_SEARCH_DIST:.2f}')
                    else: 
                        print('ERROR: PARKING: In front of parking spot, or maximum search distance reached')
                        self.car.drive_speed(0.0)
                        sleep(3)
                        raise NotImplementedError 
            elif (self.parking_method == 'sign' or ALWAYS_USE_SIGN_FOR_PARKING) and not ALWAYS_USE_GPS_FOR_PARKING:
                print('Using sign for parking')
                if just_changed:
                    #create a deque of past PARK_SIGN_DETETCTION_PATIENCE sing detection results
                    park_sign_counter = 0
                    self.curr_state.var2 = park_sign_counter #assign var2 to the queue
                    self.car.reset_rel_pose() #reset the car pose to the current pose
                    self.curr_state.var1 = (park_state, park_type, False) #set var1, with just_changed to false
                    self.curr_state.var3 = False #parking sign reached or not
                    self.car.drive_speed(PARK_SEARCH_SPEED)
                    self.curr_state.var1 = (park_state, park_type, False)

                sign, _, _, _ = self.detect.detect_sign(self.car.frame, show_ROI=True)
                park_sign_counter = self.curr_state.var2
                parking_spot_reached = self.curr_state.var3

                if not parking_spot_reached: 
                    if sign == 'park': park_sign_counter += 1
                    else: park_sign_counter = 0
                    if park_sign_counter >= PARK_SIGN_DETETCTION_PATIENCE:
                        self.curr_state.var3 = True #parking sign reached
                        park_sign_counter = 0
                        print('Reached parking spot, keep going until the sign disappears')
                else: #parking sign reached
                    if sign != 'park':
                        park_sign_counter += 1
                    if park_sign_counter >= PARK_SIGN_DETETCTION_PATIENCE:
                        print('Sign disappeared, setting up things for searching for parked cars')
                        self.car.drive_speed(0.0)
                        #go to next substate
                        self.curr_state.var1 = (CHECKING_FOR_PARKED_CARS, park_type, True)
                self.curr_state.var2 = park_sign_counter

        #############################################################################################
        #second state: checking if there are parked cars in the parking spots    
        elif park_state == CHECKING_FOR_PARKED_CARS:
            print('Checking for parked cars...')
            if just_changed:
                self.activate_routines([FOLLOW_LANE])
                assert self.next_event.name == PARKING_EVENT
                self.car.reset_rel_pose()
                self.curr_state.var2 = (False, False, False, False) #(car in spot1, car in spot2, looked for car in spot1, looked for car in spot2)
                self.car.drive_speed(PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)

            car_in_spot1, car_in_spot2, checked1, checked2 = self.curr_state.var2
            curr_dist = self.car.dist_loc

            if park_type == T_PARK:
                dist_first_spot = DIST_SIGN_FIRST_T_SPOT
                dist_spots = DIST_T_SPOTS
                further_dist = FURTHER_DIST_T
            elif park_type == S_PARK:
                dist_first_spot = DIST_SIGN_FIRST_S_SPOT
                dist_spots = DIST_S_SPOTS
                further_dist = FURTHER_DIST_S
            else:
                print('ERROR: PARKING: Unknown parking type!')
                self.car.stop()
                sleep(5)
                exit()

            if (dist_first_spot < curr_dist < (dist_first_spot+0.1)) and not checked1:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                #get lateral sonar distance
                lateral_sonar_dist = self.car.filtered_lateral_sonar_distance
                checked1 = True
                if lateral_sonar_dist < 0.5:
                    print('Car in spot 1')
                    car_in_spot1 = True
                self.car.drive_speed(PARK_MANOUVER_SPEED)
            elif (dist_first_spot+dist_spots < curr_dist < (dist_first_spot+dist_spots+0.1)) and not checked2:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                #get lateral sonar distance
                lateral_sonar_dist = self.car.filtered_lateral_sonar_distance
                checked2 = True
                if lateral_sonar_dist < 0.5:
                    print('Car in spot 2')
                    car_in_spot2 = True
                self.car.drive_speed(PARK_MANOUVER_SPEED)
            elif dist_first_spot+dist_spots+further_dist < curr_dist:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Ending search for parked cars')
                print('Car in spot 1: {}'.format(car_in_spot1))
                print('Car in spot 2: {}'.format(car_in_spot2))
                sleep(1)
                self.curr_state.var1 = (STEP0, park_type, True)
            if dist_first_spot+dist_spots+further_dist + MAX_ERROR_ON_LOCAL_DIST < curr_dist:
                print(f'ERROR: PARKING: CHECKING_CARS: Overshoot distance, error: {dist_first_spot+dist_spots+further_dist + MAX_ERROR_ON_LOCAL_DIST-curr_dist:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError

            self.curr_state.var2 = (car_in_spot1, car_in_spot2, checked1, checked2) #update var2 at the end of every iteration
        
        #############################################################################################
        #STEP 0 -> ALIGN WITH THE PARKING SPOT
        elif park_state == STEP0:
            #we are standing still at the end of the 2 parking spots 
            print('STEP0 -> Aligning with the parking spot...')

            car_in_spot1, car_in_spot2, _, _ = self.curr_state.var2

            if just_changed:
                if car_in_spot1 and car_in_spot2: #TODO change the behaviour: go to next event or something
                    print('ERROR: PARKING: Car in both spots!')
                    self.car.stop()
                    sleep(3)
                    exit()
                elif not car_in_spot2:
                    print('Spot 2 is free. Going to spot 2 now')
                    park_state = T_STEP2 if park_type == T_PARK else S_STEP2
                    self.curr_state.var1 = (park_state, park_type, True)
                else: #car in spot2, spot1 free
                    print('Spot 1 is free. Going to spot 1 now')
                    self.activate_routines([FOLLOW_LANE])
                    self.car.reset_rel_pose()
                    self.car.drive_speed(-PARK_MANOUVER_SPEED)
                    self.curr_state.var1 = (park_state, park_type, False)

            if not car_in_spot1 and car_in_spot2:
                dist = self.car.dist_loc
                print(f'Distance: {dist}')
                dist_spots = DIST_S_SPOTS if park_type == S_PARK else DIST_T_SPOTS
                if dist > dist_spots:
                    self.car.drive_speed(0.0)
                    sleep(SLEEP_AFTER_STOPPING)
                    print('Aligned with first parking spot')
                    sleep(1)
                    park_state = T_STEP2 if park_type == T_PARK else S_STEP2
                    self.curr_state.var1 = (park_state, park_type, True)
                if dist > dist_spots + MAX_ERROR_ON_LOCAL_DIST:
                    print(f'ERROR: PARKING: STEP0: Overshoot distance, error:{dist_spots + MAX_ERROR_ON_LOCAL_DIST - dist:.2f}')
                    self.car.stop()
                    sleep(3)
                    raise NotImplementedError

        elif park_state == T_STEP2:
            print('T-parking manouver step 2, we are aligned with the parking spot, going right and backward')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(+T_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2T:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (T_STEP3, park_type, True)
            if self.car.dist_loc > DIST_2T + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: T_STEP2: Overshoot distance, error:{DIST_2T + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == T_STEP3:
            print('T-parking manouver step 3, going backward')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_3T}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(0.0)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_3T:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Parked')
                self.achievements[PARK_ACHIEVED] = True #<><><><><><><><><><><><><> PARK ACHIEVED <><><><><><><><><><><><><>
                sleep(3.0)
                self.curr_state.var1 = (T_STEP4, park_type, True)
            if self.car.dist_loc > DIST_3T + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: T_STEP3: Overshoot distance, error:{DIST_3T + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == T_STEP4:
            print('T-parking manouver step 4, going forward')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_3T}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(0.0)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_3T:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (T_STEP5, park_type, True)
            if self.car.dist_loc > DIST_3T + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: T_STEP4: Overshoot distance, error:{DIST_3T + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == T_STEP5:
            print('T-parking manouver step 5, going right and forward')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2T}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(+T_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(+PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2T:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Back in lane')
                self.curr_state.var1 = (PARK_END, park_type, True)
            if self.car.dist_loc > DIST_2T + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: T_STEP5: Overshoot distance, error:{DIST_2T + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        
        # # S parking manouver
        # elif park_state == S_STEP1:
        #     print('S-parking manouver step 1')
        elif park_state == S_STEP2:
            print('S-parking manouver step 2')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(+S_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (S_STEP3, park_type, True)
            if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP2: Overshoot distance, error:{DIST_2S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == S_STEP3:
            print('S-parking manouver step 3')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(-S_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (S_STEP4, park_type, True)
            if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP3: Overshoot distance, error:{DIST_2S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == S_STEP4:
            print('S-parking manouver step 4')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_4S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(0.0)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(+PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_4S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Parked')
                self.achievements[PARK_ACHIEVED] = True #<><><><><><><><><><><><><> PARK ACHIEVED <><><><><><><><><><><><><>
                sleep(3.0)
                self.curr_state.var1 = (S_STEP5, park_type, True)
            if self.car.dist_loc > DIST_4S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP4: Overshoot distance, error:{DIST_4S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == S_STEP5:
            print('S-parking manouver step 5')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_4S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(0.0)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(-PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_4S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                print('Parked')
                self.curr_state.var1 = (S_STEP6, park_type, True)
            if self.car.dist_loc > DIST_4S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP5: Overshoot distance, error:{DIST_4S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == S_STEP6:
            print('S-parking manouver step 6')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(-S_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(+PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (S_STEP7, park_type, True)
            if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP6: Overshoot distance, error:{DIST_2S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError
        elif park_state == S_STEP7:
            print('S-parking manouver step 7')
            print(f'Distance: {self.car.dist_loc:.2f}/{DIST_2S}')
            self.activate_routines([])
            if just_changed:
                self.car.drive_angle(+S_ANGLE)
                sleep(STEER_ACTUATION_DELAY_PARK)
                self.car.reset_rel_pose()
                self.car.drive_speed(+PARK_MANOUVER_SPEED)
                self.curr_state.var1 = (park_state, park_type, False)
            if self.car.dist_loc > DIST_2S:
                self.car.drive_speed(0.0)
                sleep(SLEEP_AFTER_STOPPING)
                self.curr_state.var1 = (PARK_END, park_type, True)
            if self.car.dist_loc > DIST_2S + MAX_ERROR_ON_LOCAL_DIST:
                print(f'ERROR: PARKING: S_STEP7: Overshoot distance, error:{DIST_2S + MAX_ERROR_ON_LOCAL_DIST - self.car.dist_loc:.2f}')
                self.car.stop()
                sleep(3)
                raise NotImplementedError

        #end of manouver, go to next event
        elif park_state == PARK_END:
            self.switch_to_state(LANE_FOLLOWING)
            self.go_to_next_event()

    def crosswalk_navigation(self):
        self.activate_routines([CONTROL_FOR_OBSTACLES])
        #Every time we stop for a stopline, we reset the local frame of reference
        self.car.reset_rel_pose() 
        self.go_to_next_event()
        self.switch_to_state(LANE_FOLLOWING)
    
    def classifying_obstacle(self):
        self.activate_routines([FOLLOW_LANE])
        IMGS_DEQUE_MAX_LEN = 20
        dist = self.car.filtered_sonar_distance
        if self.curr_state.just_switched:
            #drive to fixed dist from the obstacle
            dist_to_drive = dist - OBSTACLE_CONTROL_DISTANCE
            self.car.drive_distance(dist_to_drive)
            #initialize deque of images
            self.curr_state.var1 = deque(maxlen=IMGS_DEQUE_MAX_LEN) #stored in var1
            dist_save_imgs = dist_to_drive 
            self.curr_state.var2 = (0, np.linspace(OBSTACLE_IMGS_CAPTURE_START_DISTANCE, OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE, IMGS_DEQUE_MAX_LEN)) #var2 holds a touple: (index, position where to take the image)
            self.curr_state.var3 = []
            self.curr_state.just_switched=False

        if dist >= OBSTACLE_DISTANCE_THRESHOLD: #
            print('Sonar got confused: switch back to previous state')
            self.switch_to_prev_state()
            return

        #more readable variables
        imgs_deque = self.curr_state.var1
        idx, img_distances = self.curr_state.var2
        distances = self.curr_state.var3

        if dist > OBSTACLE_IMGS_CAPTURE_STOP_DISTANCE: #we are approaching the obstacle
            print('Capturing imgs')
            if dist < img_distances[idx]:
                print(f'Saving img {idx}, at distance: {img_distances[idx]}, sonar_dist: {dist} ')
                frame_to_save = self.car.frame.copy()
                imgs_deque.append(frame_to_save)
                distances.append(dist)
                idx += 1
        else:
            if len(imgs_deque) > 0:
                ## DO SOMETHING WITH THE IMAGES
                frames = [img for img in imgs_deque]
                print(f'Captured {len(frames)} imgs, running classification...')
                obstacle, conf = self.detect.classify_frontal_obstacle(frames, distances, show_ROI=SHOW_IMGS)    
                print(f'Obstacle: {obstacle}')
                sleep(1)
                # for i,img in enumerate(imgs_deque):
                #     cv.imshow(f'img {i}', img)
                # SWITCH STATE
                if OBSTACLE_IS_ALWAYS_CAR: obstacle = 'car'
                if OBSTACLE_IS_ALWAYS_PEDESTRIAN: obstacle = 'pedestrian'
                if OBSTACLE_IS_ALWAYS_ROADBLOCK: obstacle = 'roadblock'
                if obstacle == 'car':
                    self.switch_to_state(TAILING_CAR)
                elif obstacle == 'pedestrian':
                    self.switch_to_state(WAITING_FOR_PEDESTRIAN)
                elif obstacle == 'roadblock':
                    self.switch_to_state(AVOIDING_ROADBLOCK)
                else:
                    print('ERROR: OBSTACLE CLASSIFICATION: Unknown obstacle')
                    exit()
            else:
                print('ERROR: FRONT DETECTION: Couldnt capture images while approachin obstacle')
                self.car.stop()
                sleep(3)
                exit()

        #reassing the variables
        self.curr_state.var1 = imgs_deque
        self.curr_state.var2 = (idx, img_distances)
        self.curr_state.var3 = distances

    #=============== ROUTINES ===============#
    def follow_lane(self):
        if not SPEED_CHALLENGE:
            e2, e3, point_ahead = self.detect.detect_lane(self.car.frame, SHOW_IMGS)
            #NOTE 
            e3 = -e3 if not SIMULATOR_FLAG else e3 
            _, angle_ref = self.controller.get_control(e2, e3, 0, self.desired_speed)
            angle_ref = np.rad2deg(angle_ref)
            if self.car.speed < 0.1: #inverse driving in reverse
                angle_ref = -angle_ref
            print(f'angle_ref: {angle_ref}')
            self.car.drive_angle(angle_ref)
        else:
            e3, _ = self.detect.detect_lane_ahead(self.car.frame)
            output_speed, output_angle = self.controller_sp.get_control_speed(e3)
            print(f'output_speed: {output_speed:.2f}, output_angle: {np.rad2deg(output_angle):.2f}')
            self.car.drive(speed=output_speed, angle=np.rad2deg(output_angle))

    def detect_stop_line(self):
        #update the variable self.detect.est_dist_to_stop_line
        # x_stop, y_stop, yaw_stop = self.detect.detect_stop_line(self.car.frame, SHOW_IMGS)
        if USE_ADVANCED_NETWORK_FOR_STOPLINES:
            stopline_x, stopline_y, stopline_angle = self.detect.detect_stop_line2(self.car.frame, show_ROI=True)
            dist = stopline_x
        else:
            dist = self.detect.detect_stop_line(self.car.frame, SHOW_IMGS)

        past_detections = self.routines[DETECT_STOP_LINE].var2
        if dist < STOP_LINE_APPROACH_DISTANCE-0.1:
            DETECTION_DEQUE_LENGTH = 50
            SAMPLE_BEFORE_CONFIDENCE = 20
            # var1 holds last detection time
            last_detection_time = self.routines[DETECT_STOP_LINE].var1 if self.routines[DETECT_STOP_LINE].var1 is not None else time() - 1.0
            curr_time = time()

            if curr_time - last_detection_time > 0.5:
                # var2 holds the list of past detections, reset 
                self.routines[DETECT_STOP_LINE].var2 = past_detections = deque(maxlen=DETECTION_DEQUE_LENGTH)
            
            assert past_detections is not None, 'past_detections is None, wrong initialization'

            adapted_distance = dist + self.car.encoder_distance #- 0.4 #we substract the car length = 0.4

            past_detections.append(adapted_distance)
            self.stop_line_distance_median = np.mean(past_detections) if len(past_detections) > SAMPLE_BEFORE_CONFIDENCE else None
            self.routines[DETECT_STOP_LINE].var1 = curr_time
        else:
            self.stop_line_distance_median = None 

    def slow_down(self):
        if np.abs(self.car.filtered_encoder_velocity - SLOW_DOWN_CONST*self.desired_speed) > 0.1:
            self.car.drive_speed(self.desired_speed*SLOW_DOWN_CONST)

    def accelerate(self):
        if self.car.filtered_encoder_velocity < ACCELERATION_CONST*self.desired_speed:
            self.car.drive_speed(ACCELERATION_CONST*self.desired_speed)

    def control_for_signs(self):
        if SPEED_CHALLENGE: return
        if not self.conditions[REROUTING]:
            if self.conditions[TRUST_GPS]:
                car_pos_on_path = self.path_planner.path[int(round(self.car_dist_on_path*100))]
                distances = norm(self.sign_points-car_pos_on_path, axis=1)
                
                print(f'MIN DISTANCE = {np.min(distances)}')
                idx_close_signs = np.where(distances < SIGN_DIST_THRESHOLD)[0]
                if len(idx_close_signs) > 0:
                    for i in idx_close_signs:
                        if self.sign_seen[i] == 0:
                            self.sign_seen[i] = 1
                            print(f'SEEN SIGN {SIGN_NAMES[self.sign_types[i]]}, at pos {self.sign_points[i]}')
                            self.curr_sign = SIGN_NAMES[self.sign_types[i]]
                            #PUB SIGN TODO
                else:
                    self.curr_sign = NO_SIGN

            else: #Use signs
                sign, _ = self.detect.detect_sign(self.car.frame, show_ROI=SHOW_IMGS, show_kp=SHOW_IMGS)
                if sign != NO_SIGN and sign !=self.curr_sign:
                    self.curr_sign = sign

    def control_for_semaphore(self):
        pass

    def control_for_pedestrians(self):
        pass

    def control_for_vehicles(self):
        pass

    def control_for_roadblocks(self):
        pass

    def control_for_obstacles(self):
        #check for obstacles
        if not SPEED_CHALLENGE:
            last_obstacle_dist = self.routines[CONTROL_FOR_OBSTACLES].var1 if self.routines[CONTROL_FOR_OBSTACLES].var1 is not None else self.car.encoder_distance - 1.0
            curr_dist = self.car.encoder_distance
            if curr_dist - last_obstacle_dist > MIN_DIST_BETWEEN_OBSTACLES:
                dist = self.car.filtered_sonar_distance
                if dist < OBSTACLE_DISTANCE_THRESHOLD + 0.1:
                    self.car.drive_speed(self.desired_speed*0.2)
                if dist < OBSTACLE_DISTANCE_THRESHOLD:
                    self.switch_to_state(CLASSIFYING_OBSTACLE)
                    self.routines[CONTROL_FOR_OBSTACLES].var1 = curr_dist



    # STATE CHECKS
    def check_logic(self):
        if not self.conditions[CAR_ON_PATH]:
            print(f'ERROR: CHECKS: Car is not on path')
            self.car.stop()
            while True and SHOW_IMGS:
                if cv.waitKey(10) == 27:
                    break
            exit()

    # UPDATE CONDITIONS
    def update_state(self):
        """"
        This will update the conditions at every iteration, it is called at the end of a self.run
        """
        #mirror trust gps from automobile_data
        self.conditions[TRUST_GPS] = self.car.trust_gps and not ALWAYS_DISTRUST_GPS or ALWAYS_TRUST_GPS
        
        if self.conditions[TRUST_GPS]:

            est_pos = np.array([self.car.x_est, self.car.y_est])
            closest_node, distance = self.path_planner.get_closest_node(est_pos)

            #HIGHWAY
            self.conditions[HIGHWAY] = closest_node in self.path_planner.highway_nodes

            #OVERTAKE/DOTTED_LINE
            self.conditions[CAN_OVERTAKE] = self.path_planner.is_dotted(closest_node) 

            #REROUTING updated in start state

            if not self.conditions[REROUTING]:
                #CAR_ON_PATH
                path = self.path_planner.path
                diff = norm(est_pos - path, axis=1)
                min_diff = np.min(diff)
                if min_diff > MAX_DIST_AWAY_FROM_LANE: #TODO #IMPLEMENT THIS CASE
                    self.conditions[CAR_ON_PATH] = False
                else:
                    self.conditions[CAR_ON_PATH] = True

                #UPDATING CAR PATH INDEX
                if self.conditions[CAR_ON_PATH]:
                    # print(f'total path: {len(path)} \n{self.path_planner.path}')
                    # print(f'est_pos: {est_pos}')

                    self.car_dist_on_path = np.argmin(norm(self.path_planner.path - est_pos, axis=1))*0.01 #NOTE assume step length is 0.01 #NOTE 2: Assumes no self loops in the path 
                    print(f'car_dist_on_path: {self.car_dist_on_path:.2f} [m]')

            





    #===================== STATE MACHINE MANAGEMENT =====================#
    def run(self):
        print(f'CURR_SIGN: {self.curr_sign}')
        print('==========================================================================')
        print(f'STATE:          {self.curr_state}')
        print(f'UPCOMING_EVENT: {self.next_event}')
        print(f'ROUTINES:       {self.active_routines_names+ALWAYS_ON_ROUTINES}')
        print(f'CONDITIONS:     {self.conditions}')
        print('==========================================================================')
        self.run_current_state()
        print('==========================================================================')
        print()
        self.run_routines()
        print('==========================================================================')
        print()
        self.check_logic()

    def run_current_state(self):
        self.curr_state.run()

    def run_routines(self):
        for k, r in self.routines.items():
            if r.active: r.run()
        
        for k in ALWAYS_ON_ROUTINES:
            self.routines[k].run()

    def activate_routines(self, routines_to_activate):
        """
        routines_to_activate are a list of strings (routines)
        ex: ['follow_lane', 'control_for_signs']
        """
        assert all([r in self.routines.keys() for r in routines_to_activate]), 'ERROR: activate_routines: routines_to_activate contains invalid routine'
        self.active_routines_names = []
        for k,r in self.routines.items():
            r.active = k in routines_to_activate
            if r.active: self.active_routines_names.append(k)
    
    def add_routines(self, routines):
        """
        add routines to he active routines without overwriting the other
        ex: ['follow_lane', 'control_for_signs']
        """    
        assert all([r in self.routines.keys() for r in routines]), 'ERROR: add_routines: routines_to_activate contains invalid routine'
        for k in routines:
            self.routines[k].active = True

    def switch_to_state(self, to_state, interrupt=False):
        """
        to_state is the string of the desired state to switch to
        ex: 'lane_following'
        """
        assert to_state in self.states, f'{to_state} is not a valid state'
        self.prev_state = self.curr_state
        self.curr_state = self.states[to_state]
        for k,s in self.states.items():
            s.active = k == to_state
        if not interrupt:
            self.curr_state.start_time = time()
            self.curr_state.start_position = np.array([self.car.x_est, self.car.y_est]) ######## maybe another position
            self.curr_state.start_distance = self.car.encoder_distance
            self.curr_state.interrupted = False
        else:
            self.curr_state.interrupted = True
        self.curr_state.just_switched = True

    def switch_to_prev_state(self):
        self.switch_to_state(self.prev_state.name)
    
    def go_to_next_event(self):
        """
        Switches to the next event on the path
        """
        self.prev_event = self.next_event
        if self.event_idx == len(self.events):
            #no more events, for now
            pass
        else:
            self.next_event = self.events[self.event_idx]
            self.event_idx += 1
    
    def next_checkpoint(self):
        self.checkpoint_idx += 1
        if self.checkpoint_idx < (len(self.checkpoints)-1): #check if it's last
            #update events
            self.prev_event = self.next_event#deepcopy(self.next_event)
            pass
        else: 
            #it was the last checkpoint
            print('Reached last checkpoint...\nExiting...')
            cv.destroyAllWindows() if SHOW_IMGS else None
            exit()

    def create_sequence_of_events(self, events):
        """
        events is a list of strings (events)
        ex: ['lane_following', 'control_for_signs']
        """
        to_ret = []
        for e in events:
            name = e[0]
            dist = e[1]
            point = e[2]
            path_ahead = e[3] #path in global coordinates
            if path_ahead is not None:
                loc_path = path_ahead - point
                #get yaw of the stopline
                assert path_ahead.shape[0] > 10, f'path_ahead is too short: {path_ahead.shape[0]}'
                path_first_10 = path_ahead[:10]
                diff10 = path_first_10[1:] - path_first_10[:-1]
                yaw_raw = np.median(np.arctan2(diff10[:,1], diff10[:,0]))
                yaw_stopline = get_yaw_closest_axis(yaw_raw)
                rot_matrix = np.array([[np.cos(yaw_stopline), -np.sin(yaw_stopline)],
                                        [np.sin(yaw_stopline), np.cos(yaw_stopline)]])
                loc_path = loc_path @ rot_matrix
                path_to_ret = loc_path
                curv = get_curvature(path_ahead)
                print(f'yaw_stopline: {yaw_stopline}, name: {name}, curv: {curv}')
                len_path_ahead = 0.01*len(path_ahead)
            else:
                path_to_ret = None
                curv = None
                yaw_stopline = None
                len_path_ahead = None

            if name == HIGHWAY_EXIT_EVENT and curv > 0.05 : #going straight is ~0.15, right is ~ -0.15
                pass #skip this case
            else:
                event = Event(name, dist, point, yaw_stopline, path_to_ret, len_path_ahead, curv)
                to_ret.append(event)
        #add end of path event
        # ee_dist = (len(self.path_planner.path) - 1 )*0.01
        ee_point = self.path_planner.path[-1]
        end_event = Event(END_EVENT, dist=0.0, point=ee_point)
        to_ret.append(end_event)
        return to_ret

    