from collections import namedtuple
from math import sqrt, exp

TrajectoryData = namedtuple(
    "TrajectoryData", [
        'intended_lane',
        'final_lane',
        'end_distance_to_goal',
    ]
)

'''
Provided here are templates for two possible suggestions for cost functions,
but other metrics can be used as long as they represent the cost for choosing
a specific behaviour.
The weighted cost over all cost functions is computed in calculate_cost.
Cost calculation of each cost function is based on the trajectory data
whose format is defined above, and the trajectory data values are
computed by get_helper_data function.
'''

# weights for costs
REACH_GOAL = 0
EFFICIENCY = 0

DEBUG = False

def goal_distance_cost(vehicle, trajectory, predictions, data):
    '''
    Cost increases based on distance of intended lane (for planning a
    lane change) and final lane of a trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches
    the goal distance.
    '''
    c = 0
    dist = data[2] 

    if dist > 0 :
        delta_d = 2.0 * vehicle.goal_lane - data[0] - data[1]

        c = 1 - 2 * exp(-(abs(delta_d) / dist))
    else: 
        c = 1
    
    return c
    # return 0

def inefficiency_cost(vehicle, trajectory, predictions, data):
    '''
    Cost becomes higher for trajectories with intended lane and final lane
    that have slower traffic.
    '''
    def ahead_speed(predictions,lane,vehicle):

        min_s          = vehicle.goal_s
        found_vehicle = False
        speed        = 0.0

        temp_vehicle = None

        for k,v in predictions.items():
            
            if len(v) == 0: 
                continue;

            temp_vehicle = v[1];

            if temp_vehicle.lane == lane and temp_vehicle.s > vehicle.s and temp_vehicle.s < min_s:
                min_s         = temp_vehicle.s;
                speed         = temp_vehicle.v;
                found_vehicle = True;
               
        

        shortest_dst = temp_vehicle.s - vehicle.s;

        if found_vehicle and (shortest_dst < 40) : 
            return speed;

        else : 
            return -1.0;

    ahead_speed_intended = ahead_speed(predictions,data[0],vehicle)

    if ahead_speed_intended < 0 :
        ahead_speed_intended = vehicle.target_speed

    ahead_speed_final = ahead_speed(predictions, data[1], vehicle);

    if ahead_speed_final < 0:
        ahead_speed_final= vehicle.target_speed
    
    c = (2.0 * vehicle.target_speed - ahead_speed_intended - ahead_speed_final)/vehicle.target_speed;
    
    # return 0
    return c

def calculate_cost(vehicle, trajectory, predictions):
    '''
    Sum weighted cost functions to get total cost for trajectory.
    '''
    trajectory_data = get_helper_data(vehicle, trajectory, predictions)
    cost = 0.0
    # list of cost functions and their associated costs
    cf_list = [goal_distance_cost, inefficiency_cost]
    weight_list = [REACH_GOAL, EFFICIENCY]

    for weight, cf in zip(weight_list, cf_list):
        new_cost = weight \
                   * cf(vehicle, trajectory, predictions, trajectory_data)
        if DEBUG:
            print(
                "%s has cost %.1f for lane %d" % \
                (cf.__name__, new_cost, trajectory[-1].lane)
            )
        cost += new_cost
    return cost

def get_helper_data(vehicle, trajectory, predictions):
    '''
    Generate helper data to use in cost functions:
    indended_lane:  +/-1 from the current lane if the vehicle is planning
                    or executing a lane change.
    final_lane: The lane of the vehicle at the end of the trajectory.
                The lane is unchanged for KL and LCL/LCR trajectories.
    distance_to_goal: The s distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help
    differentiate between planning and executing a lane change
    in the cost functions.
    '''

    last = trajectory[1]

    if last.state == "PLCL":
        intended_lane = last.lane + 1
    elif last.state == "PLCR":
        intended_lane = last.lane - 1
    else:
        intended_lane = last.lane

    distance_to_goal = vehicle.goal_s - last.s
    final_lane = last.lane

    return TrajectoryData(
        intended_lane,
        final_lane,
        distance_to_goal
    )

def velocity(predictions, lane):
    '''
    Get the velocity (speed) flowing in the specified lane.
    All non-ego vehicles in a lane have the same speed,
    so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    '''
    for v_id, predicted_trajectory in predictions.items():
        if predicted_trajectory[0].lane == lane and v_id != -1:
            return predicted_trajectory[0].v
