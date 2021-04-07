import numpy as np
import itertools

# Given map
grid = np.array([
    [1, 1, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1]
])

# List of possible actions defined in terms of changes in
# the coordinates (y, x)
forward = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

# Three actions are defined:
# - right turn & move forward
# - straight forward
# - left turn & move forward
# Note that each action transforms the orientation along the
# forward array defined above.
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

init = (4, 3, 0)    # Representing (y, x, o), where
                    # o denotes the orientation as follows:
                    # 0: up
                    # 1: left
                    # 2: down
                    # 3: right
                    # Note that this order corresponds to forward above.
goal = (2, 0)
cost = (2, 1, 20)   # Cost for each action (right, straight, left)

# EXAMPLE OUTPUT:
# calling optimum_policy_2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]

def optimum_policy_2D(grid, init, goal, cost):
    # Initialize the value function with (infeasibly) high costs.
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    # Initialize the policy function with negative (unused) values.
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    # Final path policy will be in 2D, instead of 3D.
    policy2D = np.full(grid.shape, ' ')

    # Apply dynamic programming with the flag change.
    change = True
    while change:
        change = False
        # This will provide a useful iterator for the state space.
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        # Compute the value function for each state and
        # update policy function accordingly.
        for y, x, t in p:

            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
                # TODO: implement code.
                change = True
                value[(t,y,x)] = 0
                policy[(t,y,x)] = -9999
                # pass
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:
                # TODO: implement code.
                # pass
               for i in range(3):
                   o2 = (t + action[i]) % 4
                   x2 = x + forward[o2][1]
                   y2 = y + forward[o2][0]

                   if x2 >= 0 and x2 < grid.shape[1] and y2 >= 0 and y2 < grid.shape[0] and grid[(y2, x2)] == 0:
                        # print(value[(o2,x2,y2)])
                        # print((o2,x2,y2))
                        # print(value.shape)
                        # print(i)
                        # print(cost[i])

                        # print('------------')
                        v2 = value[(o2,y2,x2)] + cost[i]
                       
                        if v2 < value[(t,y,x)]:
                            value[(t,y,x)] = v2
                            policy[(t,y,x)] = action[i]#action_name[i]
                            change = True
    x = init[1]
    y = init[0]
    orientation = init[2]
    
    policy_st = policy[(orientation,y,x)]
    # policy_name = -1
    # print(policy_temp)
    if policy_st == action[0]:
        policy_name_st = action_name[0]
    elif policy_st == action[1]:
        policy_name_st = action_name[1]
    elif policy_st == action[2]:
        policy_name_st = action_name[2]
    elif policy_st == -9999:
        policy_name_st = "*"

    policy2D[(y,x)] = policy_name_st
    while policy[(orientation,y,x)] != -9999:
        if policy[(orientation,y,x)] == action[1]:
            o2 = orientation
        elif policy[(orientation,y,x)] == action[0]:
            o2 = (orientation - 1) % 4
        elif policy[(orientation,y,x)] == action[2]:
            o2 = (orientation + 1) % 4
        x = x + forward[o2][1]
        y = y + forward[o2][0]

        orientation = o2
        
        policy_temp = policy[(orientation,y,x)]
        # policy_name = -1
        # print(policy_temp)
        if policy_temp == action[0]:
            policy_name = action_name[0]
        elif policy_temp == action[1]:
            policy_name = action_name[1]
            # print(policy_temp)
        elif policy_temp == action[2]:
            policy_name = action_name[2]
        elif policy_temp == -9999:
            policy_name = "*"
        # print(policy_name)
        # print(policy_temp)
        # print("---------")
        policy2D[(y,x)] = policy_name 
        # print(policy)
    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
    # TODO: implement code.

    # Return the optimum policy generated above.
    # print(value)
    return policy2D

print(optimum_policy_2D(grid, init, goal, cost))
