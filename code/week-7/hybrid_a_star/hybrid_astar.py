import numpy as np

class HybridAStar:
    # Determine how many grid cells to have for theta-axis.
    NUM_THETA_CELLS = 90

    # Define min, max, and resolution of steering angles
    omega_min = -35
    omega_max = 35
    omega_step = 5

    # A very simple bicycle model
    speed = 1.0
    length = 0.5

    # Initialize the search structure.
    def __init__(self, dim):
        self.dim = dim
        self.closed = np.zeros(self.dim, dtype=np.int)
        self.came_from = np.full(self.dim, None)
    
    # Expand from a given state by enumerating reachable states.
    def expand(self, current, goal):
        g = current['g']
        x, y, theta = current['x'], current['y'], current['t']

        # The g value of a newly expanded cell increases by 1 from the
        # previously expanded cell.
        g2 = g + 1
        next_states = []
        import math
        # Consider a discrete selection of steering angles.
        for delta_t in range(-35,40,5):#[]:
            d = math.pi / 180.0 * delta_t
            o = self.speed / self.length * math.tan(d)
            theta2 = theta + o
            if theta2 < 0:
                theta2 += 2*math.pi
            # pass
            x2 = x + self.speed * math.cos(theta)
            y2 = y + self.speed * math.sin(theta)
            s2 = {}
            s2['f'] = g2 + self.heuristic(x2,y2,goal)
            s2['g'] = g2
            s2['x'] = x2
            s2['y'] = y2
            s2['t'] = theta2
            next_states.append(s2)
            # TODO: implement the trajectory generation based on
            # a simple bicycle model.
            # Let theta2 be the vehicle's heading (in radian)
            # between 0 and 2 * PI.
            # Check validity and then add to the next_states list.

        return next_states

    # Perform a breadth-first search based on the Hybrid A* algorithm.
    def search(self, grid, start, goal):
        # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.
        while len(opened) > 0:
            # TODO: implement prioritized breadth-first search
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for n in next_states:
                # pass
                g2 = n['g']
                x2 = n['x']
                y2 = n['y']
                theta2 = n['t']

                if (x2 < 0 or x2 >= len(grid)) or (y2 < 0 or y2 >= len(grid[0])):
                    continue

                st2 = self.theta_to_stack_num(theta2)

                if self.closed[st2][self.idx(x2)][self.idx(y2)] == 0 and grid[self.idx(x2)][self.idx(y2)] == 0:
                    opened.append(n)
                    self.closed[st2][self.idx(x2)][self.idx(y2)] = 1
                    self.came_from[st2][self.idx(x2)][self.idx(y2)] = curr
                    total_closed += 1
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed

    # Calculate the stack index of a state based on the vehicle's heading.
    def theta_to_stack_num(self, theta):
        # TODO: implement a function that calculate the stack number
        # given theta represented in radian. Note that the calculation
        # should partition 360 degrees (2 * PI rad) into different
        # cells whose number is given by NUM_THETA_CELLS.
        import math
        theta_ = math.fmod((theta + 2 * math.pi),2 * math.pi)
        sn = int((round(theta_ * self.NUM_THETA_CELLS / (2*math.pi))) % self.NUM_THETA_CELLS);
        return sn

    # Calculate the index of the grid cell based on the vehicle's position.
    def idx(self, pos):
        # We simply assume that each of the grid cell is the size 1 X 1.
        return int(np.floor(pos))

    # Implement a heuristic function to be used in the hybrid A* algorithm.
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        import math
        return math.fabs(y - goal[0]) + math.fabs(x-goal[1])

    # Reconstruct the path taken by the hybrid A* algorithm.
    def reconstruct_path(self, start, goal):
        # Start from the final state, and follow the link to the
        # previous state using the came_from matrix.
        curr = self.final
        x, y = curr['x'], curr['y']
        path = []
        while x != start[0] and y != start[1]:
            path.append(curr)
            stack = self.theta_to_stack_num(curr['t'])
            x, y = curr['x'], curr['y']
            curr = self.came_from[stack][self.idx(x)][self.idx(y)]
        # Reverse the path so that it begins at the starting state
        # and ends at the final state.
        path.reverse()
        return path
