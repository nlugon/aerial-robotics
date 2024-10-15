 # Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt

# Global variables
on_ground = True
height_desired = 0.5

# Obstacle avoidance with range sensors
def obstacle_avoidance(sensor_data):
    global on_ground, height_desired

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Obstacle avoidance with distance sensors
    if sensor_data['range_front'] < 0.2:
        if sensor_data['range_left'] > sensor_data['range_right']:
            control_command = [0.0, 0.2, 0.0, height_desired]
        else:
            control_command = [0.0, -0.2, 0.0, height_desired]
    else:
        control_command = [0.2, 0.0, 0.0, height_desired]

    return control_command
    
# Better Obstacle avoidance with range sensors
def obstacle_avoidance2(sensor_data):
    global on_ground, height_desired

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Obstacle avoidance with distance sensors
    if sensor_data['range_front'] < 0.2:
        if sensor_data['range_left'] > sensor_data['range_right']:
            control_command = [0.05, 0.1, 0.2, height_desired]
        else:
            control_command = [0.05, -0.1, -0.2, height_desired]
    else:
        control_command = [0.2, 0.0, 0.0, height_desired]

    return control_command
    
    
    
    
    
# Wall following with range sensors
# Wall following with range sensors and P controller
def wall_following2(sensor_data):
    global on_ground, height_desired

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Wall following with distance sensors
    distance_error = 0.6 - sensor_data['range_left'] # distance error from desired distance
    proportional_term = 0.5 * distance_error # P term of controller

    if sensor_data['range_front'] < 0.2:
        # Rotate on itself when detecting the wall
        control_command = [0.0, 0.0, 0.5, height_desired] # 0.5 rad/s yaw rate
    else:
        if sensor_data['range_left'] < 0.5: # Too close to the wall on the left
            control_command = [0.2, 0.0, 0.5 + proportional_term, height_desired] # Move forward and turn right
        elif sensor_data['range_left'] > 0.7: # Too far from the wall on the left
            control_command = [0.2, 0.0, -0.5 - proportional_term, height_desired] # Move forward and turn left
        else: # Maintain distance to the wall on the left
            control_command = [0.2, 0.0, 0.0 + proportional_term, height_desired] # Move forward

    return control_command



import math

# Wall following with P controller
def wall_following(sensor_data):
    global on_ground, height_desired, distance_from_wall, k_p
    distance_from_wall = 0.3
    k_p = 0.5

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Wall following with distance sensors
    range_front = sensor_data['range_front']
    range_left = sensor_data['range_left']
    range_right = sensor_data['range_right']

    # Calculating height of triangle if the angle between sides is known
    beta = math.radians(60)
    height = (range_left * range_front * math.sin(beta)) / math.sqrt(math.pow(range_left, 2) + math.pow(range_front, 2) - 2 * range_left * range_front * math.cos(beta))

    # This is to compare the range side with to keep it perpendicular to the wall
    range_equal_to = range_front * math.cos(math.radians(60))

    # Most important, first check if the robot is away from the wall!
    if abs(height - distance_from_wall) < 0.1:
        # If so, just try to align to the wall by changing the angle
        if range_left > range_equal_to - 0.05 and range_front != 0.0:
            yaw_rate = -0.15
        elif range_left < range_equal_to + 0.05 and range_front != 0.0:
            yaw_rate = 0.15
        else:
            yaw_rate = 0.0
    else:
        # If not, increase or decrease the distance by changing the heading
        error = distance_from_wall - height
        yaw_rate = k_p * error

    control_command = [0.2, 0.0, yaw_rate, height_desired]

    return control_command



    
    
    
    
    
    
    
    
# Bug2 Algorithm
setpoints = [[4.5, 0.5], [4.5, 2.5], [3.5, 0.5], [3.5, 2.5]]
index_current_setpoint = 0
def bug2_obstacle_avoidance(sensor_data):
    global on_ground, height_desired, mode, state, hit_point, distance_to_goal
    
    # initialize variables
    state = "INIT"
    hit_point = None
    distance_to_goal = None
        
    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False
        
    # Get current position and goal
    x_goal, y_goal = setpoints[index_current_setpoint]
    x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
    
    # Calculate distance to goal
    distance_to_goal = ((x_goal - x_drone)**2 + (y_goal - y_drone)**2)**0.5
    
    # Implement Bug2 algorithm
    if state == "INIT":
        # move towards goal
        v_x, v_y = x_goal - x_drone, y_goal - y_drone
        control_command = [v_x, v_y, 0.0, height_desired]
        
        if sensor_data['range_front'] < 0.2:
            # switch to following obstacle mode
            state = "FOLLOW_OBSTACLE"
            hit_point = (x_drone, y_drone)
            print(f"New hit point : {hit_point}\n")
            distance_to_hit_point = ((x_goal - x_drone)**2 + (y_goal - y_drone)**2)**0.5
            mode = "LEFT" if sensor_data['range_left'] > sensor_data['range_right'] else "RIGHT"
            
    elif state == "FOLLOW_OBSTACLE":
        if sensor_data['range_front'] < 0.2:
            if mode == "LEFT":
                control_command = [0.0, 0.2, 0.0, height_desired]
            else:
                control_command = [0.0, -0.2, 0.0, height_desired]
        else:
            control_command = [0.2, 0.0, 0.0, height_desired]
        
        # calculate distance to hit point
        distance_to_hit_point = ((x_drone - hit_point[0])**2 + (y_drone - hit_point[1])**2)**0.5
        
        if sensor_data['range_front'] > 0.3:
            # back to initial mode
            state = "INIT"
            control_command = [v_x, v_y, 0.0, height_desired]
        elif distance_to_hit_point > distance_to_goal:
            # drone passed the obstacle, back to initial mode
            state = "INIT"
            control_command = [v_x, v_y, 0.0, height_desired]
        elif sensor_data['range_left'] > 0.5 and sensor_data['range_right'] > 0.5:
            # obstacle cleared, back to initial mode
            state = "INIT"
            control_command = [v_x, v_y, 0.0, height_desired]
        else:
            # continue following obstacle
            if mode == "LEFT":
                control_command = [0.0, 0.2, 0.0, height_desired]
            else:
                control_command = [0.0, -0.2, 0.0, height_desired]

# ----------------------------
# Coverage path planning
setpoints = [[-0.0, 0.0], [-0.0, -2.0], [-0.5, -2.0], [-0.5, 0.0]]
index_current_setpoint = 0
def path_planning(sensor_data):
    global on_ground, height_desired, index_current_setpoint, setpoints

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Hover at the final setpoint
    if index_current_setpoint == len(setpoints):
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command

    # Get the goal position and drone position
    x_goal, y_goal = setpoints[index_current_setpoint]
    x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
    distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(setpoints):
            control_command = [0.0, 0.0, 0.0, height_desired]
            return control_command

    # Calculate the control command based on current goal setpoint
    x_goal, y_goal = setpoints[index_current_setpoint]
    x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
    v_x, v_y = x_goal - x_drone, y_goal - y_drone
    control_command = [v_x, v_y, 0.0, height_desired]
    return control_command
    
# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 5.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.2 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the point is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
    t +=1

    return map