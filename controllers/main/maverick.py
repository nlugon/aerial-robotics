# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np
import matplotlib.pyplot as plt

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.acquired_home_position = False
        self.on_ground = True
        self.return_home = False
        
        self.height_desired = 0.5
        self.x_home = 0.0
        self.y_home = 0.0
        self.setpoints = [(4.6, 1.0), (4.4, 2.7), (4.1, 2.7), (4.0, 1.0), (4.8, 0.6), (4.6, 0.2), (3.6, 0.2), (3.4, 2.9), (4.6, 1.4)]
        # self.setpoints = [[4.8, 0.2], [3.6, 2.8], [4.8, 2.8], [3.6, 2.2], [4.8, 2.2], [3.6, 1.6], [4.8, 1.6], [3.6, 1.0], [4.8, 1.0], [3.6, 0.4], [4.8, 0.4], [3.6, 0.1]]
        self.index_current_setpoint = 0

        # Path
        self.path = []
        self.index_current_Astar = 1
        
        self.state = "TAKEOFF" # States: "TAKEOFF", "FORWARD", "OBSTACLE", "LAND"
        self.mode = "FINDTARGET" # "FINDTARGET", "GOBACKHOME", "DONE"
        self.direction = "Xaxis"
        self.MAX_SPEED = 0.2
        self.PLATFORM_DIST = 0.44
        self.random_sign = np.random.choice([-1, 1])

        self.start_hover = 0.0
        self.sensor_data_list = []
        self.min_index = 0
        
       
        
        # MAP ATTRIBUTES
        self.min_x, self.max_x = 0, 5.0 # meter
        self.min_y, self.max_y = 0, 3.0 # meter
        self.range_max = 2.0 # meter, maximum range of distance sensor
        self.res_pos = 0.2 # meter
        self.conf = 0.2 # certainty given by each measurement
        self.t = 0
        
        self.current_time = 0.0
        self.previous_time = 0.0
        self.period = 0.1

        self.Dt = 0 # only for plotting

        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
        self.map_ground = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
        self.nav_map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos)))
        
        
        
        

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
    
    
        self.update_map(sensor_data)

        if self.index_current_setpoint >= len(self.setpoints):
            self.index_current_setpoint = 0
            
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_home = np.linalg.norm([self.x_home - x_drone, self.y_home - y_drone])
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        print(f"Setpoints : {self.setpoints}\n")
        print(f"Distance to setpoint {self.index_current_setpoint} : {distance_drone_to_goal:.2f}\n")

        print(f"State : {self.state}\n")
        print(f"Mode : {self.mode}\n")
        print(f"Direction : {self.direction}\n")
        print(f"Elevation : {sensor_data['range_down']:.2f} AGL")
      
        
        
        
        
        
        
        
        if self.state == "TAKEOFF":
            self.height_desired = 0.5
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            # LIFT UPWARDS UNTIL CERTAIN ALTITUDE
            if sensor_data['range_down'] < 0.49:
            
                # if not self.acquired_home_position:
                    # self.x_home, self.y_home = x_drone, y_drone
                    # print(f"x_home :{self.x_home}\n")
                    # print(f"y_home : {self.y_home}\n")
                    # self.acquired_home_position = True
                # else:
                    # pass
                
                return control_command

                
            # ONCE UP, TRANSITION TO NEXT STATE
            else:
                if not self.acquired_home_position:
                    self.x_home, self.y_home = x_drone, y_drone
                    print(f"x_home :{self.x_home:.2f}\n")
                    print(f"y_home : {self.y_home:.2f}\n")
                    self.acquired_home_position = True
                else:
                    pass
                    
                if self.mode == "FINDTARGET":
                    pass
                elif self.mode == "GOBACKHOME":
                    self.index_current_setpoint = 0
                    self.setpoints = [[self.x_home, self.y_home]]
                else:
                    pass
                    
                self.state = "FORWARD"
                return control_command
            

        elif self.state == "HOVER":
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                time_diff = abs(self.start_hover - sensor_data['t'])
                if time_diff > 0.5:
                    self.state = "FORWARD"
                    
                    return control_command
                return control_command
                
                
                
            
            
            
            
                
        elif self.state == "FORWARD":
        
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            
            
            # IF OBSTACLE DETECTED :
            if self.direction == "Xaxis":
                if sensor_data['range_front'] < 0.25 or sensor_data['range_back'] < 0.25 or sensor_data['range_left'] < 0.08 or sensor_data['range_right'] < 0.08:
                    self.sensor_data_list = [sensor_data['range_front'], sensor_data['range_left'], sensor_data['range_right'], sensor_data['range_back']]
                    self.min_index = self.sensor_data_list.index(min(self.sensor_data_list))

                    self.random_sign = np.random.choice([-1, 1])
                    self.state = "OBSTACLE"
                    return control_command
                else:
                    pass
            elif self.direction == "Yaxis":
                if sensor_data['range_front'] < 0.08 or sensor_data['range_back'] < 0.08 or sensor_data['range_left'] < 0.25 or sensor_data['range_right'] < 0.25:
                    self.sensor_data_list = [sensor_data['range_front'], sensor_data['range_left'], sensor_data['range_right'], sensor_data['range_back']]
                    self.min_index = self.sensor_data_list.index(min(self.sensor_data_list))

                    self.random_sign = np.random.choice([-1, 1])
                    self.state = "OBSTACLE"
                    return control_command
                else:
                    pass
            else:
                pass

                
            # IF PLATFORM BELOW DETECTED : 
            if sensor_data['range_down'] < self.PLATFORM_DIST:
                if self.mode == "GOBACKHOME":
                    print(f"Distance drone to home : {distance_drone_to_home:.2f}\n")
                    if distance_drone_to_home < 0.1:
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        self.state = "LAND"
                        return control_command
                    
                elif self.mode == "FINDTARGET":
                    if sensor_data['x_global'] >= 3.5:
                        self.state = "LAND"
                        return control_command
                    else:
                        print("Unexpected landing platform detected")
                            
                    
                # elif self.mode == "GOBACKHOME":
                    # print(f"elif {distance_drone_to_home}\n")
                    # if distance_drone_to_home < 0.1:
                        # print("bla\n")
                        # self.state = "LAND"
                        # return control_command
                    # else:
                        # pass
                else:
                    self.state = "LAND"
                    control_command = [0.0, 0.0, 0.0, self.height_desired]
                    return control_command
                    
            else:
                pass
                
            control_command = self.path_planning(sensor_data)
            return control_command
                    

                
            
            
            
            
            
            
            
            
        elif self.state == "OBSTACLE":

            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
            scaled_forward_velocity, scaled_left_velocity = self.scale_speed(x_goal - x_drone, y_goal - y_drone)
        
            # COMMENT THIS ONCE MAPPING IMPLEMENTED --------
            if distance_drone_to_goal < 0.3 and self.mode == "FINDTARGET":
                if self.index_current_setpoint == len(self.setpoints):
                    self.index_current_setpoint = 0
                else:
                    self.index_current_setpoint += 1
            # COMMENT THIS ONCE MAPPING IMPLEMENTED --------
        
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.sensor_data_list = [sensor_data['range_front'], sensor_data['range_left'], sensor_data['range_right'], sensor_data['range_back']]
            
            
            
            # if sensor_data['range_front'] < 0.3:
            #     if sensor_data['range_left'] >= 0.3 and sensor_data['range_right'] >= 0.3:
            #         control_command = [-0.05, 0.25, 0.0, self.height_desired]
            #         return control_command
            #     else:
            #         if sensor_data['range_left'] >= sensor_data['range_right']:
            #             control_command = [-0.05, 0.25, 0.0, self.height_desired]
            #             return control_command
            #         else:
            #             control_command = [-0.05, -0.25, 0.0, self.height_desired]
            #             return control_command
                    
            # elif sensor_data['range_left'] < 0.2 or sensor_data['range_right'] < 0.2:
            #     if sensor_data['range_left'] >= sensor_data['range_right']:
            #         control_command = [0.0, 0.2, 0.0, self.height_desired]
            #         return control_command
            #     else:
            #         control_command = [0.0, -0.2, 0.0, self.height_desired]
            #         return control_command
            # else: 
            #     control_command = [0.0, 0.0, 0.0, self.height_desired]
            #     self.state = "HOVER"
            #     return control_command

            if self.sensor_data_list[self.min_index] < 0.35:

                if self.min_index == 0:
                    control_command = [-0.08, 0.3*np.sign(scaled_left_velocity)*self.random_sign, 0.0, self.height_desired]
                    return control_command
                elif self.min_index == 1:
                    control_command = [0.3*np.sign(scaled_forward_velocity)*self.random_sign, -0.08, 0.0, self.height_desired]
                    return control_command
                elif self.min_index == 2:
                    control_command = [0.3*np.sign(scaled_forward_velocity)*self.random_sign, 0.08, 0.0, self.height_desired] 
                    return control_command
                elif self.min_index == 3:
                    control_command = [0.08, 0.3*np.sign(scaled_left_velocity)*self.random_sign, 0.0, self.height_desired]
                    return control_command
                else:
                    pass
            else:
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                self.state = "HOVER"
                
                
            return control_command
            
            
            
            
            
            
            
            
            
            
        elif self.state == "LAND":
        
            if self.height_desired <= 0.0:
                self.height_desired = 0.0
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                if self.mode == "FINDTARGET":
                    self.mode = "GOBACKHOME"
                    self.state = "TAKEOFF"
                elif self.mode == "GOBACKHOME":
                    self.mode = "DONE"
                elif self.mode == "DONE":
                    pass
                else:
                    self.mode = "GOBACKHOME"
                
                return control_command
            else:
                pass
        
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
            
            
            
            
        else:
            print(f"WARNING : IN UNDEFINED STATE")
            self.state = "FORWARD"
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
            

            
            
        

                    


            
    def world_to_grid(self, x, y):
        idx_x = int(np.round((x - self.min_x)/self.res_pos,0))
        idx_y = int(np.round((y - self.min_y)/self.res_pos,0))
        return idx_x, idx_y
    
    def grid_to_world(self, idx_x, idx_y):
        x = self.min_x + idx_x*self.res_pos
        y = self.min_y + idx_y*self.res_pos
        return x, y        
 
        
        
        
        
        

    

    def path_planning(self, sensor_data):

        # Get the goal position and drone position
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        print(f"Distance to setpoint {self.index_current_setpoint:.2f} : {distance_drone_to_goal:.2f}")
        

        if distance_drone_to_goal < 0.1:
            # Select the next setpoint as the goal position
            if self.mode == "FINDTARGET":
                if self.index_current_setpoint == len(self.setpoints):
                    self.index_current_setpoint = 0
                else:
                    self.index_current_setpoint += 1
                    
            elif self.mode == "GOBACKHOME":
                self.state = "LAND"
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command
                
            else:
                pass
            # Hover at the final setpoint
            if self.index_current_setpoint == len(self.setpoints):
                self.index_current_setpoint = 0
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                return control_command 
            else:
                pass
        else:
            pass      

        # MOTOR COMMANDS : start by moving forwards/backwards and then left/right

        if abs(x_goal - x_drone) > 0.07:
            self.direction = "Xaxis"
            scaled_forward_velocity, scaled_left_velocity = self.scale_speed(x_goal - x_drone, 0.0)
            control_command = [scaled_forward_velocity, scaled_left_velocity, 0.0, self.height_desired]
            return control_command
        else:
            pass
        if abs(y_goal - y_drone) > 0.07:
            self.direction = "Yaxis"
            scaled_forward_velocity, scaled_left_velocity = self.scale_speed(0.0, y_goal - y_drone)
            control_command = [scaled_forward_velocity, scaled_left_velocity, 0.0, self.height_desired]
            return control_command  
        else:
            pass 
                   
        control_command = [0.2, 0.0, 0.0, self.height_desired]
        return control_command
        
        
        
        
    def update_map(self, sensor_data):
        pos_x = sensor_data['x_global']
        pos_y = sensor_data['y_global']
        yaw = sensor_data['yaw']
        
        # OBSTACLE MAPPING
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
            
            for i in range(int(self.range_max/self.res_pos)): # range is 2 meters
                dist = i*self.res_pos
                idx_x = int(np.round((pos_x - self.min_x + dist*np.cos(yaw_sensor))/self.res_pos,0))
                idx_y = int(np.round((pos_y - self.min_y + dist*np.sin(yaw_sensor))/self.res_pos,0))
                
                # make sure the point is within the map
                if idx_x < 0 or idx_x >= self.map.shape[0] or idx_y < 0 or idx_y >= self.map.shape[1] or dist > self.range_max:
                    break

                # update the map
                if dist < measurement:
                    self.map[idx_x, idx_y] += self.conf
                else:
                    self.map[idx_x, idx_y] -= self.conf
                    break
                    
                    
                    
                    
                    
        # GROUND MAPPING
        measurement = sensor_data['range_down']
        idx_x_ground = int(np.round((pos_x - self.min_x)/self.res_pos,0))
        idx_y_ground = int(np.round((pos_y - self.min_y)/self.res_pos,0))        
        if not (idx_x_ground < 0 or idx_x_ground >= self.map.shape[0] or idx_y_ground < 0 or idx_y_ground >= self.map.shape[1]):
  
            # update the map
            if measurement > self.PLATFORM_DIST:
                self.map_ground[idx_x_ground, idx_y_ground] += self.conf
            else:
                self.map_ground[idx_x_ground, idx_y_ground] -= self.conf
                
                
                
                
                
                    

        
        self.map = np.clip(self.map, -1, 1) # certainty can never be more than 100%
        self.map_ground = np.clip(self.map_ground, -1, 1) # certainty can never be more than 100%


        # UPDATE LIST OF SETPOINTS : 
        if self.mode == "FINDTARGET" and self.state == "FORWARD":
            indices = np.where(self.map <= -1.0)
            obstacle_positions = list(zip(indices[0], indices[1]))
            for setpoint in self.setpoints:
                setpoint_in_map = self.world_to_grid(setpoint[0], setpoint[1])

                if setpoint_in_map in obstacle_positions:
                    print("Setpoint in obstacle")
                    print(setpoint_in_map)
                    self.setpoints.remove(setpoint)
                    break
        
        
        
        

    
        # only plot every Nth time step (comment out if not needed)
        if self.t % 50 == 0:
            plt.imshow(np.flip(self.map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            plt.savefig("map.png")
            
            plt.imshow(np.flip(self.map_ground,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
            plt.savefig("map_ground.png")
        self.t +=1
    
        return None
    


    def scale_speed(self, forward_velocity, left_velocity):
        if abs(forward_velocity) >= abs(left_velocity):
            scaled_forward_velocity = max( min(forward_velocity, self.MAX_SPEED), -self.MAX_SPEED)
            
            scaled_left_velocity = left_velocity * abs(scaled_forward_velocity / forward_velocity)
        else :
            scaled_left_velocity = max( min(left_velocity, self.MAX_SPEED), -self.MAX_SPEED)
            scaled_forward_velocity = forward_velocity * abs(scaled_left_velocity / left_velocity)
        return scaled_forward_velocity, scaled_left_velocity
    




##############################################################################################
    class Node():
        """Nodes class"""
        
        def __init__(self, parent=None, x=None, y=None):
            self.parent = parent
            self.x = x
            self.y = y
            
            self.g = 0
            self.h = 0
            self.f = 0 
                       
    def A_star(self,nav_map, x_drone, y_drone, x_goal, y_goal):
        print('A star')
        # map specs
        self.min_x, self.max_x = 0, 5.0 # meter
        self.min_y, self.max_y = 0, 3.0 # meter
        self.res_pos = 0.1 # meter
        # Position conversion to map idx        
        x_drone_map = int(round((x_drone - self.min_x)/self.res_pos,0))
        y_drone_map = int(round((y_drone - self.min_y)/self.res_pos,0))
        x_goal = int(round((x_goal - self.min_x)/self.res_pos,0))
        y_goal = int(round((y_goal - self.min_y)/self.res_pos,0))
        
        # If A_star fails
        path = [[x_drone_map,y_drone_map]]
        
        
        # print current location
        plt.scatter(29-y_goal, x_goal, s=50, c='yellow', marker='o')   
        
        
        # Avoiding problem if startnode is on obstacle
        angle = 0
        amplitude = 0.5
        x_drone_map_old, y_drone_map_old = x_drone_map, y_drone_map
        x_drone_map = np.clip(x_drone_map,0,49)
        y_drone_map = np.clip(y_drone_map,0,29)
        while nav_map[x_drone_map,y_drone_map] <= -0.8:
            amplitude += 1/20
            angle += np.pi/20
            x_drone_map = x_drone_map_old + round(np.sin(angle))*round(amplitude)
            y_drone_map = y_drone_map_old + round(np.cos(angle))*round(amplitude)
            x_drone_map = np.clip(x_drone_map,0,49)
            y_drone_map = np.clip(y_drone_map,0,29)
        if amplitude > 0.5:
            print('obstacle on start:', x_drone_map_old, y_drone_map_old)
            print('new start:', x_drone_map,y_drone_map)
        # Avoiding problem if endnode is on obstacle
        angle = 0
        amplitude = 0.5
        x_goal_old, y_goal_old = x_goal, y_goal
        while nav_map[x_goal,y_goal] <= -0.8:
            amplitude += 1/20
            angle += np.pi/20
            x_goal = x_goal_old + round(np.sin(angle))*round(amplitude)
            y_goal = y_goal_old + round(np.cos(angle))*round(amplitude)
            x_goal = np.clip(x_goal,0,49)
            y_goal = np.clip(y_goal,0,29)
        if amplitude > 0.5:
            print('obstacle on setpoint objective:', x_goal_old, y_goal_old)
            print('new objective:', x_goal,y_goal)
        # start and end nodes
        StartNode = self.Node(None,x_drone_map,y_drone_map)
        EndNode = self.Node(None,x_goal,y_goal)
        
        
        
        # Initializing lists
        open_list = []
        closed_list = []
        # adding startnode
        open_list.append(StartNode)
                
        
        # Loop
        iter = 0
        while len(open_list) > 0:
            iter+=1
            if iter > 2000:
                print('A_star max iter reached')
                print('open_list length',len(open_list))
                print('close list length', len(closed_list))
                return x_goal, y_goal
                
            q = open_list[0]    # current node
            q_idx = 0           # current node index
            
            # finding node with min f in open_list
            for idx1, Node1 in enumerate(open_list):
                if Node1.f < q.f:
                    q = Node1
                    q_idx = idx1
                    
            # removing q from open_list and..
            closed_list.append(q)
            open_list.pop(q_idx)
            
            # Goal found?
            if q.x == x_goal and q.y == y_goal:
                path = []
                while q is not None:
                    path.append([q.x,q.y])
                    q = q.parent
                
                # displaying path
                if len(path)<=2 or path == None or path == []:
                    print('\n \n \n A_star Warning!: len(path) =',len(path),'path = ',path)
                #plt.imshow(np.flip(nav_map,1), vmin=-1, vmax=1, cmap='gray', origin='lower')
                # flip the map to match the coordinate system
                # displaying path 
                for i,j in path:
                    # print(path)
                    plt.scatter(29-j, i, s=10, c='green', marker='o')
                plt.scatter(29-y_goal, x_goal, s=50, c='green', marker='*')
                plt.savefig("nav_map.png")
                return path[::-1] # reversed path
                
 
            # q's successors
            #successors = []
            for i,j in [[-1,-1],[1,1],[-1,1],[1,-1],[1,0],[-1,0],[0,-1],[0,1]]:
                # new node position
                qNew = self.Node()
                qNew.x = q.x + i
                qNew.y = q.y + j
                
                # within range of map check
                if qNew.x > 49 or qNew.x < 0 or qNew.y > 29 or qNew.y < 0:
                    continue
                # No obstacle check
                if nav_map[qNew.x,qNew.y] <= -0.8:
                    continue
                    
                # checking if already in closed list
                qNew_bad = False
                for Node1 in closed_list:
                    # not possible to be in closed list and have f < ?
                    if qNew.x == Node1.x and qNew.y == Node1.y:
                        qNew_bad = True
                        break
                if qNew_bad == True:
                    continue
                    
                # qNew properties
                qNew.parent = q
                qNew.g = q.g + abs(i) + abs(j)
                qNew.h = abs(qNew.x - x_goal) + abs(qNew.y - y_goal)
                qNew.f = qNew.g + qNew.h
                
                # checking if qNew in open_list is better
                for Node1 in open_list:
                    if qNew.x == Node1.x and qNew.y == Node1.y and qNew.g >= Node1.g:
                        qNew_bad = True
                        break
                if qNew_bad == True:
                    continue
                open_list.append(qNew)
            
        print('No path found')  
        self.index_current_Astar = 0  
        return path
##############################################################################################

##############################################################################################
    def navigation_map(self):
        nav_map = self.map.copy()
        
        a = 2   # thickness
        # map borders
        nav_map[0:a,:] = -1
        nav_map[50-a:50,:] = -1
        nav_map[:,0:a] = -1
        nav_map[:,30-a:30] = -1
        
        # map obstacles
        for i in range(a,50-a):
            for j in range(a,30-a):
                if self.map[i,j] <= -0.8:
                    # enlarging obstacles
                    nav_map[i-a:i+a,j-a:j+a] = -1
        
        # A very nice map with enlarged obstacles and borders is achieved at this point!
        return nav_map

##############################################################################################


##############################################################################################
    def path_control(self, sensor_data):
        # coordinates
        x_drone = sensor_data['x_global']
        y_drone = sensor_data['y_global']
        # setpoints
        if self.index_current_setpoint == len(self.setpoints) - 1:
            self.index_current_setpoint = 0
        if self.landing_operation == 7:
            x_goal = self.x_home
            y_goal = self.y_home
        else:
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        # drone map coordinates
        x_drone_map = int(round((x_drone - self.min_x)/self.res_pos,0))
        y_drone_map = int(round((y_drone - self.min_y)/self.res_pos,0))
        
        # speed control
        self.speed = 0.3
        if len(self.path) < 3:
            self.speed = 0.1
        else:
            for i in range(-2,3):
                i = np.clip(x_drone_map + i,0,49)
                for j in range(-2,3):
                    j = np.clip(y_drone_map + j,0,29)
                    if self.nav_map[i,j] <= 0.2:
                        self.speed = 0.1
        
        # target
        if len(self.path) <= self.index_current_Astar:
            self.index_current_Astar = len(self.path)-1
        x_target, y_target = self.path[self.index_current_Astar]
        # If end of path reached
        if self.index_current_Astar >= (len(self.path)-1) and len(self.path) > 2 and self.landing_operation != 7:
            self.index_current_setpoint += 1
            x_goal, y_goal = self.setpoints[self.index_current_setpoint]
            print('setpoint reached, new setpoint:', x_goal, y_goal, 'Astar idx:',self.index_current_Astar)
            self.path = self.A_star(self.nav_map, x_drone, y_drone, x_goal, y_goal)
            self.index_current_Astar = 1
        # If drone position too far from path
        elif (abs(x_drone_map - x_target) + abs(y_drone_map-y_target)) > 6:
            print('position too far from path, recalculating:')
            self.path = self.A_star(self.nav_map, x_drone, y_drone, x_goal, y_goal)
            self.index_current_Astar = 1
        # If new obstacle detected on path, recalculate
        else:
            for i,j in self.path:
                if self.nav_map[i,j] <= -0.8:
                    print('coordinate :',i,j,' obstructed:')
                    self.path = self.A_star(self.nav_map, x_drone, y_drone, x_goal, y_goal)
                    self.index_current_Astar = 2 # not 1 to avoid drone turning around
                    break
        # target
        if len(self.path) < 3:
            self.index_current_Astar = len(self.path)-1
            print('warning: short path; len(path) = ',len(self.path))
            x_target, y_target = self.path[self.index_current_Astar]
        else:
            x_target, y_target = self.path[self.index_current_Astar]
        x_target *= self.res_pos   #self.res_pos
        y_target *= self.res_pos   #self.res_pos
        
        return x_target, y_target
##############################################################################################
    def point_control(self, sensor_data, x_goal, y_goal):
        
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        yaw = sensor_data['yaw']
        
        delta_x = x_goal - x_drone
        delta_y = y_goal - y_drone
        distance_to_goal = np.linalg.norm([delta_x, delta_y])
        angle_to_goal = np.arctan2(delta_y,delta_x)
        angle_to_goal_inv = np.arctan2(delta_y,-delta_x)
        
        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
        if distance_to_goal < 0.12:
            # Select the next setpoint as the goal position
            # print('point reached')
            self.index_current_Astar += 1
        
        # yaw proportional controller
        # opposite representation
        if yaw < 0:
            yaw_inv = -yaw - np.pi
        else:
            yaw_inv = -yaw + np.pi
        # selecting representation (normal or inv) to avoid oscillation phenomenon
        if delta_x < 0.0:
            yaw_speed = -angle_to_goal_inv + yaw_inv
        else: 
            yaw_speed = (angle_to_goal - yaw)
        
        yaw_speed = np.clip(yaw_speed,-8,8)
        control_command = [self.speed, 0.0, yaw_speed, self.height_desired]
        # print(control_command)
        # control_command = [0.0, 0.0, 0.0, self.height_desired]
        return control_command
############################################################################################## 