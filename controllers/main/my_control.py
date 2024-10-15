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
        
        self.state = "TAKEOFF" # States: "TAKEOFF", "FORWARD", "OBSTACLE", "LAND"
        self.mode = "FINDTARGET" # "FINDTARGET", "GOBACKHOME", "DONE"
        self.MAX_SPEED = 0.2
        self.PLATFORM_DIST = 0.44

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
        self.period = 0.2

        self.map = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied
        self.map_ground = np.zeros((int((self.max_x-self.min_x)/self.res_pos), int((self.max_y-self.min_y)/self.res_pos))) # 0 = unknown, 1 = free, -1 = occupied

        
        
        
        
        

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
        print(f"Range_ground : {sensor_data['range_down']:.2f}")
      
        
        
        
        
        
        
        
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
                    print(f"x_home :{self.x_home}\n")
                    print(f"y_home : {self.y_home}\n")
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
                self.state = "FORWARD"
                # time_diff = abs(self.start_hover - sensor_data['t'])
                # if time_diff > 0.5:
                #     self.state = "FORWARD"
                    
                #     return control_command
                return control_command
                
                
                
            
            
            
            
                
        elif self.state == "FORWARD":
        
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            
            
            # IF OBSTACLE DETECTED :
            if sensor_data['range_front'] < 0.2 or sensor_data['range_left'] < 0.2 or sensor_data['range_right'] < 0.2 or sensor_data['range_back'] < 0.2:
                self.sensor_data_list = [sensor_data['range_front'], sensor_data['range_left'], sensor_data['range_right'], sensor_data['range_back']]
                self.min_index = self.sensor_data_list.index(min(self.sensor_data_list))
                print(f"Closest obstacle : {self.min_index}, value : {self.sensor_data_list[self.min_index]}\n")

                self.state = "OBSTACLE"
                return control_command
            else:
                pass
                
            # IF PLATFORM BELOW DETECTED : 
            if sensor_data['range_down'] < self.PLATFORM_DIST:
                if self.mode == "GOBACKHOME":
                    print(f"Distance drone to home : {distance_drone_to_home}\n")
                    if distance_drone_to_home < 0.1:
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        self.state = "LAND"
                        return control_command
                    
                elif self.mode == "FINDTARGET":
                    if sensor_data['x_global'] >= 3.5:
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        self.state = "LAND"
                        return control_command
                    else:
                        print("blabla")
                            
                    
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
            print(f"Closest obstacle at start : {self.min_index}, value : {self.sensor_data_list[self.min_index]}\n")



            if self.sensor_data_list[self.min_index] < 0.3:

                if self.min_index == 0:
                    control_command = [-0.08, 0.2*np.sign(scaled_left_velocity), 0.0, self.height_desired]
                    return control_command
                elif self.min_index == 1:
                    control_command = [0.2*np.sign(scaled_forward_velocity), -0.08, 0.0, self.height_desired]
                    return control_command
                elif self.min_index == 2:
                    control_command = [0.2*np.sign(scaled_forward_velocity), 0.08, 0.0, self.height_desired] 
                    return control_command
                elif self.min_index == 3:
                    control_command = [0.08, 0.2*np.sign(scaled_left_velocity), 0.0, self.height_desired]
                    return control_command
                else:
                    pass
            else:
                control_command = [0.0, 0.0, 0.0, self.height_desired]
                self.state = "HOVER"
                return control_command


            
            
            
            
                
                
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
        
            self.height_desired -= 0.001
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
        print(f"Distance to setpoint {self.index_current_setpoint} : {distance_drone_to_goal}")
        

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

        if abs(x_goal - x_drone) > 0.1:
            scaled_forward_velocity, scaled_left_velocity = self.scale_speed(x_goal - x_drone, 0.0)
            control_command = [scaled_forward_velocity, scaled_left_velocity, 0.0, self.height_desired]
            return control_command
        else:
            pass
        if abs(y_goal - y_drone) > 0.1:
            scaled_forward_velocity, scaled_left_velocity = self.scale_speed(0.0, y_goal - y_drone)
            control_command = [scaled_forward_velocity, scaled_left_velocity, 0.0, self.height_desired]
            return control_command  
        else:
            pass  

        # yaw_to_goal = np.arctan2(y_goal - y_drone, x_goal - x_drone)
        # yaw_diff = yaw_to_goal - sensor_data['yaw']
        # yaw_diff = np.mod(yaw_diff + np.pi, 2 * np.pi) - np.pi
        # if abs(yaw_diff) > 0.1:
        #     control_command = [0.0, 0.0, yaw_diff, self.height_desired]
        #     return control_command
        # else:
        #     pass

        # if distance_drone_to_goal < 0.1:
        #     # Select the next setpoint as the goal position
        #     if self.mode == "FINDTARGET":
        #         if self.index_current_setpoint == len(self.setpoints):
        #             self.index_current_setpoint = 0
        #         else:
        #             self.index_current_setpoint += 1
                    
        #     elif self.mode == "GOBACKHOME":
        #         self.state = "LAND"
        #         control_command = [0.0, 0.0, 0.0, self.height_desired]
        #         return control_command
                
        #     else:
        #         pass
                   
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


        # Current setpoints : 
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