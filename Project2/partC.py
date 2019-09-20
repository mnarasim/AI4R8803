
import math
import random
from robot import *
count=0



loc_p=[]
boxes_d =0
box_picked=0
grid=[[]]
closer=0
m = 7
n = 7
global_obs=[]
grid=[[ ' ' for row in range(n)] for col in range(m)]

class OnlineDeliveryPlanner:

    def __init__(self, todo, max_distance, max_steering, verbose=False):

        ######################################################################################
        # TODO: You may use this function for any initialization required for your planner  
        ######################################################################################
        
        self.moves = []  
        self.todo = todo
        self.max_distance = max_distance
        self.max_steering = max_steering

    def process_measurement(self, data, verbose = False):
        noise_var=0.0
        ######################################################################################
        # TODO:  Replace the following code with your own measurement processing to support  
        # your dynamic planner
        ######################################################################################
        
        dcos=[]
        dsin=[]
        x_pos=0.0
        y_pos=0.0
        global count,loc_p
        if count == 0:
            for m1 in data:
                if m1[0]=='warehouse':
                    if m1[1] == 0:
                        dsin.append(m1[2]*math.sin(m1[3]))
                    elif m1[1]==2:
                        dcos.append(m1[2]*math.cos(m1[3]))
            x_pos = 0.0 - (sum(dcos)/len(dcos))
            y_pos = 0.0 - (sum(dsin)/len(dsin))
                        
            self.res = Robot(x_pos,y_pos,0.0,self.max_distance,self.max_steering)
            loc_p=[x_pos,y_pos]

        
        obs=[]
        measurement_process=[]

        north_wall=[]
        south_wall=[]
        west_wall=[]
        east_wall=[]
        taken=[]
        measures=[]
   
        global m, n, grid
        for meas in data:
            dist = meas[2] + random.gauss(0, meas[2]*0.1*noise_var)
            bear = meas[3] + random.gauss(0, meas[2]*0.05*noise_var)
            dx = dist * math.cos(bear)
            dy = dist * math.sin(bear)
            x1 = self.res.x + dx
            y1 = self.res.y + dy
       
            if meas[0] == 'box':
                if meas[1] == 0:
                    x1_obs = x1 
                    y1_obs = y1 - 0.1
                elif meas[1] == 1:
                    x1_obs = x1
                    y1_obs = y1 + 0.1
                elif meas[1] == 2:
                    x1_obs = x1 + 0.1
                    y1_obs = y1
                elif meas[1] == 3:
                    x1_obs = x1 - 0.1
                    y1_obs = y1
            elif meas[0] == 'wall':
                if meas[1] == 0:
                    x1_obs = x1 
                    y1_obs = y1 - 0.5
                elif meas[1] == 1:
                    x1_obs = x1
                    y1_obs = y1 + 0.5
                elif meas[1] == 2:
                    x1_obs = x1 + 0.5
                    y1_obs = y1
                elif meas[1] == 3:
                    x1_obs = x1 - 0.5
                    y1_obs = y1
            elif meas[0] == 'warehouse':
                x1_obs = x1
                y1_obs = y1
           
            measurement_process.append([meas[0],meas[1],meas[2],meas[3],dx,dy,x1,y1,x1_obs,y1_obs])
               
    
        for i in range(len(measurement_process)):
          
            if measurement_process[i][0] == 'warehouse':
                if measurement_process[i][1] == 0:
                    north_wall.append([measurement_process[i][8],measurement_process[i][9]])
                elif measurement_process[i][1] == 1:
                    south_wall.append([measurement_process[i][8],measurement_process[i][9]])

                elif measurement_process[i][1] == 2:
                    west_wall.append([measurement_process[i][8],measurement_process[i][9]])

                elif measurement_process[i][1] == 3:
                    east_wall.append([measurement_process[i][8],measurement_process[i][9]])
                    
        for i in range(len(measurement_process)):
            if measurement_process[i][0] == 'wall':
                obs.append([measurement_process[i][8],measurement_process[i][9]])
                taken.append(0)
                
  
                   
      
        if count ==0:
            for i in north_wall:
                tempx= i[0]
                tempy = i[1]
                for j in range(n):
                    if tempx > j and tempx <= j + 1:
                        grid[0][j] = 'N'
                    

            for i in south_wall:
                tempx= i[0]
                tempy = i[1]
                for i in range(m):
                    if abs(tempy) > i and abs(tempy) <= i + 1:
                        for j in range(n):
                            if tempx > j and tempx <= j + 1:
                                grid[i][j] = 'S'
                                grid[i+1][j] = '#'

                    
            for i in west_wall:
                tempx= i[0]
                tempy = i[1]
                for i in range(m):
                    if abs(tempy) > i and abs(tempy) <= i + 1:
                        grid[i][0] = 'W'

            for i in east_wall:
                tempx= i[0]
                tempy = i[1]
                for i in range(m):
                    if abs(tempy) > i and abs(tempy) <= i+1:
                        for j in range(n):
                            if tempx > j and tempx <= j + 1:
                                grid[j][i] = 'E'

            ind=-1
            for i in obs:
                ind = ind+1
                if taken[ind]==0:
                    tempx= i[0]
                    tempy = i[1]
                    for i in range(m):
     
                        if abs(tempy) > i and abs(tempy) <= i+1:
      
                            for j in range(n):
                                if tempx > j and tempx <= j + 1:
                                    if grid[i][j]== '#':
                                        grid[i][j] = '#'
                                    else:
                                    
                                        for xyz in range(i+1, len(obs)):
                                        
                                        
                                        
                                            min_dist = math.sqrt((tempx-obs[xyz][0])**2 + (tempy-obs[xyz][1])**2)
                                            if min_dist < 1:
                                                taken[xyz]=1
 
                                        grid[i][j]= '#'
                                        global_obs.append([tempx,tempy])
        
      
        if count >= 0 and count<3:
            
            for i in obs:
                
                tempx= i[0]
                tempy = i[1]
                for i in range(m):
       
                    if abs(tempy) > i and abs(tempy) <= i+1:
      
                        for j in range(n):
                            if tempx > j and tempx <= j + 1:
                                 if grid[i][j]== '#':
                                    grid[i][j] = '#'
                                 else:
                                    temp_coord=[]
                                    for xyz in range(len(global_obs)):
                                        
                                        
       
                                        min_dist = math.sqrt((tempx-global_obs[xyz][0])**2 + (tempy-global_obs[xyz][1])**2)
                                        temp_coord.append(min_dist)
                                    temp_coord.sort()
                                    
                                    if temp_coord[0] > 1.0:
                                        grid[i][j] = '#'
                                    
                                            
                                    #grid[temp_coord[0][0][1]][temp_coord[0][0][2]] = '#'

       
                                
                     
        count = count + 1
        
    def next_move(self, verbose = False):
        move_dist=0.0
        move_turn=0.0
        #######################################################################################
        # TODO:  Finish this function
        #######################################################################################
       
        global boxes_d,closer,m,n, box_picked,grid,loc_P,count
        blocked = [[99 for row in range(n)] for col in range(m)]
        value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
        policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
        init=[]
        goal=[]
        init_p=[]
        goal_p=[]
        path=[]
 
        path_pts=[]
        init_p=[self.res.x, self.res.y]
   
      
        for i in range(m):
            tempx=init_p[0]
            tempy = init_p[1]
            if abs(tempy) > i and abs(tempy) <= i+1:
                for j in range(n):
                    if tempx > j and tempx <= j + 1:
                        init=[i,j]
                        
        if box_picked ==0:
            goal_p=self.todo[boxes_d]
        elif box_picked == 1:
            goal_p=loc_p
            
        for x in range(boxes_d+1, len(self.todo)):
            for i in range(m):
                for j in range(n):
                    if self.todo[x][0] - 0.1 > i and self.todo[x][0] - 0.1 < i+ 1:
                        if abs(self.todo[x][1] - 0.1) > j and abs(self.todo[x][1] - 0.1) < j + 1:
                            blocked[j][i]= x
                    if self.todo[x][0] + 0.1 > i and self.todo[x][0] + 0.1 < i+ 1:
                        if abs(self.todo[x][1] + 0.1) > j and abs(self.todo[x][1] + 0.1) < j + 1:
                            blocked[j][i]= x
               
        for i in range(m):
            tempx=goal_p[0]
            tempy = goal_p[1]
     
            if abs(tempy) > i and abs(tempy) <= i+1:
                for j in range(n):
                    if tempx > j and tempx <= j + 1:
                        goal=[i,j]
       
   
        delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ], # go right
                 [-1, 1 ], # go up right (diagnol)
                 [ 1, 1 ], # go down right (diagnol)
                 [-1, -1], # go up left (diagnol)
                 [ 1, -1]] # go down left(diagnol)

        delta_name = ['^', '<', 'v', '>', '^>', 'v>', '^<', 'v<']
       
        change = True
        while change:
            change = False
     
            for x in range(len(grid)):
              
                for y in range(len(grid[0])):
                  
                    if goal[0] == x and goal[1] == y:
                    
                        if value[x][y] > 0:
                            value[x][y] = 0
                            policy[x][y]= '*'
                            change = True
      
      
                    
                    elif grid[x][y] != '#':
                        for a in range(len(delta)):
                          
                            x2 = x + delta[a][0]
                            y2 = y + delta[a][1]
                            
                            
                            if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] != '#' and blocked[x2][y2]== 99:
          
                             
                                if a == 4:
                                    if grid[x-1][y] != '#' and grid[x][y+1] != '#':
                                         v2 = value[x2][y2] + math.sqrt((x2-x)**2 + (y2-y)**2)
                                       
                                elif a == 5:
                                    if grid[x][y+1] != '#' and grid[x+1][y] != '#':
                                        v2 = value[x2][y2] + math.sqrt((x2-x)**2 + (y2-y)**2)
                                    
                                elif a == 6:
                                    if grid[x-1][y] != '#' and grid[x][y-1] != '#':
                                        v2 = value[x2][y2] + math.sqrt((x2-x)**2 + (y2-y)**2)
       
                                elif a == 7:
                                    if grid[x][y-1] != '#' and grid[x+1][y] != '#':
                                        v2 = value[x2][y2] + math.sqrt((x2-x)**2 + (y2-y)**2)
      
                                else:
                                    v2 = value[x2][y2] + math.sqrt((x2-x)**2 + (y2-y)**2)
        
        
           
                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                
                                    policy[x][y]=delta_name[a]
                                  
      
        
        x = init[0]
        y = init[1]
       
       
        path.append([x,y])
        
        while x != goal[0] or y!=goal[1]:
           
            for c in range(len(delta_name)):
         
                if policy[x][y] == delta_name[c]:
                    x2 = x + delta[c][0]
                    y2 = y + delta[c][1]
                    path.append([x2,y2])
    
              
                    x = x2
                    y = y2

   
        if closer==0:                    
            for i in range(len(path)):
                if i ==0:
                    path_pts.append([self.res.x, self.res.y])
                elif i == len(path)-1:
                    if box_picked == 0:
                    
                        path_pts.append(self.todo[boxes_d])
                    else:
                        path_pts.append(loc_p)
                else:

                    path_pts.append([(path[i][1]+path[i][1]+1.)/2.0,-1.*(path[i][0]+path[i][0]+1.)/2.0])
     
        

        tolerance=0.45
        
        if len(path) > 1 and closer==0:
            move_dist,move_turn = self.res.measure_distance_and_bearing_to(path_pts[1],True)       
        
            if move_turn > self.max_steering:
                        
                move_turn = self.max_steering
                self.res.move(move_turn,0.0,True)
                return 'move ' + str(move_turn) + ' ' + str(0.0)
                   
            elif move_turn < -self.max_steering:
                move_turn = -self.max_steering
                self.res.move(move_turn,0.0,True)
                return 'move ' + str(move_turn) + ' ' + str(0.0)
 
            else:
                if path[1]== goal:
                    move_dist = move_dist-tolerance
                    closer=1
    
                self.res.move(move_turn,move_dist,True)
                return 'move ' + str(move_turn) + ' ' + str(move_dist)
            
        else:
      
                if box_picked == 0:
                    move_dist,move_turn = self.res.measure_distance_and_bearing_to(goal_p,True)
                    
          
                    
                    if move_dist >= 0.45:
                        self.res.move(0,0.02,True)
    
                        return 'move ' + str(0) + ' ' + str(0.02)
                    elif abs(move_turn) > 0.17:
         
                        if move_turn > self.max_steering:
                        
                            move_turn = self.max_steering
                        elif move_turn < -self.max_steering:
                        
                            move_turn = -self.max_steering
                        self.res.move(move_turn,0.0,True)
                        
                        
                        return 'move ' + str(move_turn) + ' ' + str(0.0)
    
                    
                    else:
                        c10=True
                        while c10:
                            c10=False
                            move_dist,move_turn = self.res.measure_distance_and_bearing_to(goal_p,True)
                            if move_dist >=0.45:
                                self.res.move(0,0.02,True)
                                c10=True
                                return 'move ' + str(0) + ' ' + str(0.02)
                            else:
                                box_picked = box_picked + 1
                                closer=0
                                
                                return 'lift ' + str(move_turn)
                        
                elif box_picked == 1:
                    move_dist,move_turn = self.res.measure_distance_and_bearing_to(goal_p,True)
                    
                    if move_dist>=0.45:
                        self.res.move(0,0.02,True)
    
                        return 'move ' + str(0) + ' ' + str(0.02)
                    elif abs(move_turn) > 0.17:
                        #move_turn = move_turn - self.res.bearing
                        if move_turn > self.max_steering:
                        
                            move_turn = self.max_steering
                        elif move_turn < -self.max_steering:
                        
                            move_turn = -self.max_steering
                        self.res.move(move_turn,0.0,True)
                        
                        return 'move ' + str(move_turn) + ' ' + str(0.0)
                    
                    else:
                        box_picked = box_picked - 1
                        closer=0
                        boxes_d = boxes_d + 1
                        if boxes_d == len(self.todo):
                            boxes_d =0
                            count=0
                        return 'down ' + str(move_turn)
                    
           


                
        


    def get_robot_state(self, robot_has_box, robot_is_crashed, boxes_delivered, verbose = False):
        
        #######################################################################################
        # TODO:  This function is populated from testing_suite_partC to let you know if the
        # robot is holding a box,
        # robot is crashed,
        # a list of the boxes delivered so far
        #######################################################################################      
        
        self.robot_has_box = robot_has_box
        self.robot_is_crashed = robot_is_crashed
        self.boxes_delivered = boxes_delivered

        if verbose:
            print "Robot has box: {},  Robot is crashed: {}".format(robot_has_box, robot_is_crashed)
            print "Boxes delivered: {}".format(boxes_delivered)

