import math
from robot import *




class DeliveryPlanner:

    def __init__(self, warehouse, todo, max_distance, max_steering):

  
        self.warehouse = warehouse
        self.todo = todo
        self.max_distance = max_distance
        self.max_steering = max_steering
        

    def plan_delivery(self):
      
        grid=[[]]
    
        m = len(self.warehouse)
        n = len(self.warehouse[0])
        moves=[]

        grid=[[ ' ' for row in range(n)] for col in range(m)]
       
        loc=[]
        loc_pts=[0.0,0.0,0.0]
        init=[]
        for i in range(m):
            for j in range(n):
                
                grid[i][j] = self.warehouse[i][j]
                if self.warehouse[i][j] == '@':
                    loc=[i,j]
                    loc_pts[0]=(j+j+1.)/2.0
                    loc_pts[1]=-1.*(i+i+1.)/2.0
                    loc_pts[2]=0.0

        
         
        delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ], # go right
                 [-1, 1 ], # go up right (diagnol)
                 [ 1, 1 ], # go down right (diagnol)
                 [-1, -1], # go up left (diagnol)
                 [ 1, -1]] # go down left(diagnol)

        delta_name = ['^', '<', 'v', '>', '^>', 'v>', '^<', 'v<']
        temp=0
        m5 =[]
       
        for xyz in range(len(self.todo)):
            blocked = [[99 for row in range(n)] for col in range(m)]

            temp = self.todo[xyz]
            box = xyz
           
            for i in range(m):
               if abs(temp[1]) > i and abs(temp[1]) <= i + 1:
                   for j in range(n):
                       if temp[0] > j and temp[0]  <= j+ 1:
                           goal=[i,j]
                           
     
            for x in range(xyz+1, len(self.todo)):
                for i in range(m):
                    for j in range(n):
                        if self.todo[x][0] - 0.1 > i and self.todo[x][0] - 0.1 < i+ 1:
                            if abs(self.todo[x][1] - 0.1) > j and abs(self.todo[x][1] - 0.1) < j + 1:
                                blocked[j][i]= x
                        if self.todo[x][0] + 0.1 > i and self.todo[x][0] + 0.1 < i+ 1:
                            if abs(self.todo[x][1] + 0.1) > j and abs(self.todo[x][1] + 0.1) < j + 1:
                                blocked[j][i]= x
                        
      
            init = loc
    
            
     
    
            m1 = (optimum_policy(grid,goal,init,delta, delta_name, self.max_distance,self.max_steering,self.todo,loc,loc_pts,blocked,box))
            m5.append(m1)
      
          
        for i in m5:
            for j in i:
                 moves.append(j)
           
       
        
        return moves
   

def optimum_policy(grid,goal,init, delta, delta_name,max_distance,max_steering,todo,loc,loc_pts,blocked,box):

    value = [[99 for row in xrange(len(grid[0]))] for col in xrange(len(grid))]
    policy = [[' ' for row in xrange(len(grid[0]))] for col in xrange(len(grid))]
   
    change = True
    
    path=[]
    directions=[]
    
    newpath=[]
    newpath_pts=[]
    moves=[]
    while change:
        change = False

        for x in range(len(grid)):
           
            for y in range(len(grid[0])):
                
                if goal[0] == x and goal[1] == y:
                    
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y]= '*'
                        change = True
                                
                elif grid[x][y] == '.' or grid[x][y] == '@':
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

    for i in range(len(path)):
       directions.append(policy[path[i][0]][path[i][1]])
      
        

    i=0
    
    newpath.append(path[0])
    while i < len(directions)-1:
        d1 = directions[i]
        j1=i
     
        check = True
        no_steps=0
        while check and j1<len(directions)-1:
            j1 = j1 + 1
            no_steps=no_steps+1
            if d1 != directions[j1] or no_steps >= max_distance:
                check = False
        i = j1
        newpath.append(path[j1])
 
    for i in range(len(newpath)):
        if i == 0:
            
            newpath_pts.append([loc_pts[0],loc_pts[1]])
           
        elif i == len(newpath)-1:
     
            newpath_pts.append([todo[box][0],todo[box][1]])
    
        else:
            newpath_pts.append([(newpath[i][1]+newpath[i][1]+1)/2.0, -1*(newpath[i][0]+newpath[i][0]+1)/2.0])
    
    
    
    move_dist=0.
    move_turn=0.
   
    res=Robot(loc_pts[0],loc_pts[1],loc_pts[2],max_distance,max_steering)
    for k in range(2):
        
      
        tolerance=0.47

        if k == 1:
            newpath.reverse()
            newpath_pts.reverse()
            init=newpath[0]
            goal=newpath[-1]
           
      
                
        for i in range(len(newpath)):
   
            check = True
            if i < len(newpath)-1:
    
                while check:
                    check = False
                    
                    move_dist,move_turn =  res.measure_distance_and_bearing_to(newpath_pts[i+1])
                    move_dist = min(max_distance,move_dist)
                    #move_turn = move_turn - res.bearing
               
    
                    if move_turn >= max_steering:
                        
                        move_turn = -max_steering
                        res.move(move_turn,0.0)
                        moves.append('move ' + str(move_turn) + ' ' + str(0.0))
                        check = True
                       
                   
                    elif move_turn <= -max_steering:
                        
                        move_turn = max_steering
                        res.move(move_turn,0.0)
                        moves.append('move ' + str(move_turn) + ' ' + str(0.0))
                        check = True
                        
                    else:
                        if i == len(newpath)-2:
                            move_dist = move_dist-tolerance
                            
                        
                        
                        res.move(move_turn,move_dist)
                        
                        moves.append('move ' + str(move_turn) + ' ' + str(move_dist))
                        
            elif newpath[i] == goal and goal != loc:

      
                loc_pts[0]=res.x
                loc_pts[1]=res.y
                loc_pts[2]=res.bearing
                newpath_pts[0]= [(loc[1]+loc[1]+1)/2.0,-1*(loc[0]+loc[0]+1)/2.0]
 
     
            
                moves.append('lift ' + str(box))
            elif newpath[i] == goal and goal == loc:
    
        
                loc_pts[0]=res.x
                loc_pts[1]=res.y
                loc_pts[2]=res.bearing
                
                
                moves.append('down ' + str((loc[1]+loc[1]+1)/2.0)+ ' ' + str(-1*(loc[0]+loc[0]+1)/2.0))
           
 
 


    return moves

    

