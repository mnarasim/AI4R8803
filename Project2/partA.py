#
# === Introduction ===
#
# In this problem, you will build a planner that helps a robot
#   find the best path through a warehouse filled with boxes
#   that it has to pick up and deliver to a dropzone.
# 
# Your file must be called `partA.py` and must have a class
#   called `DeliveryPlanner`.
# This class must have an `__init__` function that takes three 
#   arguments: `self`, `warehouse`, and `todo`.
# The class must also have a function called `plan_delivery` that 
#   takes a single argument, `self`.
#
# === Input Specifications ===
# 
# `warehouse` will be a list of m strings, each with n characters,
#   corresponding to the layout of the warehouse. The warehouse is an
#   m x n grid. warehouse[i][j] corresponds to the spot in the ith row
#   and jth column of the warehouse, where the 0th row is the northern
#   end of the warehouse and the 0th column is the western end.
#
# The characters in each string will be one of the following:
#
# '.' (period) : traversable space. The robot may enter from any adjacent space.
# '#' (hash) : a wall. The robot cannot enter this space.
# '@' (dropzone): the starting point for the robot and the space where all boxes must be delivered.
#   The dropzone may be traversed like a '.' space.
# [0-9a-zA-Z] (any alphanumeric character) : a box. At most one of each alphanumeric character 
#   will be present in the warehouse (meaning there will be at most 62 boxes). A box may not
#   be traversed, but if the robot is adjacent to the box, the robot can pick up the box.
#   Once the box has been removed, the space functions as a '.' space.
# 
# For example, 
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   is a 3x3 warehouse.
#   - The dropzone is at the warehouse cell in row 2, column 2.
#   - Box '1' is located in the warehouse cell in row 0, column 0.
#   - Box '2' is located in the warehouse cell in row 0, column 2.
#   - There are walls in the warehouse cells in row 0, column 1 and row 1, column 1.
#   - The remaining five warehouse cells contain empty space.
#
# The argument `todo` is a list of alphanumeric characters giving the order in which the 
#   boxes must be delivered to the dropzone. For example, if 
#   todo = ['1','2']
#   is given with the above example `warehouse`, then the robot must first deliver box '1'
#   to the dropzone, and then the robot must deliver box '2' to the dropzone.
#
# === Rules for Movement ===
#
# - Two spaces are considered adjacent if they share an edge or a corner.
# - The robot may move horizontally or vertically at a cost of 2 per move.
# - The robot may move diagonally at a cost of 3 per move.
# - The robot may not move outside the warehouse.
# - The warehouse does not "wrap" around.
# - As described earlier, the robot may pick up a box that is in an adjacent square.
# - The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
# - While holding a box, the robot may not pick up another box.
# - The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
#   of 1 (regardless of the direction in which the robot puts down the box).
# - If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
#   house.
# - The warehouse will be arranged so that it is always possible for the robot to move to the 
#   next box on the todo list without having to rearrange any other boxes.
#
# An illegal move will incur a cost of 100, and the robot will not move (the standard costs for a 
#   move will not be additionally incurred). Illegal moves include:
# - attempting to move to a nonadjacent, nonexistent, or occupied space
# - attempting to pick up a nonadjacent or nonexistent box
# - attempting to pick up a box while holding one already
# - attempting to put down a box on a nonadjacent, nonexistent, or occupied space
# - attempting to put down a box while not holding one
#
# === Output Specifications ===
#
# `plan_delivery` should return a LIST of moves that minimizes the total cost of completing
#   the task successfully.
# Each move should be a string formatted as follows:
#
# 'move {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot moves
#   to and '{j}' is replaced by the column-coordinate of the space the robot moves to
# 
# 'lift {x}', where '{x}' is replaced by the alphanumeric character of the box being picked up
#
# 'down {i} {j}', where '{i}' is replaced by the row-coordinate of the space the robot puts 
#   the box, and '{j}' is replaced by the column-coordinate of the space the robot puts the box
#
# For example, for the values of `warehouse` and `todo` given previously (reproduced below):
#   warehouse = ['1#2',
#                '.#.',
#                '..@']
#   todo = ['1','2']
# `plan_delivery` might return the following:
#   ['move 2 1',
#    'move 1 0',
#    'lift 1',
#    'move 2 1',
#    'down 2 2',
#    'move 1 2',
#    'lift 2',
#    'down 2 2']
#
# === Grading ===
# 
# - Your planner will be graded against a set of test cases, each equally weighted.
# - If your planner returns a list of moves of total cost that is K times the minimum cost of 
#   successfully completing the task, you will receive 1/K of the credit for that test case.
# - Otherwise, you will receive no credit for that test case. This could happen for one of several 
#   reasons including (but not necessarily limited to):
#   - plan_delivery's moves do not deliver the boxes in the correct order.
#   - plan_delivery's output is not a list of strings in the prescribed format.
#   - plan_delivery does not return an output within the prescribed time limit.
#   - Your code raises an exception.
#
# === Additional Info ===
# 
# - You may add additional classes and functions as needed provided they are all in the file `partA.py`.
# - Upload partA.py to Project 2 on T-Square in the Assignments section. Do not put it into an 
#   archive with other files.
# - Ask any questions about the directions or specifications on Piazza.
#

class DeliveryPlanner:

    def __init__(self, warehouse, todo):
        self.warehouse = warehouse
        self.todo = todo

    def plan_delivery(self):
        grid=[[]]
        m = len(self.warehouse)
        n = len(self.warehouse[0])
        grid=[[ ' ' for row in range(n)] for col in range(m)]
        moves=[]
        pick_up=[]
        loc=[]
        
        for i in range(m):
            for j in range(n):
     
                grid[i][j] = self.warehouse[i][j]
        
        for k in range(len(self.todo)):
            
            for i in range(m):
                for j in range(n):
                    if grid[i][j] == self.todo[k]:
                        
                        pick_up.append([i,j])
        for i in range(m):
            for j in range(n):
     
                if grid[i][j] == '@':
                    loc=[i,j]
        
                        
        cost = [2,2,2,2,3,3,3,3]
        cost_load = 4
        cost_unload = 2
        delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ], # go right
                 [-1, 1 ], # go up right (diagnol)
                 [ 1, 1 ], # go down right (diagnol)
                 [-1, -1], # go up left (diagnol)
                 [ 1, -1]] # go down left(diagnol)

        delta_name = ['^', '<', 'v', '>', '^>', 'v>', '^<', 'v<']
        final_cost=0.0
        count=0
        m5=[]
        
        for n in range(len(pick_up)):
            cycle =0
            last_goal=[]
            if n==0:
                init = loc
            while cycle < 2 or count > 0:
                if cycle==0:
            
                    goal = pick_up[n]
            
     
                p,x4,y4,tc,c1,m1 = (optimum_policy(grid,goal,cost,init,loc,count,delta,delta_name,cost_load,cost_unload))
                temp=[x4,y4]
                m5.append(m1)
                last_goal = goal
                goal = loc
                init=temp
                count = c1
                if goal == init:
                    goal = last_goal
           
            
                final_cost = final_cost + tc
                
                
                cycle = cycle + 1
        for i in m5:
            for j in i:
                moves.append(j)
        return moves
def optimum_policy(grid,goal,cost,init,loc,count,delta,delta_name,cost_load,cost_unload):
   
    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    change = True
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
                        
                        if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] != '#':
                            
                            
                            if x2 == goal[0] and y2 == goal[1]:
                                v2 = value[x2][y2] + cost_load
                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                
                                    policy[x][y]=delta_name[a]
                            elif grid[x2][y2] == '.' or grid[x2][y2] == '@':
                                v2 = value[x2][y2] + cost[a]
                            
                                
                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                
                                    policy[x][y]=delta_name[a]


    
    x = init[0]
    y = init[1]
   
    x3=0
    y3=0
    total_cost=0
    
    while x != goal[0] or y!=goal[1]:
 
        for c in range(len(delta_name)):
         
            if policy[x][y] == delta_name[c]:
                
                x2 = x + delta[c][0]
                y2 = y + delta[c][1]
                
                if x2==loc[0] and y2==loc[1] and x2==goal[0] and y2==goal[1]:
                    moves.append('down ' + str(x2) +' ' +str(y2))
                    total_cost = total_cost + cost_unload
                    x3=x
                    y3=y
                    count = count -1
                elif x2 ==goal[0] and y2==goal[1] and count==0.0:
                    moves.append('lift ' + str(grid[x2][y2]))
                    grid[x2][y2] = '.'
                    x3=x
                    y3=y
                    count = count + 1
                    total_cost = total_cost + cost_load
                else:
                    moves.append('move ' + str(x2) +' ' +str(y2))
                    total_cost = total_cost + cost[c]
                    x3=x2
                    y3=y2
                x = x2
                y = y2
      
                  
 
    return policy,x3,y3,total_cost,count,moves
