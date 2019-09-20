# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random


def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""


    N=100
   
    distance_noise_factor =0.05
    if OTHER == None:
        
        p_initial = []
               
        for i in range(N):
             t1 = random.uniform(0,2*pi)
                         
             x1 = target_measurement[0]+ random.gauss(0,distance_noise_factor*random.uniform(1,5))
                     
             y1 = target_measurement[1]+ random.gauss(0,distance_noise_factor*random.uniform(1,5))
             p_initial.append([x1,y1,t1])
        #print (p_initial, 'p right irst')
     
        last_turn =0.0
        thetha_list=[]
        thetha=0.0
        turn_list=[]
        updated_measurement=[0.0,0.0]
        last_updated =[0.0,0.0]
      
        thetha_avg=0.0
        step=0
        t2=0
        d2=0
        turning_list=[]
    
        turn_avg=0.0
        OTHER = [[0.0,0.0], thetha,p_initial,step,[0.0,0.0,0.0],thetha_list,turning_list,turn_list]

  
    distance = distance_between(target_measurement, [OTHER[4][0],OTHER[4][1]])
    step=OTHER[3]
    last_updated=OTHER[4]
    last_thetha = OTHER[1]
    p_last = OTHER[2]
    thetha_list = OTHER[5]
 
    turning_list=OTHER[6]
    turn_list=OTHER[7]
    measurement_orient = atan2(target_measurement[1] -OTHER[0][1], target_measurement[0] -OTHER[0][0])
    thetha_list.append(measurement_orient)
    std_thetha=0.0
    thetha_avg=sum(thetha_list)/len(thetha_list)
    for i in range(len(thetha_list)):
        std_thetha = std_thetha + (thetha_list[i]-thetha_avg)**2
  
    std_thetha = std_thetha/max(1,len(thetha_list)-1)
    
    thetha = atan2(target_measurement[1] -OTHER[4][1], target_measurement[0] -OTHER[4][0])
    
    turn = thetha - last_updated[2]
    if turn < -pi:
        turn = 2*pi + turn
    elif turn > pi:
        turn = turn - 2*pi
    motions=[turn,distance]
    step = step + 1
    #print('The measurement is', target_measurement, 'thetha', thetha, 'turn', turn, 'distance', distance)

    result=robot()
    result = particle_filter(motions, target_measurement, N,p_last,distance_noise_factor,measurement_orient, std_thetha,step)
    position=get_position(result)
    res = robot(position[0],position[1],position[2])
    updated_measurement=[position[0],position[1],position[2]]
    distance2 = distance_between([updated_measurement[0],updated_measurement[1]], [last_updated[0],last_updated[1]])
    turn2 = updated_measurement[2]-last_updated[2]
    if turn2 < -pi:
        turn2 = 2*pi + turn2
    elif turn2 > pi:
        turn2 = turn2 - 2*pi
    turn_list.append(turn2)
    turn_avg = sum(turn_list)/len(turn_list)     
    res.move(turn_avg, distance2)
    x = res.x
    y = res.y
    z= res.heading
    xy_estimate = [x, y]
    #print('xy estimate is', xy_estimate, 'heading s', z)
   

    
    hunter_distance=0.0
  

    required_distance = distance_between(hunter_position, xy_estimate)



    
        
    if required_distance > max_distance:
        heading_to_target = get_heading(hunter_position, xy_estimate)
        
        heading_difference = heading_to_target - hunter_heading
        turning = heading_difference
        hunter_distance = max_distance
        #print(required_distance, turning)
    else:
        
        heading_to_target = get_heading(hunter_position, xy_estimate)
        
        heading_difference = heading_to_target - hunter_heading
        turning = heading_difference
        hunter_distance = required_distance
        

    if turning < -pi:
        turning = 2*pi + turning
    elif turning > pi:
        turning = turning - 2*pi

    #turning = turning + sqrt((random.gauss(0, distance_noise_factor*distance2)*2)**2)
    
    
    turning_list.append(turning)
    turning_avg = sum(turning_list)/len(turning_list)
    #print('distance reqd is', required_distance, 'reqd turning', turning)
 


 
    
    #print(hunter_position, 'hunter is at', hunter_heading, 'hunter heading is')
    #print(target_measurement, 'target is at')
    #print('distance between target and hunter', distance_between(hunter_position,target_measurement))
    #print(xy_estimate, 'target will go to', z)
    #print(heading_to_target, 'now the heading to target is')
    #print(turning, hunter_distance, max_distance, 'turn, dist, max')
    #print ('')    
    
    
    OTHER = [target_measurement,thetha,result,step,updated_measurement,thetha_list,turning_list,turn_list]
    return turning, hunter_distance, OTHER
def measurement_prob(predict, measurements,bearing_noise,m_thetha,std_mthetha,p_heading):

        # calculate the correct measurement
        #predict = [predict_x, predict_y]
        #print('')
        #print(predict, 'predicted inside sense', bearing_noise, 'Noise')
        #print(measurements, 'inside sense')
       
        # compute errors
        error = 1.0
       
        
        error_measurement = distance_between(measurements,predict)
        #print('The error in meas', error_measurement)
        #for i in range(len(measurements)):
        #    error_measurement = abs(measurements[i] - predict[i])
            #error_measurement = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            

            # update Gaussian
        for i in range(len(measurements)):
            if bearing_noise > 0.0:
                mu = measurements[i]
                sigma = bearing_noise
                x=predict[i]
        
            
                #expo = exp(-1/2.0*((round(predict[0],2)-round(measurements[0],2))**2/(bearing_noise**2))+((round(predict[1],2)-round(measurements[1],2))**2/(bearing_noise**2)))
                #a1 = ((round(predict[0],4)-round(measurements[0],4))**2/(bearing_noise**2))
                #a2 = ((round(predict[1],4)-round(measurements[1],4))**2/(bearing_noise**2))
                #print(a1,a2,'test')
                #expo = exp(-0.5*(a1+a2))
                #error = (1/(2*pi*bearing_noise**2))* expo
                error *= exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
                #error *= (exp(- (error_measurement ** 2) / (bearing_noise ** 2) / 2.0) /  
        error *= exp(- ((m_thetha - p_heading) ** 2) / (std_mthetha ** 2) / 2.0) / sqrt(2.0 * pi * (std_mthetha ** 2))                       
        #print('Gaussian Probab', error)
        
        return error
def get_position(final_p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    #print(final_p)
    for i in range(len(final_p)):
        x += final_p[i][0]
        y += final_p[i][1]
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((final_p[i][2] - final_p[0][2] + pi) % (2.0 * pi)) 
                        + final_p[0][2] - pi)
    return [x / len(final_p), y / len(final_p), orientation / len(final_p)]
def particle_filter(motions, measurements, N,plist,noise_factor,measurement_thetha,std_thetha,step):
    p_result=[]
    #print('')
    #print(plist, 'printing whats coming in')
    # motion update (prediction)
    p=[]
    for i in range(N):
        r = robot(plist[i][0],plist[i][1],plist[i][2])
        r.set_noise(0, 0, noise_factor*motions[1])
        p.append(r)

    if step > 1:
        p2 = []
        #print('print p before move', p)
   
        for i in range(N):
        
            p[i].move(motions[0],motions[1])
        
        #print(p, 'P after move', motions)
   
             
    # measurement update
        w = []
    
        for i in range(N):
        
            w.append(measurement_prob(p[i].sense(),measurements,noise_factor*motions[1],measurement_thetha,std_thetha,p[i].heading))
        #print(w, 'weights')   
        
    # resampling
        p3 = []
        h3=[]
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(p[index])
       
        p = p3
    
        p_result=[]

        for i in range(len(p)):
        
            p_result.append([p[i].x,p[i].y,p[i].heading])
        #print('final p', p_result)def next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER = None):
        #print('')
    
        return p_result
    else:
        return plist
# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(hunter_bot, target_bot, next_move_fcn, OTHER = None):
    """Returns True if your next_move_fcn successfully guides the hunter_bot
    to the target_bot. This function is here to help you understand how we 
    will grade your submission."""
    max_distance = 1.94 * target_bot.distance # 1.94 is an example. It will change.
    separation_tolerance = 0.02 * target_bot.distance # hunter must be within 0.02 step size to catch target
    caught = False
    ctr = 0
    #For Visualization
    import turtle
    window = turtle.Screen()
    window.bgcolor('white')
    chaser_robot = turtle.Turtle()
    chaser_robot.shape('arrow')
    chaser_robot.color('blue')
    chaser_robot.resizemode('user')
    chaser_robot.shapesize(0.3, 0.3, 0.3)
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.3, 0.3, 0.3)
    size_multiplier = 15.0 #change Size of animation
    chaser_robot.hideturtle()
    chaser_robot.penup()
    chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
    chaser_robot.showturtle()
    broken_robot.hideturtle()
    broken_robot.penup()
    broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
    broken_robot.showturtle()
    measuredbroken_robot = turtle.Turtle()
    measuredbroken_robot.shape('circle')
    measuredbroken_robot.color('red')
    measuredbroken_robot.penup()
    measuredbroken_robot.resizemode('user')
    measuredbroken_robot.shapesize(0.1, 0.1, 0.1)
    broken_robot.pendown()
    chaser_robot.pendown()
    #End of Visualization
    # We will use your next_move_fcn until we catch the target or time expires.
    while not caught and ctr < 1000:
        # Check to see if the hunter has caught the target.
        hunter_position = (hunter_bot.x, hunter_bot.y)
        target_position = (target_bot.x, target_bot.y)
        separation = distance_between(hunter_position, target_position)
        if separation < separation_tolerance:
            print "You got it right! It took you ", ctr, " steps to catch the target."
            caught = True

        # The target broadcasts its noisy measurement
        target_measurement = target_bot.sense()
        

        # This is where YOUR function will be called.
        turning, distance, OTHER = next_move_fcn(hunter_position, hunter_bot.heading, target_measurement, max_distance, OTHER)

        # Don't try to move faster than allowed!
        if distance > max_distance:
            distance = max_distance

        # We move the hunter according to your instructions
        hunter_bot.move(turning, distance)

        # The target continues its (nearly) circular motion.
        target_bot.move_in_circle()
        #Visualize it
        measuredbroken_robot.setheading(target_bot.heading*180/pi)
        measuredbroken_robot.goto(target_measurement[0]*size_multiplier, target_measurement[1]*size_multiplier-100)
        measuredbroken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-100)
        chaser_robot.setheading(hunter_bot.heading*180/pi)
        chaser_robot.goto(hunter_bot.x*size_multiplier, hunter_bot.y*size_multiplier-100)
        #End of visualization
        ctr += 1            
        if ctr >= 1000:
            print "It took too many steps to catch the target."
    return caught

def angle_trunc(a):
    """This maps all angles to a domain of [-pi, pi]"""
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

def get_heading(hunter_position, target_position):
    """Returns the angle, in radians, between the target and hunter positions"""
    hunter_x, hunter_y = hunter_position
    target_x, target_y = target_position
    heading = atan2(target_y - hunter_y, target_x - hunter_x)
    heading = angle_trunc(heading)
    return heading

def naive_next_move(hunter_position, hunter_heading, target_measurement, max_distance, OTHER):
    """This strategy always tries to steer the hunter directly towards where the target last
    said it was and then moves forwards at full speed. This strategy also keeps track of all 
    the target measurements, hunter positions, and hunter headings over time, but it doesn't 
    do anything with that information."""
    if not OTHER: # first time calling this function, set up my OTHER variables.
        measurements = [target_measurement]
        hunter_positions = [hunter_position]
        hunter_headings = [hunter_heading]
        OTHER = (measurements, hunter_positions, hunter_headings) # now I can keep track of history
    else: # not the first time, update my history
        OTHER[0].append(target_measurement)
        OTHER[1].append(hunter_position)
        OTHER[2].append(hunter_heading)
        measurements, hunter_positions, hunter_headings = OTHER # now I can always refer to these variables
    
    heading_to_target = get_heading(hunter_position, target_measurement)
    heading_difference = heading_to_target - hunter_heading
    turning =  heading_difference # turn towards the target
    distance = max_distance # full speed ahead!
    return turning, distance, OTHER

#target = robot(0.0, 10.0, 0.0, 2*pi / 30, 1.5)
#measurement_noise = .05*target.distance
#target.set_noise(0.0, 0.0, measurement_noise)
#hunter = robot(-10.0, -10.0, 0.0)

#print demo_grading(hunter, target, next_move)





