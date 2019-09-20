# ----------
# Part Two
#
# Now we'll make the scenario a bit more realistic. Now Traxbot's
# sensor measurements are a bit noisy (though its motions are still
# completetly noise-free and it still moves in an almost-circle).
# You'll have to write a function that takes as input the next
# noisy (x, y) sensor measurement and outputs the best guess 
# for the robot's next position.
#
# ----------
# YOUR JOB
#
# Complete the function estimate_next_pos. You will be considered 
# correct if your estimate is within 0.01 stepsizes of Traxbot's next
# true position. 
#
# ----------
# GRADING
# 
# We will make repeated calls to your estimate_next_pos function. After
# each call, we will compare your estimated position to the robot's true
# position. As soon as you are within 0.01 stepsizes of the true position,
# you will be marked correct and we will tell you how many steps it took
# before your function successfully located the target bot.

# These import steps give you access to libraries which you may (or may
# not) want to use.
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

# This is the function you have to write. Note that measurement is a 
# single (x, y) point. This function will have to be called multiple
# times before you have enough information to accurately predict the
# next position. The OTHER variable that your function returns will be 
# passed back to your function the next time it is called. You can use
# this to keep track of important information over time.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    N=100
   
    distance_noise_factor =0.05
    if OTHER == None:
        
        p_initial = []
               
        for i in range(N):
             t1 = random.uniform(0,2*pi)
                         
             x1 = measurement[0]+ random.gauss(0,distance_noise_factor*random.uniform(1,5))
                     
             y1 = measurement[1]+ random.gauss(0,distance_noise_factor*random.uniform(1,5))
             p_initial.append([x1,y1,t1])
        #print (p_initial, 'p right irst')
     
        last_turn =0.0
        thetha_list=[]
        thetha=0.0
        turn_list=[]
        updated_measurement=[0.0,0.0]
        last_updated =[0.0,0.0]
        turn_avg=0.0
        thetha_avg=0.0
        step=0
        t2=0
        d2=0
        turning_list=[]
    
        
        OTHER = [[0.0,0.0], thetha,p_initial,step,[0.0,0.0,0.0],thetha_list, turn_list]

  
    distance = distance_between(measurement, [OTHER[4][0],OTHER[4][1]])
    step=OTHER[3]
    last_updated=OTHER[4]
    last_thetha = OTHER[1]
    p_last = OTHER[2]
    thetha_list = OTHER[5]
    turn_list = OTHER[6]
  
    measurement_orient = atan2(measurement[1] -OTHER[0][1], measurement[0] -OTHER[0][0])
    thetha_list.append(measurement_orient)
    std_thetha=0.0
    thetha_avg=sum(thetha_list)/len(thetha_list)
    for i in range(len(thetha_list)):
        std_thetha = std_thetha + (thetha_list[i]-thetha_avg)**2
  
    std_thetha = std_thetha/max(1,len(thetha_list)-1)
    
    thetha = atan2(measurement[1] -OTHER[4][1],measurement[0] -OTHER[4][0])
    
    turn = thetha - last_updated[2]
    if turn < -pi:
        turn = 2*pi + turn
    elif turn > pi:
        turn = turn - 2*pi
    
    motions=[turn,distance]
    step = step + 1
    #print('The measurement is', measurement, 'thetha', thetha, 'turn', turn, 'distance', distance)

    result=robot()
    result = particle_filter(motions,measurement, N,p_last,distance_noise_factor,measurement_orient, std_thetha,step)
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
   
    OTHER = [measurement,thetha,result,step,updated_measurement,thetha_list,turn_list]
    return xy_estimate, OTHER
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
    '''
    for i in range(len(p)):
        if p[i].heading + motions[0] > pi:
            p[i].heading = -pi
        elif p[i].heading + motions[0] < -pi:
            p[i].heading = pi
    '''        
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
def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.1, 0.1, 0.1)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.1, 0.1, 0.1)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.1, 0.1, 0.1)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        #print('The robot is actually at', true_position, target_bot.heading)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print "You got it right! It took you ", ctr, " steps to localize."
            localized = True
        if ctr == 1000:
            print "Sorry, it took you too many steps to localize the target."
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized

# This is a demo for what a strategy could look like. This one isn't very good.
def naive_next_pos(measurement, OTHER = None):
    """This strategy records the first reported position of the target and
    assumes that eventually the target bot will eventually return to that 
    position, so it always guesses that the first position will be the next."""
    if not OTHER: # this is the first measurement
        OTHER = measurement
    xy_estimate = OTHER 
    return xy_estimate, OTHER

# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
#test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
#measurement_noise = 0.05 * test_target.distance 
#test_target.set_noise(0.0, 0.0, measurement_noise)

#demo_grading(estimate_next_pos, test_target)









