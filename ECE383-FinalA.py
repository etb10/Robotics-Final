from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.model import ik
from common import *
import math
import random
#this may be useful...
import numpy as np

##################### SETTINGS ########################
event = 'A'

#difficulty 
difficulty = 'easy'
#difficulty = 'medium'
#difficulty = 'hard'

omniscient_sensor = False

random_seed = 123451
random_seed = random.seed()

verbose = True

################ STATE ESTIMATION #####################

class MyObjectStateEstimator:
    """Your own state estimator that will provide a state estimate given
    CameraColorDetectorOutput readings."""
    def __init__(self):
        self.reset()
        
        self.color = [-1,-1,-1]
        #TODO: fill this in with your own camera model, if you wish
        self.Tsensor = None
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        self.w,self.h = 320,240
        self.fov = 90
        self.dmax = 7
        if event == 'A':
            #at goal post, pointing a bit up and to the left
            self.Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        elif event == 'B':
            #on ground near robot, pointing up and slightly to the left
            self.Tsensor = (so3.mul(so3.rotation([1,0,0],-0.10),so3.mul(so3.rotation([0,-1,0],math.radians(90)),cameraRot)),[-1.5,-0.5,0.25])
            self.w = 640
            self.h = 480
            self.dmax = 10
        else:
            #on ground near robot, pointing to the right
            self.Tsensor = (cameraRot,[-1.5,-0.5,0.25])
        self.dt = 0.02
        return
    def reset(self):
        self.radius_history = []
        self.position_history = []
        self.x_pos = []
        self.y_pos = []
        self.z_pos = []
        self.x_vel = []
        self.y_vel = []
        self.z_vel = []
    def update(self,observation):
        """Produces an updated MultiObjectStateEstimate given a CameraColorDetectorOutput
        sensor reading."""
        #TODO
        if(len(observation.blobs) > 0):
            
            index = len(observation.blobs) - 1
            x_cam = observation.blobs[index].x
            y_cam = observation.blobs[index].y
            w_cam = observation.blobs[index].w
            h_cam = observation.blobs[index].h
            
            color = observation.blobs[index].color
            if( (self.color[0] == color[0]) and  (self.color[1] == color[1]) and (self.color[2] == color[2])):
                #print "same color"
                pass
            else:
                self.color = color
                self.reset()
                #print "different color"
            
            # assumptions
            actual_radius = 0.103
            x_camera_center = self.w/2
            y_camera_center = self.h/2

            # pixel radius is average of height and width (divided by 2)
            pixel_radius = ((w_cam + h_cam)/float(2)) / float(2)
            
            # implement weighted average for calculating a smoother trajectory of the ball
            pixel_radius = self.getSmoothedRadius(pixel_radius)
            pixel_to_actual = actual_radius / pixel_radius
            
            # convert reading to actual 
            xPos = pixel_to_actual * (x_cam - x_camera_center)
            yPos = pixel_to_actual * (y_cam - y_camera_center)
            
            # zPos is pixel radius scaled
            zPos = 15 * (pixel_radius)**-1
            #zPos = math.sqrt(math.fabs(z_depth**2 - (xPos**2 + yPos**2)**2))
            
            worldPosition=se3.apply(self.Tsensor,[xPos, yPos, zPos])
            #print "worldPosition: ", worldPosition
            
            if(len(self.position_history) == 0):
                self.position_history = [worldPosition[0], worldPosition[1], worldPosition[2]]
            
            x = [worldPosition[0], worldPosition[1], worldPosition[2], 0,0,0]
            # TODO calculate velocity
            v = [0,0,0]
            for i in range(3):
                v[i] = (x[i] - self.position_history[i])/self.dt
                self.position_history[i] = x[i]
            
            #print "Initial V: ", v
            
            x[0] = self.getSmoothedPositionX(x[0], 7)
            x[1] = self.getSmoothedPositionY(x[1], 7)
            x[2] = self.getSmoothedPositionZ(x[2], 7)
            
            v[0] = self.getSmoothedVelocityX(v[0], 7)
            v[1] = self.getSmoothedVelocityY(v[1], 7)
            v[2] = self.getSmoothedVelocityZ(v[2], 7)
            
            
            
            #print "Smoothed V: ", v
            
            x = [worldPosition[0], worldPosition[1], worldPosition[2], v[0], v[1], v[2]]
        
            #print " " 
            #print "Percieved x: ", x
            
            # construct ObjectStateEstimate
            myObject = ObjectStateEstimate("guess position", x)
            
            #self.namething = self.namething + "a"
            #self.colorthing = self.colorthing + 0.03
            #a = self.colorthing
            
            # Draw projected location
            kviz.update_sphere("name",worldPosition[0],worldPosition[1],worldPosition[2],0.1)
            kviz.set_color("name",1,1,1)
        else:
            #print "No object measurements"
            # reset radius history
            self.radius_history = []
            self.position_history = []
            self.x_pos = []
            self.y_pos = []
            self.z_pos = []
            self.x_vel = []
            self.y_vel = []
            self.z_vel = []
            myObject = ObjectStateEstimate("guess position", [0,0,0,0,0,0])


        return MultiObjectStateEstimate([myObject])


    # return a radius value based on the previous values stored in self
    def getSmoothedRadius(self, new_radius):
        max_length = 2
        # if vector is empty, append our value and return new_radius as value
        if(len(self.radius_history) == 0):
            self.radius_history.append(new_radius)
            return new_radius
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.radius_history)):
            num = num + self.radius_history[i]*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.radius_history)
        num = num + new_radius*weight
        denom = denom + weight
        
        smoothed_radius = num / float(denom)
        # if the smoothed_radius is less than most recent value, don't let it do that
        #if(smoothed_radius < self.radius_history[len(self.radius_history) - 1]):
         #   smoothed_radius = self.radius_history[len(self.radius_history) - 1]
            
        # if the list is smaller than the max length, just append
        if(len(self.radius_history) < max_length):
            self.radius_history.append(smoothed_radius)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.radius_history) - 1):
                # shift values
                self.radius_history[i] = self.radius_history[i+1]
            self.radius_history[max_length-1] = smoothed_radius    
            
        return smoothed_radius
       
        
    def getSmoothedVelocityX(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.x_vel) == 0):
            self.x_vel.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.x_vel)):
            num = num + self.x_vel[i]*(i)
            denom = denom + (i)
        #for k in range(len(self.x_pos)):
         #   num = num + ((self.x_pos[len(self.x_pos)-1] - self.x_pos[k])/ (self.dt * (len(self.x_pos) - k)))*(k)
          #  denom = denom + (k)
        
        # factor in the new value
        weight = len(self.x_vel)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.x_vel) < max_length):
            self.x_vel.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.x_vel) - 1):
                # shift values
                self.x_vel[i] = self.x_vel[i+1]
            self.x_vel[max_length-1] = smoothed_value    
            
        return smoothed_value
        
        
    def getSmoothedVelocityY(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.y_vel) == 0):
            self.y_vel.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.y_vel)):
            num = num + self.y_vel[i]*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.y_vel)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.y_vel) < max_length):
            self.y_vel.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.y_vel) - 1):
                # shift values
                self.y_vel[i] = self.y_vel[i+1]
            self.y_vel[max_length-1] = smoothed_value    
            
        return smoothed_value
        
    def getSmoothedVelocityZ(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.z_vel) == 0):
            self.z_vel.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.z_vel)):
            # v(i+1) = v(i) - g * dt
            predicted_v = self.z_vel[i] - self.dt * 9.8 * (len(self.z_vel) - i)
            num = num + predicted_v*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.z_vel)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.z_vel) < max_length):
            self.z_vel.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.z_vel) - 1):
                # shift values
                self.z_vel[i] = self.z_vel[i+1]
            self.z_vel[max_length-1] = smoothed_value    
            
        return smoothed_value    
    
    
    def getSmoothedPositionX(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.x_pos) == 0):
            self.x_pos.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.x_pos)):
            # v(i+1) = v(i) - g * dt
            predicted_x = self.x_pos[i] + self.dt * self.x_vel[i] * (len(self.x_pos) - i) # - 9.8 * ((len(self.x_pos) - i) * self.dt)**2
            num = num + predicted_x*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.x_pos)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.x_pos) < max_length):
            self.x_pos.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.x_pos) - 1):
                # shift values
                self.x_pos[i] = self.x_pos[i+1]
            self.x_pos[max_length-1] = smoothed_value    
            
        return smoothed_value
    
    def getSmoothedPositionY(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.y_pos) == 0):
            self.y_pos.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.y_pos)):
            # v(i+1) = v(i) - g * dt
            predicted_x = self.y_pos[i] + self.dt * self.y_vel[i] * (len(self.y_pos) - i) # - 9.8 * ((len(self.z_pos) - i) * self.dt)**2
            num = num + predicted_x*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.y_pos)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.y_pos) < max_length):
            self.y_pos.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.y_pos) - 1):
                # shift values
                self.y_pos[i] = self.y_pos[i+1]
            self.y_pos[max_length-1] = smoothed_value    
            
        return smoothed_value
        
    def getSmoothedPositionZ(self, new_value, max_len):
        max_length = max_len
        # if vector is empty, append our value and return new_radius as value
        if(len(self.z_pos) == 0):
            self.z_pos.append(new_value)
            return new_value
        # if vector is smaller than MAX, append our value and continue to calculate mean

        # calculate mean of old values
        num = 0
        denom = 0
        for i in range(len(self.z_pos)):
            # v(i+1) = v(i) - g * dt
            predicted_x = self.z_pos[i] + self.dt * self.z_vel[i] * (len(self.z_pos) - i) #- 9.8 * ((len(self.z_pos) - i) * self.dt)**2
            num = num + predicted_x*(i)
            denom = denom + (i)
        
        # factor in the new value
        weight = len(self.z_pos)
        num = num + new_value*weight
        denom = denom + weight
        
        smoothed_value = num / float(denom)
        
        # if the list is smaller than the max length, just append
        if(len(self.z_pos) < max_length):
            self.z_pos.append(smoothed_value)
        # if vector is equal to MAX, then shift values down 1 spot, and put our value into top. calculate mean
        else:   # length is greater than or equal to max_length
            for i in range(len(self.z_pos) - 1):
                # shift values
                self.z_pos[i] = self.z_pos[i+1]
            self.z_pos[max_length-1] = smoothed_value    
            
        return smoothed_value
    

################### CONTROLLER ########################

class MyController:
    """Attributes:
    - world: the WorldModel instance used for planning.
    - objectStateEstimator: a StateEstimator instance, which you may set up.
    - state: a string indicating the state of the state machine. 

    TODO:
      decide what states you want in your state machine and how you want
      them to be named.
    """
    def __init__(self,world,robotController):
        self.world = world
        self.objectStateEstimator = None
        self.state = None
        self.robotController = robotController
        self.reset(robotController)
        self.lastx = [0,0,0]
        
    def reset(self,robotController):
        """Called on initialization, and when the simulator is reset.
        TODO: You may wish to fill this in with custom initialization code.
        """
        self.objectStateEstimator = MyObjectStateEstimator()
        self.objectEstimates = None
        self.state = 'waiting'
        #TODO: you may want to do more here to set up your
        #state machine and other initial settings of your controller.
        #The 'waiting' state is just a placeholder and you are free to
        #change it as you see fit.

        self.qdes = robotController.getCommandedConfig()
        self.initVis()
        pass

    def myPlayerLogic(self,
                      dt,
                      sensorReadings,
                      objectStateEstimate,
                      robotController):
        """
        TODO: fill this out to updates the robot's low level controller
        in response to a new time step.  This is allowed to set any
        attributes of MyController that you wish, such as self.state.
        
        Arguments:
        - dt: the simulation time elapsed since the last call
        - sensorReadings: the sensor readings given on the current time step.
          this will be a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading
          coming from the omniscient object sensor.  You will not need to
          use raw sensor data directly, if you have a working state estimator.
        - objectStateEstimate: a MultiObjectStateEstimate class (see
          stateestimation.py) produced by the state estimator.
        - robotController: a SimRobotController instance giving access
          to the robot's low-level controller.  You can call any of the
          methods.  At the end of this call, you can either compute some
          PID command via robotController.setPIDCommand(), or compute a
          trajectory to execute via robotController.set/addMilestone().
          (if you are into masochism you can use robotController.setTorque())
        """
        #these are pulled out here for your convenience
        qcmd = robotController.getCommandedConfig()
        vcmd = robotController.getCommandedVelocity()
        qsns = robotController.getSensedConfig()
        vsns = robotController.getSensedVelocity()

        #setting a PID command can be accomplished with the following
        #robotController.setPIDCommand(self.qdes,[0.0]*7)

        #queuing up linear interpolation can be accomplished with the following
        #dt = 0.5   #how much time it takes for the robot to reach the target
        #robotController.appendLinear(self.qdes,dt)

        #queuing up a fast-as possible interpolation can be accomplished with the following
        #robotController.addMilestone(self.qdes)


        # get the RobotModel object from the robotController
        robot = robotController.model()
        solved_location = 0
        
        
        # Check if a ball is within the space "reachable" by our robot.
        for o in self.objectEstimates.objects:
            x = [o.x[0],o.x[1],o.x[2]]
            v = [o.x[3],o.x[4],o.x[5]]
            
            #print "Object Position: ", x
            #print "Object Velocity: ", v
            
            # if the ball is off the table, don't evaluate
            if( (x[2] < (-.5)) ):
                 #print o.name, " is off the table"
                 pass
            # if ball is in the corner, 
            # if ball is moving backwards,
            # if ball's velocity is below threshold
            elif( x[0] >= 3 or o.x[3] > 0 or vectorops.norm(v) < 0.5):  
                #print o.name, " is not valid"
                pass
            else:
                #print o.name, " is moving"
                # define velocity for these calculations
                gravity = 9.8
                for i in range(20):
                    # set time interval
                    t = 0.05
                    # check if the trajectory falls within robot's arm 
                    [x,v] = self.predict_next_location(x, v, t, gravity)
                    inBounds = self.withinReach(x, -2.5, -1.7, -1.2, 1.2, 0.09, 1.2)
                    if(inBounds == 1):
                        #print o.name, " is projected to score. Block this location at:"
                        #print x[0], x[1], x[2]
                        (q,solved) = self.ik_solver(robot, x, qsns)
                        self.state = "blocking"
                        solved_location = 1
                        if(solved):
                            #print "setting milestone for blocking robot"
                            q[robot.numLinks()-1] = 0
                            self.lastx = x
                            robotController.setMilestone(q)
                            break;
                    # predict the next location
        # end loop
        
        # state machine to determine which state the robot should enter
        # requires 4 misses to enter blocking mode
        if(solved_location == 0):
            if(self.state == "blocking"):
                self.state = "blocking1"
            elif(self.state == "blocking1"):
                self.state = "blocking2"
            elif(self.state == "blocking2"):
                self.state = "blocking3"
            else: # if waiting or blocking 3
                self.state = 'waiting'


        # if the robot doesn't have any obstacles to block, go back to initial position
        if self.state == 'waiting':
            #print qsns
            #print "waiting"
            if(self.lastx[1] <= -0.29):
                if(self.lastx[1] < 0.3):
                    xdes = [-1.5, -0.3, 0.5]
                else:
                    xdes = [-1.5, -0.3, 0.2]
            elif(self.lastx[1] >= 0.29):
                if(self.lastx[1] < 0.3):
                    xdes = [-1.5, 0.3, 0.5]
                else:
                    xdes = [-1.5, 0.3, 0.2]
            #if(qsns[1] > 0.4 or qsns[1] < -0.4):
             #   xdes = [-1.2, 0, 0.5]
            else:
                xdes = [-1.2, 0, 0.9]
            
            self.lastx = xdes
            (q,solved) = self.ik_solver(robot, xdes, qsns)
            q[robot.numLinks()-1] = 0
            if(solved):
                #print "setting milestone for waiting robot"
                robotController.setMilestone(q)
        elif(self.state == "reset"): # if you're blocking, the controller was already programmed so just wait it out
            pass
        else:
            #print "blocking"
            pass
        return
        
    def loop(self,dt,robotController,sensorReadings):
        """Called every control loop (every dt seconds).
        Input:
        - dt: the simulation time elapsed since the last call
        - robotController: a SimRobotController instance. Use this to get
          sensor data, like the commanded and sensed configurations.
        - sensorReadings: a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading coming
          from the omniscient object sensor.
        Output: None.  However, you should produce a command sent to
          robotController, e.g., robotController.setPIDCommand(qdesired).

        """
        multiObjectStateEstimate = None
        if self.objectStateEstimator and 'blobdetector' in sensorReadings:
            multiObjectStateEstimate = self.objectStateEstimator.update(sensorReadings['blobdetector'])
            self.objectEstimates = multiObjectStateEstimate
            #multiObjectStateEstimate is now a MultiObjectStateEstimate (see common.py)
        if 'omniscient' in sensorReadings:
            omniscientObjectState = OmniscientStateEstimator().update(sensorReadings['omniscient'])
            #omniscientObjectStateEstimate is now a MultiObjectStateEstimate (see common.py)
            multiObjectStateEstimate  = omniscientObjectState
            #if you want to visualize the traces, you can uncomment this
            self.objectEstimates = multiObjectStateEstimate

        self.myPlayerLogic(dt,
                           sensorReadings,multiObjectStateEstimate,
                           robotController)

        self.updateVis()
        return

    def initVis(self):
        """If you want to do some visualization, initialize it here.
            TODO: You may consider visually debugging some of your code here, along with updateVis().
        """
        #kviz.add_ghost()
        pass
    
    def updateVis(self):
        """This gets called every control loop.
        TODO: You may consider visually debugging some of your code here, along with initVis().

        For example, to draw a ghost robot at a given configuration q, you can call:
          kviz.add_ghost()  (in initVis)
          kviz.set_ghost(q) (in updateVis)

        The current code draws gravity-inflenced arcs leading from all the
        object position / velocity estimates from your state estimator.  Event C
        folks should set gravity=0 in the following code.
        """
        #print "Start of a new time increment"
        if self.objectEstimates:
            for o in self.objectEstimates.objects:
                
                x = [o.x[0],o.x[1],o.x[2]]
                v = [o.x[3],o.x[4],o.x[5]]
                
                # draw the actual spheres
                kviz.update_sphere("object_est"+str(o.name),o.x[0],o.x[1],o.x[2],0.03)
                kviz.set_color("object_est"+str(o.name),o.name[0],o.name[1],o.name[2])
                
                
                
                # if the ball is off the table, don't evaluate
                if( (x[2] < (-.5)) ):
                     #print o.name, " is off the table"
                     pass
                # if ball is in the corner, 
                # if ball is moving backwards,
                # if ball's velocity is below threshold
                elif( x[0] >= 3 or o.x[3] > 0 or vectorops.norm(v) < 0.5):  
                    #print o.name, " is not valid"
                    pass
                else:
                    #draw a point
                    #draw an arc
                    
                    trace = []
                    #print o.name, " x_position", o.x[0], " ", o.x[1], " ", o.x[2]
                    gravity = 9.8
                    for i in range(40):
                        # set time interval
                        t = 0.05
                        
                        # append to trace
                        trace.append(x)
                        
                        # predict the next location
                        [x,v] = self.predict_next_location(x, v, t, gravity)
    
                    kviz.update_polyline("object_trace"+str(o.name),trace);
                    kviz.set_color("object_trace"+str(o.name),o.name[0],o.name[1],o.name[2])



    # Predict the next location of the ball 
    # @x: current ball location
    # @v: current ball velocity
    # @t: time stamp
    #
    # return: updated (x,v)
    def predict_next_location(self, x, v, t, gravity):
        # update estimated x value
        x_temp = vectorops.sub(vectorops.madd(x,v,t), [0,0,0.95*gravity*t*t])
                    
                    
        # if x_temp[2] is less than 0, then extrapolate exact time where t would've hit 0.03
        if(x_temp[2] < 0.03):
            # extrapolate exact time where t would've hit
            t = t * ((x[2]-0.03)/(x[2] - x_temp[2]))
            # update x position
            x = vectorops.sub(vectorops.madd(x,v,t), [0,0,0.95*gravity*t*t])
            # set the z coordinate to 0 (to be sure)
            x[2] = 0.03
            # update velocity value
            val = 0.7
            v[0] = v[0] * val
            v[1] = v[1] * val
            v[2] = (v[2] * -1 * val) - .95*gravity*t
        else:
            x = x_temp
            v = vectorops.sub(v, [0,0,0.95*gravity*t]) # t is updated based on above computations

        return (x,v)



    def withinReach(self, x_vector, x_lower, x_upper, y_lower, y_upper, z_lower, z_upper):
        x = x_vector[0]
        y = x_vector[1]
        z = x_vector[2]

        # check values against their domains
        if( (x < x_lower) or (x > x_upper) or (y < y_lower) or (y > y_upper) or (z < z_lower) or (z > z_upper) ):
            return 0
        else:
            return 1


    def ik_solver(self, robot, x, q):
        # check over all the links in the 
        for i in range(5):   
            # Objective: end effector on center of target, end of final link along target axis
            objs = ik.objective(robot.link(6-i),local=[(0,0,0)],world=[x]);
            s = model.ik.solver(objs);
            
            # checks for solution with 100 potential tries (Restarts)
            numRestarts = 100;
            solved = False;
            for i in range(numRestarts):
                robot.setConfig(q);
                ## GLOBAL IMPROVEMENTS
                #robot.setConfig(qglobal);
    
                res = s.solve();
                
                ##now solve
                if (res) :
                    solved = True;
                    # print "IK success!"
                    ## EFFECIENCY TRACKING
                    #suc = suc + 1;
                    #print "success:", suc, "total:", tot;
                    
                    ## now the robot model's configuration is changed, and you need to
                    ## extract it. Your errpos and erraxis values should be very small
                    ## Get world axis for end effector
                    qseed = robot.getConfig();
                    ## GLOBAL IMPROVEMENTS
                    #qglobal = qseed;
                    return (qseed, solved)
                
        # if never solved, print failure
        # print "IK failure... returning best solution found"
        return (q, solved)
        ## EFFECIENCY TRACKING

