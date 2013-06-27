#!/usr/bin/env python
import roslib
roslib.load_manifest('openni_tracker_multi')
import rospy    
import tf
import math
import time
from std_msgs.msg import String
from collections import deque
from numpy import *

# same file as older joint_tf_tracker.py- renamed for clarity

# take a simple average
def average(numList):
    sum = 0
    for i in numList:
        sum += i
    if len(numList) > 0:
        return sum/len(numList)
    else:
        return 0

# add each element of a 3-element array to a specified deque
# assumes all deques are of the same size
def addToMovingWindow(elements, windowSize, *deque):
    if len(deque[0][0]) > windowSize-1:         # remove oldest element from the deques if bigger than a certain windowSize
        for i in deque[0]:
            i.popleft()

    for i in range(len(deque[0])):
#        print i, "elements length:", len(elements), "deque length:", len(deque[0])
        deque[0][i].append(elements[i])

    return deque

# find the number label of each person the kinect(s) see
# takes the output of listener.getFrameStrings(), a list of frames as strings
def getSkeletonList(tfFrames):
    skeletonList = []
    for frame in tfFrames:
        # find every frame with the label "torso" and return the number in the name "torso_#"           TODO - need to change this to accomodate naming convention
        if (frame.find("torso") != -1):
            skeletonList.append(int(frame.split("_")[1]))
    if (len(skeletonList) == 1):
        print "Found", str(len(skeletonList)), "person"
    else:
        print "Found", str(len(skeletonList)), "people"
    print "skeletonList", skeletonList
    return skeletonList

def handleTracking(deviceNum, listener):
    print "tracking skeleton", s, "with index", sIndex, "using kinect", deviceNum                    

    kinectTf = '/kinect'+str(deviceNum)+'_depth_frame'

    frameNameModifier = ""                      # modify appropriately to match joint naming convention
    if deviceNum != 1:
        frameNameModifier = str(deviceNum)
    
    (transTorso,rotTorso) = listener.lookupTransform(kinectTf, '/torso'+frameNameModifier+'_'+str(s), rospy.Time(0))
    # smooth out the position data
    addToMovingWindow(transTorso, windowSize, currentTorsoPosDeques[sIndex])
    posSmooth = [average(currentTorsoPosDeques[sIndex][0]), average(currentTorsoPosDeques[sIndex][1]), average(currentTorsoPosDeques[sIndex][2])]
  	
  	# calculate the velocity
    velSmooth = [posSmooth[0]-prevTorsoPos[sIndex][0], posSmooth[1]-prevTorsoPos[sIndex][1], posSmooth[2]-prevTorsoPos[sIndex][2]]

    # average the orientations of the shoulders and the hips
    (transNeck,rotNeck) = listener.lookupTransform(kinectTf, '/neck'+frameNameModifier+'_'+str(s), rospy.Time(0))                                 # neck
    (transLeftShoulder,rotLeftShoulder) = listener.lookupTransform(kinectTf, '/left_shoulder'+frameNameModifier+'_'+str(s), rospy.Time(0))        # left shoulder
    (transRightShoulder,rotRightShoulder) = listener.lookupTransform(kinectTf, '/right_shoulder'+frameNameModifier+'_'+str(s), rospy.Time(0))     # right shoulder
    (transLeftHip,rotLeftHip) = listener.lookupTransform(kinectTf, '/left_hip'+frameNameModifier+'_'+str(s), rospy.Time(0))                       # left hip
    (transRightHip,rotRightHip) = listener.lookupTransform(kinectTf, '/right_hip'+frameNameModifier+'_'+str(s), rospy.Time(0))                    # right hip
    
    tVec = matrix(transNeck)-matrix(transTorso)                     # torso vector (torso to neck)
    sVec = matrix(transRightShoulder)-matrix(transLeftShoulder)     # shoulder vector (left to right)
    hVec = matrix(transRightHip)-matrix(transLeftHip)               # hip vector (left to right)
    
    oVec = (cross(tVec,sVec)+cross(tVec,hVec))                      # sum of (tVec cross sVec) and (tVec cross hVec)
    oVec = oVec/linalg.norm(oVec)                                   # divide by the norm to get the unit vector (orientation is always a unit vector)
    oVec = oVec[0]                                                  # change from matrix to vector

    # visualization section ######################
    # broadcast transforms for visualization (can comment out later for speed)
    
    oVecScaled = [x*5 for x in oVec]                         # scale the orientation so it's easier to see on rviz
    velSmoothScaled = [x*10 for x in velSmooth]              # scale the velocity vector so it's easier to see in rviz
    
    velocityTfBroadcasters[sIndex].sendTransform([velSmoothScaled[1], velSmoothScaled[2], velSmoothScaled[0]], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "velocity_"+str(s), "torso_"+str(s))
    
    orientationTfBroadcasters[sIndex].sendTransform([0,0,-1], tf.transformations.quaternion_from_euler(oVecScaled[0],oVecScaled[1],oVecScaled[2]), rospy.Time.now(), "orientation_"+str(s), "torso_"+str(s))

    ###############################################

    # smooth out the orientation data
    addToMovingWindow(oVec, windowSize, currentOrientationDeques[sIndex])
    oVecSmooth = [average(currentOrientationDeques[sIndex][0]), average(currentOrientationDeques[sIndex][1]), average(currentOrientationDeques[sIndex][2])]
    
    dataLog = str(s), str(posSmooth[0]), str(posSmooth[1]), str(posSmooth[2]), str(velSmooth[0]), str(velSmooth[1]), str(velSmooth[2]), str(oVecSmooth[0]), str(oVecSmooth[1]), str(oVecSmooth[2])
    rospy.loginfo(dataLog)
    pub.publish(String(dataLog))
    
    # debug prints                
#                print "Skeleton ID:", s
#                print "position", posSmooth
#                print "velocity", velSmooth
#                print "orientation", oVec
    print "----------------------------- \n"
   
    prevTorsoPos[sIndex] = posSmooth


if __name__ == '__main__':
    rospy.init_node('movement_tracker')		            # start the node
    pub = rospy.Publisher('joint_state', String)		# start the rostopic to be published to
    listener = tf.TransformListener()                   # start the transform listener

    velocityTfBroadcasters = []                         # vector of transforms that represent users' velocities
    orientationTfBroadcasters = []                      # vector of transforms that represent users' orientations

    prevSkeletonList = []                               # tracks users that are entering/leaving frame to create position and rotation arrays appropriately
    
    prevTorsoPos = []                                   # stores 3-element vectors of previous positions for instantaneous velocity calculations
    currentTorsoPosDeques = []                          # stores 3-element tuples (each element is a deque of up to 10 elements) of positions for smoothed positions (arrays of x, y and z)
    currentOrientationDeques = []                          # stores 4-element tuples (each element is a deque of up to 10 elements) of rotations for smoothed positions (arrays of qx, qy, qz, and qw)

    timeWindow = 1                                      # seconds
    windowSize = timeWindow*10                          # 10 frames per second

    time.sleep(5)                                            # wait for connections to devices to start up

    while not rospy.is_shutdown():
        try:
            # TODO - need to change all references from openni_depth_frame to kinect#_depth_frame
            # TODO - will probably also resolve the issue of multiple kinects going into here, currently triggering a can't-read error
            skeletonList = getSkeletonList(listener.getFrameStrings())
            
            # TODO - assumes that the skeletonList never loses members, only gains them... valid assumption? (if it is, that probably needs to be changed)
            # TODO - scan over the currentTorsoPosDeques and currentOrientationDeques to remove skeletons that have stopped moving entirely (not actually present anymore)
                # maybe use a count of some kind so that they are removed only after ~3-5 seconds of nonactivity instead of 1?
            
            for s in skeletonList:
                if s not in prevSkeletonList:
                    prevTorsoPos.append([0,0,0])
                    currentTorsoPosDeques.append((deque([]), deque([]), deque([])))                         # tuple of 3 deques for x, y, and z positions
                    currentOrientationDeques.append((deque([]), deque([]), deque([])))                      # tuple of 3 deques for rpy rotations
                    velocityTfBroadcasters.append(tf.TransformBroadcaster())                                # add a new broadcaster for representing velocities
                    orientationTfBroadcasters.append(tf.TransformBroadcaster())                             # add a new broadcaster for representing orientations

                # TODO move some of this up into handleTracking?

                # position in the list does NOT necessarily correspond to the skeleton number
                sIndex = skeletonList.index(s)
                print "sIndex", sIndex, "s", s
                if listener.canTransform('/kinect1_depth_frame','/torso_'+str(s), rospy.Time(0)):
                    handleTracking(1, listener)
                if listener.canTransform('/kinect2_depth_frame','/torso2_'+str(s), rospy.Time(0)):          # remember the joint naming conventions change slightly after the first kinect!
                    handleTracking(2, listener)
                
#                if listener.canTransform('/kinect1_depth_frame','/torso_'+str(s), rospy.Time(0)):
#                    try:
#                        listener.lookupTransform('/kinect1_depth_frame', '/torso_'+str(s), rospy.Time(0))
#                        print "kinect1 can do the transform"
#                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                        continue
#                    print "tracking person seen by kinect 1"
#                    handleTracking(1, listener)
#                elif listener.canTransform('/kinect2_depth_frame','/torso_'+str(s), rospy.Time(0)):
#                    try:
#                        listener.lookupTransform('/kinect1_depth_frame', '/torso_'+str(s), rospy.Time(0))
#                        print "kinect2 can do the transform"
#                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                        continue
#                    print "tracking person seen by kinect 2"
#                    handleTracking(2, listener)
                
            # put any new skeletons into the old skeleton list
            prevSkeletonList = skeletonList
            rospy.Rate(10.0).sleep()            # checking at 10 hz
          	  
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Error: something is wrong with the transform."
            rospy.Rate(1.0).sleep()             # checks every second to see if the data is coming through
            continue
