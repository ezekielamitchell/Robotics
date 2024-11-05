#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import rospy

rospy.init_node('responsive_arm')

#Constants
nbPCAServo=16 

#Parameters
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]
MIN_ANG  =[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
MAX_ANG  =[180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180]

#Objects
pca = ServoKit(channels=16)

# function init 
def init():

    pca.servo[5].angle=90 # center arm0
    pca.servo[8].angle=90 # center arm0
    pca.servo[9].angle=45 # center base
    time.sleep(0.01)

    for i in range(3): # run three iterations of arm movement
        for j in range(MIN_ANG[i],MAX_ANG[i],1):
            print("Send angle {} to Servo {}".format(j,i))
            pca.servo[0].angle = j
            pca.servo[4].angle = j
            time.sleep(0.01)
        for j in range(MAX_ANG[i],MIN_ANG[i],-1):
            print("Send angle {} to Servo {}".format(j,i))
            pca.servo[0].angle = j
            pca.servo[4].angle = j
            time.sleep(0.01)
        pca.servo[0].angle=None #disable channel
        pca.servo[4].angle=None #disable channel
        time.sleep(0.5)



if __name__ == '__main__':
    init()
    
    # main()
