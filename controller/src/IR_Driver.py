#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
from circular_queue import Circular_Queue
from controller.msg import IR_Data


class ROVER_IR_ARRAY:

    def __init__(self, pins):
        self.queue = Circular_Queue(50)
        for pin in pins:
            GPIO.setup(pin, GPIO.IN)
            GPIO.add_event_detect(pin, GPIO.RISING, callback=self.callback)

    def callback(self, channel):
        self.queue.enqueue(channel%4)

    def getDirection(self):
        return 90 * self.queue.average()



def main():
    GPIO.setmode(GPIO.BCM)
    rover_array = ROVER_IR_ARRAY([22, 23, 24, 25])

    pub = rospy.Publisher("ir_array", IR_Data, queue_size=1)
    rospy.init_node('ir_driver')
    r = rospy.Rate(10)

    msg = IR_Data()

    while not rospy.is_shutdown():
        msg.direction = rover_array.getDirection()
        print(rover_array.getDirection())
        pub.publish(msg)
        r.sleep()

    GPIO.cleanup()


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException: pass