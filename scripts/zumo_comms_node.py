#!/usr/bin/env python

import rospy
import sys

from geometry_msgs.msg import Twist
from bupimo_msgs.msg import ZumoProximities
from std_msgs.msg import String
from std_msgs.msg import Float64

from SerialDataGateway import SerialDataGateway

class zumo_comms_node(object):
        
	def ReceivedLine(self,  line):
                #print("LINE: " + line)
                words = line.split("\t")
                #rospy.loginfo("LINE: " + line)

                if len(line) > 0:
                        currentLinearVelMsg = Float64()
                        try:
                                currentLinearVelMsg.data = float(words[3])
                        except: # Catch all 
                                exception = sys.exc_info()[0]
                                print("Exception in zumo_comms_node: " + str(exception))                               

                        self.currentLinearVelPublisher.publish(currentLinearVelMsg);

                                
        def TwistCallback(self, twistMessage):
                #print("Command Recieved!")
                
                v = twistMessage.linear.x 
                w = twistMessage.angular.z
                t = twistMessage.angular.y

                message = str(v) + ':' + str(w) + ':'+str(t)
                self.SerialDataGateway.Write(message)
            
        def __init__(self, port="/dev/serial0", baudrate=57600):
                rospy.init_node('zumo_comms_node')
                    
                port = rospy.get_param("~port", "/dev/serial0")
                baudRate = int(rospy.get_param("~baudRate", 57600))
                    
                rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
                    
                # subscriptions
                rospy.Subscriber("cmd_vel", Twist, self.TwistCallback)
                    
                self.currentLinearVelPublisher = rospy.Publisher('currentLinearVel', Float64, queue_size = 1)
                
                # CREATE A SERIAL_DATA_GATEWAY OBJECT 
                # pass it a function pointer to _HandleReceivedLine
                self.SerialDataGateway = SerialDataGateway(port, baudRate,  self.ReceivedLine)
                    
        def Start(self):
                self.SerialDataGateway.Start()
                            
        def Stop(self):
                self.SerialDataGateway.Stop()
                                                    
if __name__ == '__main__':
        zumo_comms_node = zumo_comms_node()
        try:
                zumo_comms_node.Start()
                rospy.spin()
                
        except rospy.ROSInterruptException:
                zumo_comms_node.Stop()
                                                    
