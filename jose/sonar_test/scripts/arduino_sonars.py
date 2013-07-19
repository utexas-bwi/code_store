#!/usr/bin/env python:
import roslib; roslib.load_manifest('sonar_test')
import rospy

import re #yeah, we're using REGEX.
import serial
from std_msgs.msg import String
from sonar_test.msg import SegbotSensorsStatus

''' new stuff'''
import time, struct
from threading import Thread


'''
SerialMonitor operates as a separate thread that 
receives incoming light level data from the Arduino Mega.  Since
light level messages are transmitted at 10Hz, this
thread loops at 20Hz.
'''

distanceRE = re.compile(r'(\d+)cm') #used to find all the distances. The first 5 should belong to the ones in the top and the last two should belong to the ones used for cliff detection

class SerialMonitor(Thread): # SerialMonitor extends Thread
  ''' 
  The constructor of SerialMonitor.

  Parameters:
   - ser: The serial port object
   - pub: The object through which LightLevel messages may be published
  '''
  def __init__(self, ser, pub):
    Thread.__init__(self, name="SerialMonitor") # Call superclass constructor
    self.ser = ser
    self.seqno = 0;

  def run(self):
    rospy.loginfo(rospy.get_name() + " SerialMonitor: Thread starting.")
    
    while not rospy.is_shutdown():
      serial_msg = ser.readline()
      rospy.loginfo("serial monitor: " + serial_msg)
      
      #this is an array of distances gathered from the 7 sensors. The last two abelong to cliff detection
      distances = distanceRE.findall(serial_msg)
      
      #publish them in meters
      msg = SegbotSensorsStatus()
      msg.sonar1 = 0.01*float(distances[0])
      msg.sonar2 = 0.01*float(distances[1])
      msg.sonar3 = 0.01*float(distances[2])
      msg.sonar4 = 0.01*float(distances[3])
      msg.sonar5 = 0.01*float(distances[4])
      msg.cliff1 = 0.01*float(distances[5])
      msg.cliff2 = 0.01*float(distances[6])
      pub.publish(msg)

      time.sleep(0.05)
      self.seqno += 1
      
    
    ser.close()


def callback(cmd):
  ''' The loginfo command might be useful, but this is commented because it's specific to that program.'''
  #data = [cmd.cmd]
  #bytes = struct.pack("<b", *data)
  #numTX = ser.write(bytes)
  #rospy.loginfo(rospy.get_name() + " - Sent command (%i bytes)", numTX)
  # Format chars: http://docs.python.org/library/struct.html#format-characters

                        
if __name__ == '__main__':
  rospy.init_node('talker')

  port = '/dev/ttyACM0'
  baud = 115200
  rospy.loginfo("Serial(USB) port = %s", port)
  rospy.loginfo("Serial baud = %i", baud)

  ser = serial.Serial(port,baud)

  if (ser):
    rospy.loginfo("Serial port " + ser.portstr + " opened.")
  else:
    rospy.logerr("Unable to open serial port")
    sys.exit()

  rospy.loginfo("Waiting 2s for Arduino to initialize...")
  time.sleep(2)
	
	#what does this do?
  ser.flushInput()

    # Create and start a serial monitor thread.
    # This is for receiving light level information.
  pub = rospy.Publisher('sonarReadings', SegbotSensorsStatus)
  smThread = SerialMonitor(ser, pub)
  smThread.start()

  rospy.spin()
