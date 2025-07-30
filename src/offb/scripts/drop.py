#!/usr/bin/env python3
# coding:utf-8
import rospy
import serial
from std_msgs.msg import Int8
def callback(data):
    print("good")
    if data.data==1:
        ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.1)
        if ser.isOpen():
            print("OK")
            ser.write("1".encode("utf-8"))
            #ser.write(b'1')
        else:
            print("NO")    
        ser.close()  

    if data.data==2:
        ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.1)
        if ser.isOpen():
            print("OK")
            ser.write("2".encode("utf-8"))
            #ser.write(b'1')
        else:
            print("NO")    
        ser.close()  
    
    if data.data==3:
        ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.1)
        if ser.isOpen():
            print("OK")
            ser.write("3".encode("utf-8"))
            #ser.write(b'1')
        else:
            print("NO")    
        ser.close()      

    
if __name__=="__main__":
    rospy.init_node("drop",anonymous=True)
    sub=rospy.Subscriber("/drop",Int8,callback)
    rospy.spin()

  