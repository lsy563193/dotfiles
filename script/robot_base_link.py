#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib
import sys
roslib.load_manifest('mybot')  
import rospy  
import tf.transformations 
import geometry_msgs.msg
from mybot.msg import sensor,peripheral
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose,Point,Quaternion,Twist
import time
import binascii
import os
import math
from math import radians,degrees,cos,sin,pi
import serial
import threading 
#import pdb
#_DEBUG=='True'

made_table=0
crc8_table=[0x00]*256

def init_CRC():
	global made_table
	global crc8_table
	if made_table == 0:
		crc = 0x00
		for i in range(0,256):
			crc = i
			for j in range(0,8):
				if (crc & 0x80) != 0:
					crc = (crc << 1) ^ 0x07
				else:
					crc = (crc << 1) ^ 0x00
			crc8_table[i] = crc & 0xFF
		made_table = 1

def crc8(crc, m):
	global made_table
	global crc8_table
	if made_table == 0:
		init_CRC()
	crc = crc8_table[crc ^ m]
	crc = crc & 0xff
	return crc

def calBufCRC8(inBuf, inBufSz):
	crc_base = 0x00
	crc_res = "00"
	for i in range(inBufSz):
		Buf = int(inBuf[2*i:2*i+2],16)
		crc_base = crc8(crc_base, Buf)
	crc_res = '%02x' % crc_base
	return crc_res

class SerialThread(threading.Thread):
	def __init__(self,name = 'SerialThread'):
		threading.Thread.__init__(self,name = name)
		self.rStreamLen=39
		self.isSend = False
		self.isRecei = False
		self.thread_stop = False
		self.ser = serial.Serial()
		self.ser.port='/dev/ttyS3'
		self.ser.baudrate=115200
		self.ser.timeout=None
		self.ser.open()
		self.isReaded = False
		self.stopSend = False
		self.ReceiEvent = threading.Event()
		self.SendEvent = threading.Event()
		self.SendEvent.set()
		if not self.ser.readable():
			print 'open serial port %s fail !'%self.ser.port
			sys.exit()
		else:
			print 'open serial port %s success !'%self.ser.port

	def run(self):
		while not self.thread_stop:
			try:
				self.isReaded = False
				h1 = binascii.b2a_hex(self.ser.read(1))
				if h1 == "aa":
					h2 = binascii.b2a_hex(self.ser.read(1))
					if h2 == "55":
						redata = binascii.b2a_hex(self.ser.read(self.rStreamLen-2))
						tail = binascii.b2a_hex(self.ser.read(2))
						if  tail ==  "cc33":
							if self.isRecei == False:
								self.isRecei = True
								self.recStream = h1+h2+redata
								self.ReceiEvent.set()
				self.isReaded = True
			except (binascii.Error,binascii.Incomplete):
				rospy.loginfo("binascii covert error")
		self.ser.flush()
		self.isReaded = True

	def send(self,data):
		if not self.stopSend:
			self.ser.write(data)
			self.SendEvent.set()

	def stop(self):	
		self.thread_stop = True
		self.stopSend = True
		while not self.isReaded:
			continue

class Robot_Base():
	def __init__(self):
		self.st = SerialThread()
		self.sendCommand = {"HEADER":"AA55","LW":"0000","RW":"0000","VACUM":"00","LBRUSH":"00","RBRUSH":"00","MBRUSH":"00","BEEP":"00","SLEEP":"00","CHARGE":"00","LEDR":"00","LEDG":"64","GYRO":"03"}
		#ros node init 
		rospy.init_node('robot_base',anonymous=True)
		rospy.on_shutdown(self.shutdown)
		self.odom_tf_trans = tf.TransformBroadcaster()
		self.setMotor = rospy.get_param("set_motor",False)
		self.pub_duration = rospy.get_param("~pub_duration",50)
		self.robot_battery = rospy.get_param("~robot_battery",16.8)
		#set lidar motor pin 
		self.lidar_motor(True)
		#start serial thread ,continue to read serial port
		self.baseAwake()
		self.st.start()
		self.odom = Odometry()
		self.robot_sensor = sensor()
		#subscribe "/cmd_vel" topic ,witch published by "move_base"
		rospy.Subscriber("/cmd_vel", Twist, self.wheels_ctrl)
		#subscribe "/periph_ctrl" topic ,
		rospy.Subscriber("/periph_ctrl",peripheral,self.periph_ctrl)
		self.sensor_pub = rospy.Publisher('/robot_sensor',sensor,queue_size = 1)
		self.odom_pub = rospy.Publisher('/odom',Odometry,queue_size=1)
		rospy.loginfo("robot base init done!")	

	def shutdown(self):
		self.st.stop()
		self.baseSleep()
		self.st.ser.close()
		print "%s serial close!"%self.st.ser.port
		self.lidar_motor(False)
		rospy.sleep(0.5)
		
	def baseSleep(self):
		self.sendCommand["SLEEP"]="01"
		self.sendCommand["GYRO"] ="00"
		self.serial_write()
		print "robot is going to sleep"

	def baseAwake(self):
		self.sendCommand["SLEEP"]="00"
		self.sendCommand["GYRO"] ="02"
		self.serial_write()
		print "wait 2 sencond or more ,before robot awake!"
		rospy.sleep(2)

	def lidar_motor(self,ctrl):
		"this function just for rplidar using only"
		if self.setMotor == True:
			if ctrl == True or ctrl == "True":
				os.system('echo 6 > /sys/class/gpio/export')
				os.system('echo "out" > /sys/class/gpio/gpio6/direction')
				os.system('echo 1 > /sys/class/gpio/gpio6/value')
			elif ctrl == False or ctrl == "False":
				os.system('echo 0 > /sys/class/gpio/gpio6/value')
				os.system('echo 6 > /sys/class/gpio/unexport')

	def serial_write(self):
		self.st.SendEvent.wait()
		self.st.SendEvent.clear()
		str_combine = self.sendCommand["HEADER"]+self.sendCommand["LW"]+self.sendCommand["RW"]+self.sendCommand["VACUM"]+self.sendCommand["LBRUSH"]+self.sendCommand["RBRUSH"]+self.sendCommand["MBRUSH"]+self.sendCommand["BEEP"]+self.sendCommand["SLEEP"]+self.sendCommand["CHARGE"]+self.sendCommand["LEDR"]+self.sendCommand["LEDG"]+self.sendCommand["GYRO"]
		crc_result = calBufCRC8(str_combine, len(str_combine)/2)
		str_combine = str_combine+crc_result+"cc33"
		tranStream = str_combine.decode("hex")
		self.st.send(tranStream)

	def periph_ctrl(periph_msg):
		self.sendCommand["VACUM"] = periph_msg.vaccum
		self.sendCommand["LBRUSH"] = periph_msg.lbrush
		self.sendCommand["RBRUSH"] = periph_msg.rbrush
		self.sendCommand["MBRUSH"] = periph_msg.mbrush
		self.sendCommand["SLEEP"] = periph_msg.sleep
		self.sendCommand["CHARGE"] = periph_msg.charge
		self.sendCommand["BEEP"] = periph_msg.beep
		self.sendCommand["LEDR"] = periph_msg.ledr
		self.sendCommand["LEDG"] = periph_msg.ledg
		self.sendCommand["GYRO"] = periph_msg.gyro
		self.serial_write()

	def wheels_ctrl(self,msg):  
		dist_bt_wheel=0.202 # robot wheel's distant
		v_l =((2*msg.linear.x)-(msg.angular.z*dist_bt_wheel))/2
		v_r =((2*msg.linear.x)+(msg.angular.z*dist_bt_wheel))/2
		if v_l<0.0:
			l_high = '%02x' % int(-v_l * 1000 / 256 + 128.0)
			l_low = '%02x' % int(-v_l * 1000.0 % 256.0)
		else:
			l_high = '%02x' % int(v_l * 1000.0 / 256.0)
			l_low = '%02x' % int(v_l * 1000.0 % 256.0)
		if v_r<0.0:
			r_high = '%02x' % int(-v_r * 1000 / 256 + 128.0)
			r_low = '%02x' % int(-v_r * 1000.0 % 256.0)
		else:
			r_high = '%02x' % int(v_r * 1000.0 / 256.0)
			r_low = '%02x' % int(v_r * 1000.0 % 256.0)
		self.sendCommand["LW"]=l_high+l_low
		self.sendCommand["RW"]=r_high+r_low
		self.serial_write()

	def robot_msg_pub(self):
		#set publish duration 50ms=20hz
		rate = rospy.Rate(self.pub_duration)
		recdata = ""
		lt = rospy.Time.now()
		pos_x = 0.0;pos_y = 0.0;pos_z=0.0;
		angle_z=0.0;
		last_batv = self.robot_battery
		th_l = 0.0
		while not rospy.is_shutdown():
			self.st.ReceiEvent.wait()
			self.st.ReceiEvent.clear()
			self.st.isRecei = False
			recdata=self.st.recStream
			r_len = len(recdata)
			r_crc = recdata[(r_len-2):r_len]
			c_crc = calBufCRC8(recdata[0:(r_len-2)],self.st.rStreamLen-1)
			if r_crc == c_crc:
				lw_vel    = int(recdata[ 4: 8],16)#2 bytes, extract left wheel value
				rw_vel    = int(recdata[ 8:12],16)#2 bytes, extract right wheel value
				angle     = int(recdata[12:16],16)#2 bytes, angle value
				angle_vel = int(recdata[16:20],16)#2 bytes, angle velocity value
				lw_crt    = int(recdata[20:24],16)#2 bytes, left wheel current value
				rw_crt    = int(recdata[24:28],16)#2 bytes, right wheel current value
				wall      = int(recdata[28:32],16)#2 bytes, wall value
				l_obs     = int(recdata[32:36],16)#2 bytes, left obs
				f_obs     = int(recdata[36:40],16)#2 bytes, front obs
				r_obs     = int(recdata[40:44],16)#2 bytes, right obs
				bumper    = int(recdata[44:46],16)#1 byte, bumper value
				ir_ctrl   = int(recdata[46:48],16)#1 byte, infrated controler
				c_stub    = int(recdata[48:54],16)#3 bytes,charge stub signal 
				key       = int(recdata[54:56],16)#1 byte, key value
				charge    = int(recdata[56:58],16)#1 byte, charge status
				w_tank    = int(recdata[58:60],16)#1 byte, water tank status
				bat       = int(recdata[60:62],16)#1 byte, battery status
				l_cliff   = int(recdata[62:66],16)#2 bytes, left cliff value
				f_cliff   = int(recdata[66:70],16)#2 bytes, front cliff value
				r_cliff   = int(recdata[70:74],16)#2 bytes, right cliff value
				over_crt  = int(recdata[74:76],16)#1 bytes, brush and vacuum current value	
				rbumper   = 0# right bumper
				lbumper   = 0# left bumper
				lbrush_oc = 0#left brush over current
				rbrush_oc = 0#right brush over current
				mbrush_oc = 0#main brush over current 
				vcum_oc   = 0#vccum over current
				if (bumper & 0xf0) != 0:
					lbumper = 1
				if (bumper & 0x0f) != 0:
					rbumper = 1
				if (0x08 & over_crt) ==1:
					lbrush_oc =1
				if (0x04 & over_crt) ==1:
					rbrush_oc =1
				if (0x02 & over_crt) ==1:
					mbrush_oc = 1
				if (0x01 & over_crt) ==1:
					vcum_oc = 1 
				if lw_vel > 0x7fff:
					lw_vel = -(float(lw_vel - 0x8000) / 1000.0)
				else:
					lw_vel = float(lw_vel) / 1000.0
				if rw_vel > 0x7fff:
					rw_vel = -(float(rw_vel - 0x8000) / 1000.0)
				else:
					rw_vel = float(rw_vel) / 1000.0
				if angle > 0x7fff:
					angle = angle-0xffff
				angle = -1*angle/100.0
				if angle_vel > 0x7fff:
					angle_vel = angle_vel - 0xffff
				angle_vel= -1* angle_vel/100.0
				curr_batv = round(bat/10.0,2)
				if curr_batv < 13.0:
					self.sendCommand["LEDR"] = "64"
					self.sendCommand["LEDG"] = "00"
				else:
					self.sendCommand["LEDR"] = "00"
					self.sendCommand["LEDG"] = "64"
				if l_obs > 0x7fff:
					l_obs = -(0xffff - l_obs)
				if r_obs > 0x7fff:
					r_obs = -(0xffff - r_obs)
				if f_obs > 0x7fff:
					f_obs = -(0xffff - f_obs)
				if wall > 0x7fff:
					wall = -(0xffff-wall)
				if l_cliff > 0x7fff:
					l_cliff = -(0xffff-wall)
				if r_cliff > 0x7fff:
					r_cliff  = -(0xffff-r_cliff)
				if f_cliff > 0x7fff:
					f_cliff = -(0xffff-f_cliff)
				self.robot_sensor.lw_vel   =  lw_vel
				self.robot_sensor.rw_vel   =  rw_vel
				self.robot_sensor.angle    =  angle
				self.robot_sensor.angle_v  =  angle_vel
				self.robot_sensor.lw_crt   =  lw_crt
				self.robot_sensor.rw_crt   =  rw_crt
				self.robot_sensor.wall     =  wall
				self.robot_sensor.l_obs    =  l_obs
				self.robot_sensor.f_obs    =  f_obs
				self.robot_sensor.r_obs    =  r_obs
				self.robot_sensor.lbumper  =  lbumper
				self.robot_sensor.rbumper  =  rbumper
				self.robot_sensor.ir_ctrl  =  ir_ctrl
				self.robot_sensor.c_stub   =  c_stub
				self.robot_sensor.key      =  key
				self.robot_sensor.c_s      =  charge
				self.robot_sensor.w_tank   =  w_tank
				self.robot_sensor.batv     =  curr_batv
				self.robot_sensor.rcliff   =  r_cliff
				self.robot_sensor.lcliff   =  l_cliff
				self.robot_sensor.fcliff   =  f_cliff
				self.robot_sensor.lbrush_oc=  lbrush_oc
				self.robot_sensor.rbrush_oc=  rbrush_oc
				self.robot_sensor.mbrush_oc=  mbrush_oc
				self.robot_sensor.vcum_oc  =  vcum_oc
				self.sensor_pub.publish(self.robot_sensor)#publish robot sensor value	
				if abs(last_batv- curr_batv)> 0.001:
					last_batv = curr_batv
					rospy.loginfo("battery voltage %2.1f v"%curr_batv)
				ct = rospy.Time.now()
				dt = ct - lt
				dt = dt.to_sec()
				vx = (lw_vel+rw_vel)/2
				#vth = (rw_vel - lw_vel)/0.2 #
				#th = vth*dt	
				#vth =angle_vel*pi/180.0
				th = angle*pi/180.0
				vth = (th-th_l)/dt
				pos_x += round((vx *math.cos(th)-0*math.sin(th))*dt,3)
				pos_y += round((vx *math.sin(th)+0*math.cos(th))*dt,3)
				pos_z += 0.000
				odom_quat_tran = quaternion_from_euler(0,0,th)
				odom_quat= quaternion_from_euler(0,0,th,axes="sxyz")
				self.odom.header.stamp = ct
				self.odom.header.frame_id = "odom"
				self.odom.child_frame_id = "base_link"
				self.odom.pose.pose = Pose(Point(pos_x,pos_y,pos_z),Quaternion(*odom_quat_tran))
				self.odom.twist.twist.linear.x = vx
				self.odom.twist.twist.linear.y = 0
				self.odom.twist.twist.linear.z = 0
				self.odom.twist.twist.angular.x = 0
				self.odom.twist.twist.angular.y = 0
				self.odom.twist.twist.angular.z = vth
				self.odom_tf_trans.sendTransform((pos_x,pos_y,pos_z),odom_quat_tran,ct,"base_link","odom")
				self.odom_pub.publish(self.odom)
				lt = ct
				th_l = th
			#rate.sleep()
