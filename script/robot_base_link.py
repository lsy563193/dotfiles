#!/usr/bin/env python  
# -*- coding: utf-8 -*-
import roslib
import sys
roslib.load_manifest('pp')  
import rospy  
import tf.transformations 
import geometry_msgs.msg
from pp.msg import sensor,periph
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose,Point,Quaternion,Twist
import time
import binascii
import os
import math
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
		self.rStreamLen=41
		self.isSend = False
		self.isRecei = False
		self.thread_stop = False
		self.ser = serial.Serial()
		self.ser.port='/dev/ttyS0'
		self.ser.baudrate=115200
		self.ser.timeout=None
		self.ser.open()
		self.isReaded = False
		self.ReceiEvent = threading.Event()
		if not self.ser.readable():
			print 'open serial port fail !'
			sys.exit()
		else:
			print 'open serial port success !'
			#print self.ser

	def run(self):
		while True:
			if self.thread_stop == False:
				try:
					self.isReaded =False
					redata = binascii.b2a_hex(self.ser.read(1)) #cover into hexadecimal representation ,therefore the length of data become twice as long as the original data
					if int(redata[0], 16) == 0xa and int(redata[1], 16) == 0xa:
						redata = binascii.b2a_hex(self.ser.read(1)) #cover into hexadecimal representation ,therefore the length of data become twice as long as the original data
						if int(redata[0], 16) == 0x5 and int(redata[1], 16) == 0x5:
							redata = 'aa55' + binascii.b2a_hex(self.ser.read(self.rStreamLen - 2)) #cover into hexadecimal representation ,therefore the length of data become twice as long as the original data
							if int(redata[78], 16) == 0x5 and int(redata[79], 16) == 0x5:
								if int(redata[80], 16) == 0xa and int(redata[81], 16) == 0xa:
									self.isReaded = True
						else:
							continue
					else:
							continue
					#self.ser.flush()

				except (binascii.Error,binascii.Incomplete):
					rospy.loginfo("binascii covert error")

				if len(redata) == ((self.rStreamLen) * 2):
					if self.isNotNull(redata):
						if self.isRecei==False:
							self.recStream=redata
							self.isRecei=True
							self.ReceiEvent.set()
					else:
						print "receive empty data"
				else:
					print "read data length incorrect!"
			else:
				self.isReaded = True
				break;

	def isNotNull(self,reData):
		reval = True
		for i in range(len(reData)):
			if reData[i] == " ":
				reval = False
				break 
		return reval

	def stop(self):	
		self.thread_stop = True
		self.ReceiEvent.clear()
		while not self.isReaded:
			continue

class Robot_Base():
	sendCommand = {"CheckHead":"AA55","LW":"0000","RW":"0000","VACUM":"00","LB":"00","RB":"00","MB":"00","AUDIO":"00","SLEEP":"00","CHARGE":"00","LED_R":"00","LED_G":"00","GYRO":"00"}
	def __init__(self):
		self.st = SerialThread()
		#ros node init 
		rospy.init_node('robot_base',anonymous=True)
		rospy.on_shutdown(self.shutdown)
		self.odom_tf_trans = tf.TransformBroadcaster()
		self.setMotor = rospy.get_param("set_motor",False)
		self.pub_duration = rospy.get_param("~pub_duration",50)
		self.robot_battery = rospy.get_param("~robot_battery",16.8)
		#start serial thread ,continue to read serial port
		self.baseAwake()
		self.st.start()
		while not self.st.isRecei:
			continue
		self.odom = Odometry()
		self.robot_sensor = sensor()
		#subscribe "/periph_ctrl" topic ,
		self.sensor_pub = rospy.Publisher('/robot_sensor',sensor,queue_size = 1)
		self.odom_pub = rospy.Publisher('/odom',Odometry,queue_size=1)
		rospy.loginfo("set robot_sensor,odom,odom_transform publish rate to :%3.2f Hz"%self.pub_duration)	

	def shutdown(self):
		self.st.stop()
		self.baseSleep()
		rospy.sleep(1)
		self.st.ser.close()
		print "%s serial close!"%self.st.ser.port
		self.st.join(1)

	def baseSleep(self):
		self.sendCommand["GYRO"]="00"
		self.serial_write()
		rospy.sleep(0.5)
		self.sendCommand["SLEEP"]="01"
		self.serial_write()
		print "robot is going to sleep"

	def baseAwake(self):
		self.baseSleep()
		rospy.sleep(0.5)
		self.sendCommand["SLEEP"]="00"
		self.serial_write()
		rospy.sleep(0.5)
		self.sendCommand["GYRO"]="02"
		self.serial_write()
		print "wait 2 sencond or more ,before robot awake!"
		rospy.sleep(2)

	def serial_write(self):
		str_combine = self.sendCommand["CheckHead"]+self.sendCommand["LW"]+self.sendCommand["RW"]+self.sendCommand["VACUM"]+self.sendCommand["LB"]+self.sendCommand["RB"]+self.sendCommand["MB"]+self.sendCommand["AUDIO"]+self.sendCommand["SLEEP"]+self.sendCommand["CHARGE"]+self.sendCommand["LED_R"]+self.sendCommand["LED_G"]+self.sendCommand["GYRO"]
		crc_result = calBufCRC8(str_combine, len(str_combine)/2)
		str_combine = str_combine+crc_result+"55aa"
		tranStream = str_combine.decode("hex")
		self.st.ser.write(tranStream)

	def robot_msg_pub(self):
		#set publish duration 50ms=20hz
		rate = rospy.Rate(self.pub_duration)
		recdata = ""
		lt = rospy.Time.now()
		pos_x = 0.0;pos_y = 0.0;pos_z=0.0;
		angle_z=0.0;
		last_batv = self.robot_battery
		delth = 0.0
		ab_len=2 #ascii byte length	
		while not rospy.is_shutdown():
			self.st.ReceiEvent.wait()
			self.st.ReceiEvent.clear()
			if self.st.isRecei:
				recdata=self.st.recStream#recStream is ascii hex format 0xaa --> 'aa'
				#print recdata
				r_len = len(recdata)
				self.st.isRecei = False
				if r_len == 0 or r_len != (self.st.rStreamLen) * ab_len:
					print "receive data incorrect"
					continue

				r_crc=recdata[(r_len-ab_len - 4 ):r_len - 4 ]
				c_crc = calBufCRC8(recdata[0 : (r_len - (3 * ab_len))], self.st.rStreamLen - 3)
				if r_crc == c_crc:
					lw_vel = recdata[2*ab_len:4*ab_len] # extract left wheel value
					rw_vel = recdata[4*ab_len:6*ab_len] # extract right wheel value
					angle = recdata[6*ab_len:8*ab_len] #extract angle value
					angle_vel = recdata[8*ab_len:10*ab_len] #extract angle velocity
					lw_cur = recdata[10*ab_len:12*ab_len] #extract angle velocity
					rw_cur = recdata[12*ab_len:14*ab_len] #extract angle velocity
					wall = recdata[14*ab_len:16*ab_len] #extract angle velocity
					l_obs = recdata[16*ab_len:18*ab_len] #extract angle velocity
					f_obs = recdata[18*ab_len:20*ab_len] #extract angle velocity
					r_obs = recdata[20*ab_len:22*ab_len] #extract angle velocity
					bumper = recdata[22*ab_len:23*ab_len] #bumper 
					remote = recdata[23*ab_len:24*ab_len] #remote
					home_wall = recdata[24*ab_len:27*ab_len] #remote
					key = recdata[27*ab_len:28*ab_len] #remote
					charge = recdata[28*ab_len:29*ab_len] #remote
					water_tank = recdata[29*ab_len:30*ab_len] #remote
					bat_v = recdata[30*ab_len:31*ab_len] #remote
					Lcliff = recdata[31*ab_len:33*ab_len] #remote
					Fcliff = recdata[33*ab_len:35*ab_len] #remote
					Rcliff = recdata[35*ab_len:37*ab_len] #remote
					current = recdata[37*ab_len:38*ab_len] #remote

					lw_vel = int(lw_vel, 16)
					rw_vel = int(rw_vel,16)
					angle = int(angle,16)
					angle_vel = int(angle_vel,16)
					bumper = int(bumper,16)
					rbumper = 0
					lbumper = 0
					if (bumper & 0xf0) != 0:
						lbumper = 1
					if (bumper & 0x0f) != 0:
						rbumper = 1

					if lw_vel > 0x7fff:
						lw_vel = -(float(lw_vel - 0x8000) / 1000.0)
					else:
						lw_vel = float(lw_vel) / 1000.0

					if rw_vel > 0x7fff:
						rw_vel = -(float(rw_vel - 0x8000) / 1000.0)
					else:
						rw_vel = float(rw_vel) / 1000.0

					if angle > 0x7fff:
						angle = angle - 0xffff
					angle = -1 * angle / 100.0

					if angle_vel > 0x7fff:
						angle_vel = angle_vel - 0xffff
					angle_vel= -1* angle_vel/100.0

					wall = int(wall, 16)
					if wall > 0x7fff:
						wall = -(0xffff - wall)

					l_obs = int(l_obs, 16)
					if l_obs > 0x7fff:
						l_obs = -(0xffff - l_obs)

					f_obs = int(f_obs, 16)
					if f_obs > 0x7fff:
						f_obs = -(0xffff - f_obs)

					r_obs = int(r_obs, 16)
					if r_obs > 0x7fff:
						r_obs = -(0xffff - r_obs)

					self.robot_sensor.angle = angle
					self.robot_sensor.angle_v = angle_vel
					self.robot_sensor.lw_cur = int(lw_cur, 16)
					self.robot_sensor.rw_cur = int(lw_cur, 16)
					self.robot_sensor.wall = wall
					self.robot_sensor.lobs = l_obs
					self.robot_sensor.fobs = f_obs
					self.robot_sensor.robs = r_obs
					self.robot_sensor.rbumper = int(rbumper)
					self.robot_sensor.lbumper = int(lbumper)
					self.robot_sensor.remote = int(remote, 16)
					self.robot_sensor.home_wall = int(home_wall, 16)
					self.robot_sensor.key = int(key, 16)
					self.robot_sensor.crg = int(charge,16)
					self.robot_sensor.water_tank = int(water_tank,16)
					self.robot_sensor.batv = int(bat_v,16)
					self.robot_sensor.rcliff = int(Rcliff,16)
					self.robot_sensor.lcliff = int(Lcliff,16)
					self.robot_sensor.fcliff = int(Fcliff,16)
					self.robot_sensor.current = int(current,16)

					self.sensor_pub.publish(self.robot_sensor)

					curr_batv = self.robot_sensor.batv/10.0
					if last_batv > curr_batv:
						last_batv = curr_batv
						print "battery voltage %2.1f v"%curr_batv

					vx = (lw_vel+rw_vel)/2
					vth = angle_vel * math.pi / 180 #trun into radian
					th = angle * math.pi / 180 #trun into radian
					ct = rospy.Time.now()
					dt = ct - lt
					dt = dt.to_sec()
					#vth = round(th-delth/dt,3)

					pos_x += round((vx * math.cos(th) - 0 * math.sin(th)) * dt,3)
					pos_y += round((vx * math.sin(th) + 0 * math.cos(th)) * dt,3)
					pos_z += 0.000
					odom_quat_tran = quaternion_from_euler(0, 0, th)
					odom_quat= quaternion_from_euler(0, 0, th, axes = "sxyz")
					self.odom.header.stamp = ct
					self.odom.header.frame_id = "odom"
					self.odom.child_frame_id = "base_link"
					self.odom.pose.pose = Pose(Point(pos_x, pos_y, pos_z), Quaternion(*odom_quat_tran))
					'''
					self.odom.pose.pose.position.x = pos_x
					self.odom.pose.pose.position.y = pos_y
					self.odom.pose.pose.position.z = pos_z
					self.odom.pose.pose.orientation.x = odom_quat[0]
					self.odom.pose.pose.orientation.y = odom_quat[1]
					self.odom.pose.pose.orientation.z = odom_quat[2]
					self.odom.pose.pose.orientation.w = odom_quat[3]
					#self.odom.pose.pose.orientation = Quaternion(*odom_quat)					'''
					self.odom.twist.twist.linear.x = vx
					self.odom.twist.twist.linear.y = 0
					self.odom.twist.twist.linear.z = 0
					self.odom.twist.twist.angular.x = 0
					self.odom.twist.twist.angular.y = 0
					self.odom.twist.twist.angular.z = vth
					self.odom_tf_trans.sendTransform((pos_x,pos_y,pos_z),odom_quat_tran,ct,"base_link","odom")
					self.odom_pub.publish(self.odom)
					lt = ct
					delth = th
				else:
					print ("crc error: ",  r_crc, c_crc, recdata)
			#rate.sleep()
