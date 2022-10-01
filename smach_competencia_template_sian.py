#!/usr/bin/env python3
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist , PoseStamped
import smach
import ros_numpy
from utils_evasion import *
import tf2_ros

########## Functions for takeshi states ##########

def get_coords ():
    for i in range(15):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans=0

def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)
    
def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 

def move_forward():
    move_base(0.10,0,0,2.5)
def move_backward():
    move_base(-0.10,0,0,2.5)
def turn_left():
    move_base(0.0,0,np.pi/9,9)
def turn_right():
    move_base(0.0,0,-np.pi/9,9)
def move_started():
	move_base(0.0,-0.1,0,15)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura)

        right_scan=lectura[:150]
        left_scan=lectura[150:]
        ront_scan=lectura[359:361]

        sd,si=0,0
        if np.mean(left_scan)< 0.5: si=1
        if np.mean(right_scan)< 0.5: sd=1
        #if np.mean(ront_scan)< 0.5: sf=1

    except:
        sd,si=0,0    

    return si,sd
##### Define state INITIAL #####

class Inicio (smach.State):
	def __init__(self):
	        smach.State.__init__(self, outcomes=['succ','fail']) #shor for success
	    
	def execute(self,userdata):
		global meta_leida,punto_inicial
		print('inicializando')
		rospy.sleep(1)
		punto_inicial = get_coords()
		
		meta_leida = rospy.wait_for_message('/meta_competencia', PoseStamped, timeout = 10)
		
		print('tiempo = '+ str(punto_inicial.header.stamp.to_sec()) ,punto_inicial.transform)
		print('meta leida', meta_leida)
		print('arrancando')
		return 'succ'

class S1(smach.State):
	def __init__(self):
	        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
	        self.counter = 0
        
	def execute(self,userdata):
# Aqui va lo que se desea ejecutar en el estado

	        print('robot Estado S_1')
	        #####Accion
	        global nm_x,nm_y
	        move_started()
	        mx=meta_leida.pose.position.x
	        my=meta_leida.pose.position.y+1.5
	        nm_x=math.sqrt(mx*mx+my*my)
	        nm_y=0
	        print('nueva meta(x,y):(',nm_x,',',nm_y,')')
	        if mx < 0 and my < 0:
	        	angle= np.pi - math.atan(my/mx)
	        	print('angulo:', angle)
	        	t1= angle/0.1
	        	print('tiempo de giro', t1)
	        	move_base(0,0,-0.1,t1)
	        if mx > 0 and my > 0:
	        	angle= math.atan(my/mx)
        		print('angulo:', angle)
        		t1= angle/0.1
        		print('tiempo de giro', t1)
        		move_base(0,0,0.1,t1)
        	if mx < 0 and my > 0:
        		angle= np.pi - math.atan(my/mx)
        		print('angulo:', angle)
        		t1= angle/0.1
        		print('tiempo de giro', t1)
        		move_base(0,0,0.1,t1)
        	if mx > 0 and my < 0:
        		angle= math.atan(my/mx)
        		print('angulo:', angle)
        		t1= ABS(angle)/0.1
        		print('tiempo de giro', t1)
        		move_base(0,0,-0.1,t1)
        	return 'outcome1'
        	
class SO(smach.State):
	def __init__(self):
	        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
	        self.counter = 0
	def execute(self,userdata):
		global nm_x, nm_y
		if nm_x < 0 and nm_y < 0:
			angle= np.pi - math.atan(nm_y/nm_x)
			print('angulo:', angle)
			t1= angle/0.1
			print('tiempo de giro', t1)
			move_base(0,0,-0.1,t1)
		if nm_x > 0 and nm_y > 0:
			angle= math.atan(nm_y/nm_x)
			print('angulo:', angle)
			t1= angle/0.1
			print('tiempo de giro', t1)
			move_base(0,0,0.1,t1)
		if nm_x < 0 and nm_y > 0:
			angle= np.pi - math.atan(nm_y/nm_x)
			print('angulo:', angle)
			t1= angle/0.1
			print('tiempo de giro', t1)
			move_base(0,0,0.1,t1)
		if nm_x > 0 and n_my < 0:
			angle= math.atan(nm_y/nm_x)
			print('angulo:', angle)
			t1= ABS(angle)/0.1
			print('tiempo de giro', t1)
			move_base(0,0,-0.1,t1)
		return 'outcome1'
		

class SYL(smach.State):
	def __init__(self):
	        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
	        self.counter = 0
	def execute(self,userdata):
		if math.sqrt(nm_x*nm_x+nm_y*nm_y) < 0.08:
			return 'outcome1'
		else:
	        	return 'outcome2'
       	

class SF(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
              
	def execute(self,userdata):
        	print('robot Estado S_2')
        	#####Accion
        	punto_final=get_coords()
        	print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform)
        	return 'outcome1'


class sopcv{

	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		
	def execute(self,userdata):
      		print('robot Estado S_opcv')
        	captura=cv2.VideoCapture(0)

		while True:
		  ret,frame = captura.read()
		  if ret == True:
		    frameHSV = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
		    im_gray = cv2.cvtColor(frameHSV,cv2.COLOR_BGR2GRAY)
		    im_gray = cv2.cvtColor(frameHSV,cv2.COLOR_BGR2GRAY)
		    im_gray = cv2.GaussianBlur(im_gray, (7, 7), 0)
		    im_gray= cv2.medianBlur(im_gray, 3)
		    _,th =  cv2.threshold(im_gray, 155, 200, cv2.THRESH_BINARY_INV)
		    cnts,hierarchy = cv2.findContours(th, 1, 2)
		    
		    for c in cnts:
		      area = cv2.contourArea(c)
		      if 120000>area> 3000:
			M = cv2.moments(c)
			if (M["m00"]==0): M["m00"]=1
			x = int(M["m10"]/M["m00"])
			y = int(M['m01']/M['m00'])
			cv2.circle(frame, (x,y), 7, (0,0,255), -1)
			font = cv2.FONT_HERSHEY_SIMPLEX
			cv2.putText(frame, '{},{}'.format(x,y),(x+10,y), font, 0.75,(0,255,0),1,cv2.LINE_AA)
			nuevocont = cv2.convexHull(c)
			cv2.drawContours(frame, [nuevocont], 0, (255,0,0), 3)
		    cv2.imshow('frame',frame)
		    cv2.imshow('im_gray',im_gray)
		    cv2.imshow('th',th)
		    if cv2.waitKey(1) & 0xFF == ord('s'):
		      break
		  else: break
		captura.release()
		cv2.destroyAllWindows()				
        	
   return 'outcome1'


}

class S4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4'])
		self.counter = 0     

	def execute(self,userdata):
	        global nm_x, nm_y
	        si,sd=get_lectura_cuant()
	        if (si==0 and sd==0): 
	        	move_forward()
	        	nm_x=nm_x-0.25
	        	
	        	return 'outcome1'
            
	        if (si==0 and sd==1): return 'outcome2'
	        if (si==1 and sd==0): return 'outcome3'
	        if (si==1 and sd==1): return 'outcome4'

class S5(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.counter = 0
        
	def execute(self,userdata):
		global nm_x, nm_y
		turn_left()
		move_forward()
		nm_y=-nm_x
		nm_x=-0.25
		return 'outcome1'

class S6(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.counter = 0
        
	def execute(self,userdata):
		global nm_x, nm_y
		turn_right()
		move_forward()
		nm_y=nm_x
		nm_x=-0.25
		return 'outcome1'

class S7(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['outcome1','outcome2'])
		self.counter = 0
	    
	def execute(self,userdata):
		turn_right()
		move_forward()
		nm_y=nm_x
		nm_x=-0.25
		return 'outcome1'


def init(node_name):
	global laser, base_vel_pub
	rospy.init_node(node_name)
	base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
	laser = Laser()  

	#Entry point
if __name__== '__main__':

	print("STATE MACHINE...")
	init("smach_node")
	sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	with sm:
		#State machine for evasion
		smach.StateMachine.add("INICIO",   Inicio(),  transitions = {'fail':'END', 'succ':'s_1'})
		smach.StateMachine.add("s_o",  SO(), transitions = {'outcome1':'s_yl','outcome2':'END'})
		smach.StateMachine.add("s_yl",   SYL(),  transitions = {'outcome1':'s_f','outcome2':'s_4'}) 
		smach.StateMachine.add("s_opcv",  sopcv(),  transitions = {'outcome1':'s_4','outcome2':'END'}) 
		smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_yl','outcome2':'END'})
		smach.StateMachine.add("s_f",   SF(),  transitions = {'outcome1':'END','outcome2':'END'})
		smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_yl', 'outcome2':'s_5','outcome3':'s_6','outcome4':'s_7'})
		smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_o','outcome2':'END'})
		smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_o','outcome2':'END'})
		smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_o','outcome2':'END'})


	outcome = sm.execute()
