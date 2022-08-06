#!/usr/bin/env python
import rospy
import math

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

banner = """
Reading from the keyboard  and Publishing to AckermannDriveStamped!
---------------------------
Moving around:
        w
   a    s    d
anything else : stop
CTRL-C to quit
"""

#Lateral force Fy(slip angle alpha)
#Fy = D*sin(C(arctg(B*angle - E*(B*angle - arctg(B*angle)))))

#Longitudinal force Fx(longitudinal slip k)
#Fx = D*sin(C(arctg(B*l_slip - E*(B*l_slip - arctg(B*l_slip)))))

#acceleration_x = -force / massa (x)
#acceleration_y = force / massa (y)
 
keyBindings = {
  'w':(1,0),
  'd':(1,-1),
  'a':(1,1),
  's':(-1,0),
}

#MF coefficients
B = 10000000.0 #Stiffness factor kp
Cy = 1.30 #Lateral force shape factor
Cx = 1.65 #Longitudinal force shape factor
D = 1 #Peak factor mu*Fz
Ey = (B - math.tan(math.pi/(2*Cy)))/(B-math.atan(B)) #Lateral curvature factor
Ex = (B - math.tan(math.pi/(2*Cx)))/(B-math.atan(B)) #Longitudinal curvature factor

#Model values
mass = 5.6922
wheel_radius = 0.05

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

speed = 0.5
turn = 0.5

def longitudinalSlip(th):
   w = speed*math.sin(th*turn)/wheel_radius
   if(speed > wheel_radius*w):
      k = (speed - wheel_radius*w)/speed
   else:
      k = (wheel_radius*w - speed)/(wheel_radius*w)
   return k

def lateralSlip(x, th):
   t1 = speed*th - (math.pi/180)*0.35
   t2 = speed*x - (math.pi/180)*0.2
   slip = math.atan(t1/t2)
   return slip

def acceleration_X(k):
   Fx = D*math.sin(Cx*(math.atan(B*k - Ex*(B*k - math.atan(B*k)))))
   accx = -Fx/mass
   return accx

def acceleration_Y(slip):
   Fy = D*math.sin(Cy*(math.atan(B*slip - Ey*(B*slip - math.atan(B*slip)))))
   accy = Fy/mass
   return accy   

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped)
  rospy.init_node('keyop')

  x = 0
  th = 0
  status = 0

  try:
    while(1):
       key = getKey()
       if key in keyBindings.keys():
          x = keyBindings[key][0]
          th = keyBindings[key][1]
       else:
          x = 0
          th = 0
          if (key == '\x03'):
             break

       #Longitudinal slip k
       k = longitudinalSlip(th)

       #Slip angle
       slip = lateralSlip(x, th)

       #Composition of msg to publish 
       msg = AckermannDriveStamped();
       msg.header.stamp = rospy.Time.now();
       msg.header.frame_id = "base_link";
       msg.drive.speed = x*speed;
       #Lateral and longitudinal forces
       msg.drive.acceleration = acceleration_X(k)*x + acceleration_Y(slip)*th;
       msg.drive.jerk = 1;
       msg.drive.steering_angle = th*turn
       msg.drive.steering_angle_velocity = x*speed

       pub.publish(msg)

  except:
    print 'error'

  finally:
    #Longitudinal Slip
    k = longitudinalSlip(th)

    #Slip angle
    slip = lateralSlip(x, th)

    #Composition of msg to publish
    msg = AckermannDriveStamped();
    msg.header.stamp = rospy.Time.now();
    msg.header.frame_id = "base_link";
    msg.drive.speed = 0;
    #Lateral and longitudinal forces
    msg.drive.acceleration = acceleration_X(k)*x + acceleration_Y(slip)*th;
    msg.drive.jerk = 1;
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 1
    pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
