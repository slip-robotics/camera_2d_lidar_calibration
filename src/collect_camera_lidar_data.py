#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import cv2
import yaml

u = None
v = None

x = None
y = None

alpha = 1.0
beta = 0

clicked = False
fig = None

sub_img = np.array([])

class matplot:

    def __init__(self):
        global alpha, beta
        alpha = 1.0
        beta = 0
        self.create_figure()
            
    def create_figure(self):
        global fig
        if sub_img.size == 0:
            print('No image obtained.')
            return
        self.img = sub_img
        self.img_ori = sub_img
        self.fig = plt.figure()
        self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        ax = self.fig.add_subplot(111)
        self.image = ax.imshow(self.img)
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.fig.canvas.mpl_connect('key_press_event', self.onkey)
        fig = self.fig
        plt.show()
        
    def onclick(self, event):
        global u, v
        if event.xdata != None and event.ydata != None and event.button == 3:
            u = int(event.xdata)
            v = int(event.ydata)
            img_temp = np.copy(self.img)
            cv2.circle(img_temp, (u, v), 1, (255,0,0), 1)
            self.image.set_data(img_temp)
            plt.draw()

    def onkey(self, event):
        global u, v, clicked, alpha, beta
        if event.key == 'enter':
            if u == None and v == None:
                print('No point selected. Try again.')
            else:
                print('Image coordinate: (%d, %d), Laser coordinate: (%f, %f)' % (u, v, x, y))
                plt.close()
                with open(output_file, 'a') as f:
                    f.write('%f %f %d %d\n' % (x, y, u, v))
                u, v = None, None
        elif event.key in ['up','down','pageup','pagedown']:
            if event.key == 'up':
                alpha += 0.1
            if event.key == 'down':
                alpha -= 0.1
            if event.key == 'pageup':
                beta += 10
            if event.key == 'pagedown':
                beta -= 10
            self.img = cv2.addWeighted(self.img_ori, alpha, self.img_ori, 0, beta)
            self.image.set_data(self.img)
            self.fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
            plt.draw()

def point_cb(msg):
    global x, y, clicked, fig
    x = msg.pose.position.x
    y = msg.pose.position.y
    if clicked and plt.get_fignums():
        fig.suptitle('Laser coordinate: (%f, %f)\nAlpha: %f   Beta: %d' % (x, y, alpha, beta))
        plt.draw()
    else:
        pass
    clicked = True

def image_info_cb(msg):
    global K, D, camera_info_recieved
    K = np.matrix([[msg.K[0], msg.K[1], msg.K[2]],
                    [msg.K[3], msg.K[4], msg.K[5]],
                    [msg.K[6], msg.K[7], msg.K[8]]])
    D = np.array([msg.D[0], msg.D[1], msg.D[2], msg.D[3]])
    camera_info_recieved = True
    print('Camera info recieved.')
    print('K=',K)
    print('D=',D)

def image_cb(msg):
    if camera_info_recieved == False:
        print('Camera info not recieved.')
        return
    global sub_img, K, D, lens
    sub_img_distorted = bridge.imgmsg_to_cv2(msg)
    sub_img_distorted = cv2.cvtColor(sub_img_distorted, cv2.COLOR_BGR2RGB)
    if lens == 'pinhole':
        sub_img = cv2.undistort(sub_img_distorted, K, D, newCameraMatrix = K)
    elif lens == 'fisheye':
        sub_img = cv2.fisheye.undistortImage(sub_img_distorted, K, D, Knew = K)

def Shutdown():
    plt.close()

rospy.on_shutdown(Shutdown)

output_file = rospy.get_param('~output_file')   

point_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, point_cb)
image_sub = rospy.Subscriber('/image', Image, image_cb)
image_info_sub = rospy.Subscriber('/camera_info', CameraInfo, image_info_cb)
bridge = CvBridge()

# with open(config_file, 'r') as f:
#     f.readline()
#     config = yaml.load(f)
#     lens = config['lens']
#     fx = float(config['fx'])
#     fy = float(config['fy'])
#     cx = float(config['cx'])
#     cy = float(config['cy'])
#     k1 = float(config['k1'])
#     k2 = float(config['k2'])
#     p1 = float(config['p1/k3'])
#     p2 = float(config['p2/k4'])
# if lens not in ['pinhole', 'fisheye']:
#     print('Invalid lens, using pinhole as default.')
#     lens = 'pinhole'

lens = 'pinhole'
K = np.matrix([[0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]])
D = np.array([0.0, 0.0, 0.0, 0.0])

camera_info_recieved = False

# print("Camera parameters")
# print("Lens = %s" % lens)
# print("K =")
# print(K)
# print("D =")
# print(D)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    if clicked:
        matplot()
        clicked = False
    rate.sleep()
