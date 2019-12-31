import requests
import numpy as np
import time
from scipy import stats
from scipy.linalg import block_diag
from itertools import count
import sys
import http
import pyvisgraph as vg

import scipy
from pyzbar import pyzbar
import imutils
import cv2

HOST      = '192.168.1.39'
PORT 	  = '8000'
autologin = 1
# BASE_URL is variant use to save the format of host and port
BASE_URL = 'http://' + HOST + ':'+ PORT + '/'
pose = 0
QR_memory = {0: [0.6,3],1: [1.5,3],2: [2.4,3],3: [3,2.4],4: [3,1.5],5: [3,0.6],6: [2.4,0],7: [1.5,0],8: [0.6,0],9: [0,0.6],10: [0,1.5],11: [0,2.4]}
def __request__(url, times=10):
	for x in range(times):
		try:
			requests.get(url)
			return 0
		except :
			print("Connection error, try again")
	print("Abort")
	return -1

def run_action(cmd):
	"""Ask server to do sth, use in running mode

	Post requests to server, server will do what client want to do according to the url.
	This function for running mode

	Args:
		# ============== Back wheels =============
		'bwready' | 'forward' | 'backward' | 'stop'

		# ============== Front wheels =============
		'fwready' | 'fwleft' | 'fwright' |  'fwstraight'

		# ================ Camera =================
		'camready' | 'camleft' | 'camright' | 'camup' | 'camdown'
	"""
	# set the url include action information
	url = BASE_URL + 'run/?action=' + cmd
	print('url: %s'% url)
	# post request with url
	__request__(url)

def run_speed(speed):
	"""Ask server to set speed, use in running mode

	Post requests to server, server will set speed according to the url.
	This function for running mode.

	Args:
		'0'~'100'
	"""
	# Set set-speed url
	url = BASE_URL + 'run/?speed=' + speed
	print('url: %s'% url)
	# Set speed
	__request__(url)

def run_fwturn(angle):

	url = BASE_URL + 'run/?action=fwturn:' + str(angle)
	print('url: %s'% url)
	__request__(url)

def connection_ok():
	"""Check whetcher connection is ok

	Post a request to server, if connection ok, server will return http response 'ok'

	Args:
		none

	Returns:
		if connection ok, return True
		if connection not ok, return False

	Raises:
		none
	"""
	cmd = 'connection_test'
	url = BASE_URL + cmd
	print('url: %s'% url)
	# if server find there is 'connection_test' in request url, server will response 'Ok'
	try:
		r=requests.get(url)
		if r.text == 'OK':
			return True
	except:
		return False

	
# Velocity -speed Calibration
##run_action('fwstraight')
##time.sleep(1)
##run_speed('25')
##run_action('forward')
##time.sleep(5)
##run_action('stop')

speeds_digital = np.array([25, 50, 75, 100])
#distance_moved = np.array([0.18, 0.9, 0.53, 1.13]) #in metres
#distance_moved = np.array([0.23, 0.9, 1.75, 2.32])
#distance_moved = np.array([0.18, 0.56, 1.0, 1.3])
distance_moved = np.array([0.18, 0.5, 0.82, 1.2])

velocity_actual = distance_moved/5.0 #m/s    

#plt.plot(velocity_actual, speeds_digital)
slope, intercept, r_value, p_value, std_err = stats.linregress(velocity_actual, speeds_digital)

print(f'slope: {slope}')
print(f'intercept: {intercept}')

def velocity_to_speed(velocity):
    return velocity*slope+intercept
def speed_to_velocity(speed):
    return (speed - intercept)/slope

def phy_to_dig_steering_angle(angle):

##    dis_from_straight_line = 7.0 * np.tan(abs(angle))
##
##    # 7 is the distance between the intersection of the 2 lines coming from the two wheels positions
##    # at the 2 extreme left and right steering position
##
##    if angle < 0:  # turn right
##        return 90 + dis_from_straight_line  # 90 is the angle to input into the car to put front wheel into forward heading
##    elif angle > 0:  # turn left
##        return 90 - dis_from_straight_line
##    else:
##        return 90

    # phy_angle = 90 - angle*180/np.pi

    # return phy_angle

    phy_angle1 = np.arctan2(3.5,5)
    phy_angle2 = np.arctan2(4.4,4)

    # if angle < 0:
    #     dig_angle = 90 + 90/phy_angle1*angle
    #     if dig_angle < 0:
    #         dig_angle = 0
    
    # else:
    #     dig_angle = 90 + 90/phy_angle2*angle
    #     if dig_angle > 180:
    #         dig_angle = 180
    if angle > 0:
        dig_angle = 90 - 90/phy_angle2*angle
        if dig_angle < 0:
            dig_angle = 0
    
    else:
        dig_angle = 90 - 90/phy_angle1*angle
        if dig_angle > 180:
            dig_angle = 180   

    return dig_angle


class BicycleKinematicModel(object):
    def __init__(self, x, y, L, fixed_timestep):
        self.theta = 0

        print(f'setting x to {x} and y to {y} in kinematics model')

        self.x = x
        self.y = y

        self.L = L  # wheel base
        self.fixed_timestep = fixed_timestep

    def forward(self, v, gamma):
        # Implement kinematic model

        print('---')
        print('in side kinematics forward')
##        print('before forward pass')
##        print(f'x: {self.x}')
##        print(f'y: {self.y}')
##        print(f'theta: {self.theta}')

        self.x += self.fixed_timestep * (v * np.cos(self.theta))
        self.y += self.fixed_timestep * (v * np.sin(self.theta))
        self.theta += self.fixed_timestep * (v * np.tan(gamma) / self.L)

        print('after forward pass')
        print(f'x: {self.x}')
        print(f'y: {self.y}')
        print(f'theta: {self.theta}')
        print('end of kinematics forward')
        print('---')

        f_x = open("data_x.txt","a+")
        f_x.write("%f\n" % self.x)
        f_y = open("data_y.txt","a+")
        f_y.write("%f\n" % self.y)

        return self.x, self.y, self.theta

class QueryImage(object):
  """Query Image
  
  Query images form http. eg: queryImage = QueryImage(HOST)

  Attributes:
    host, port. Port default 8080, post need to set when creat a new object

  """
  def __init__(self, host, port=8080, argv="/?action=snapshot"):
    # default port 8080, the same as mjpg-streamer server
    self.host = host
    self.port = port
    self.argv = argv
  
  def queryImage(self):
    """Query Image

    Query images form http.eg:data = queryImage.queryImage()

    Args:
      None

    Return:
      returnmsg.read(), http response data
    """
    http_data = http.client.HTTPConnection(self.host, self.port)
    http_data.putrequest('GET', self.argv)
    http_data.putheader('Host', self.host)
    http_data.putheader('User-agent', 'python-http.client')
    http_data.putheader('Content-type', 'image/jpeg')
    http_data.endheaders()
    returnmsg = http_data.getresponse()

    return returnmsg.read()

  def getQRcode(self):
    image = self.queryImage()
    nparr = np.fromstring(image, np.uint8)
    image_qr = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
##                image_qr = imutils.resize(image_qr,width=500)
##                dim_x = image_qr.shape[0]
##                dim_y = image_qr.shape[1]
##                print(f'dim_x={dim_x}, dim_y={dim_y}')
    qrcode = pyzbar.decode(image_qr)
    return qrcode    

def to_polar(delta_x,delta_y,theta, direction):

    rho = np.sqrt(delta_x ** 2 + delta_y ** 2)

    if direction == -1:
        beta = - np.arctan2(delta_y, delta_x)
        alpha = - theta - beta
    else:
        beta = -np.arctan2(-delta_y,-delta_x)
        alpha = - theta - beta

    # Set limit on alpha    
    if alpha > np.pi/2.0:
        alpha = np.pi/2.0

    if alpha < - np.pi/2.0:
        alpha = - np.pi/2.0     
              
    return rho,alpha,beta, direction

def are_we_there(rho):
    return np.abs(rho) < 0.1

def drive_pose(rho, alpha, beta, direction, ki_model, consts={}):

    kalpha = consts['kalpha']
    kbeta = consts['kbeta']
    krho = consts['krho']
    target = consts['target']
    wheel_base = consts['wheel_base']

    
    z1 = kalpha * alpha + kbeta * beta
    v = rho * krho * direction
    z2 = z1 * direction * wheel_base / np.abs(v)
    gamma = np.arctan(z2)
    
    v = np.clip(v, -speed_to_velocity(100), speed_to_velocity(100))
    gamma = np.clip(gamma, -np.radians(90), np.radians(90))

    c_x, c_y, theta = ki_model.forward(v, gamma)

    delta_x = c_x - target[0]
    delta_y = c_y - target[1]
    
    rho, alpha, beta, direction = to_polar(delta_x, delta_y, theta, direction)

    print(f'alpha: {alpha}')

    there_yet = are_we_there(rho)

    beta = beta + target[2]

    return rho, alpha, beta, direction, v, gamma, there_yet

def drive_qr_pose(speed,waypoint):

        while True:
                query_img = QueryImage(HOST)
                qrcodes = query_img.getQRcode()

                if not qrcodes:
                        continue
                qrcode = qrcodes[0]
                qrData = qrcode.data.decode("utf-8")
                qr_id = get_QR_id(pose, qrData)
                Landmark_pose = QR_memory[qr_id]
                if qrData==waypoint:
                                        
                    (x, y, w, h) = qrcode.rect
                    print(f'x={x}, y={y}, w={w}, h={h}')                          
                    center_x = x + w/2
                    print(f'center = {center_x}')
                    camera_data = get_feature(w,center_x)
                    length = camera_data[0]
                    d = np.sqrt((Landmark_pose[0] - final_pose[0])**2 + (Landmark_pose[1] - final_pose[1])**2)
                    if d>=length:
                            run_action('stop')
                            return
                    
                    delta_center = center_x - 320
                    str_angle = delta_center/320
                    dig_angle = int(90 + str_angle*90)
                    dig_angle = np.clip(dig_angle,70,110) 

                    run_fwturn(dig_angle)
                    run_speed(str(speed))
                    run_action('forward')

def test(target, hyperparams, final_pose):

    fixed_timestep = hyperparams['fixed_timestep']
    wheel_base = 0.15

    ki_model = BicycleKinematicModel(0.0, 0.0, wheel_base, fixed_timestep=fixed_timestep)  # the wheel base of the car is 15cm

    delta_x = 0.0 - target[0]
    delta_y = 0.0 - target[1]
    
    consts = {
        'kalpha': 5, #was 2.2
        'kbeta': -2,
        'krho': 1,
        'target': target,
        'wheel_base': wheel_base
    }
    if delta_x > 0:
        direction = -1
    else:
        direction = 1    

    rho, alpha, beta, direction = to_polar(delta_x, delta_y, 0.0, direction)
    beta = beta + target[2]

    start_t = time.time()
    for t in count():

        rho, alpha, beta, direction, v, gamma, there_yet = drive_pose(rho, alpha, beta, direction, ki_model, consts)

        time.sleep(fixed_timestep)

        dig_angle = int(phy_to_dig_steering_angle(gamma))
        dig_speed = int(velocity_to_speed(abs(v))/2)
        print(f'gamma: {gamma}')
        print(f'direction: {direction}')
        run_fwturn(dig_angle)
        run_speed(str(dig_speed))

        if direction == 1:           
            run_action('forward')
        elif direction == -1:
            run_action('backward')    
        else:
            assert False, 'unrecognized directionn'
            
        if there_yet:
            print(f'We are there at: {target}')
            print(time.time() - start_t)
            break

        if time.time() - start_t > hyperparams['time_limit']:
            print('Exceed time limit')
            break

        query_img = QueryImage(HOST)
        qrcodes = query_img.getQRcode()
        
        if qrcodes:
            print('QR code found')
            qrcode = qrcodes[0]
            qrCodeData = qrcode.data.decode("utf-8")
            print(f'Data = {qrCodeData}')
            print(f'{qrCodeData} found...Now approaching')            
            drive_qr_pose(speed=30,waypoint = final_pose)
            return
        else:
            print('No QR codes found')

def oneQRcode():

    query_img = QueryImage(HOST)
    qrcodes = query_img.getQRcode()

    if not qrcodes:
        return []

    print(f'{len(qrcodes)} QR code(s) found')
    qrcode = qrcodes[0]

    return qrcode

def calibrate_camera():

    image_width = 140 #in pixels
    known_depth = 0.6 #in m
    actual_width = 0.15 #in m
    focal_length = (image_width * known_depth)/actual_width

    return focal_length

def get_feature(image_width, center_x):

    actual_width = 0.15 #in m

    focal_length = calibrate_camera()
    depth = (actual_width * focal_length)/image_width

    #Calculate the angle
    delta_center = center_x - 320    #Check sign #320to240 ERROR0
    angle = delta_center/320*60
    rad_angle = np.radians(angle)

    length = depth/np.cos(rad_angle)

    return length, rad_angle, angle

def get_QR_id(theta,QR_data):

    if -np.pi/4 < theta < np.pi/4:
        if QR_data == "GOAL":
            return 0
        elif QR_data == "Landmark 1":
            return 1
        elif QR_data == "Landmark 2":
            return 2

    if np.pi/4 < theta < 3*np.pi/4:
        if QR_data == "Landmark 3":
            return 3
        elif QR_data == "GOAL":
            return 4
        elif QR_data == "Landmark 1":
            return 5

    if 3*np.pi/4 < theta < np.pi or -np.pi < theta < -3*np.pi/4:
        if QR_data == "GOAL":
            return 8
        elif QR_data == "Landmark 3":
            return 7
        elif QR_data == "Landmark 2":
            return 6

    if -3*np.pi/4 < theta < -np.pi/4:
        if QR_data == "Landmark 3":
            return 11
        elif QR_data == "Landmark 1":
            return 9
        elif QR_data == "Landmark 2":
            return 10
    






def reset_car_state():
    run_action('fwready')
    run_action('bwready')
    run_action('camready')

if __name__ == '__main__':

    time.sleep(7)

    conn_ok = {connection_ok()}
    print(f'Connection Ok: {conn_ok}')

    if not conn_ok:
        sys.exit(1)


    hyperparams = {
        'time_limit': 50,
        'fixed_timestep': 0.22
    }
    
    polys = [[vg.Point(0.0,0.0), vg.Point(3.0,0.0), vg.Point(3.0,3.0), vg.Point(0.0,3.0)],
              [vg.Point(1.0,1.0), vg.Point(2.0,1.0), vg.Point(1.0,2.0), vg.Point(2.0,2.0)]]
    g = vg.VisGraph()
    g.build(polys)
    shortest = g.shortest_path(vg.Point(0.5,0.5), vg.Point(1.5, 2.5))
    for i in range(len(shortest)):
        
        x = shortest[i+1].y - shortest[i].y
        y = shortest[i+1].x - shortest[i].x
        d = np.sqrt(x**2 + y**2)
        rel_pose = round(np.arctan2(y,x) - pose,2)
        rel_x = round(d*np.cos(rel_pose),2)
        rel_y = round(d*np.sin(rel_pose),2)
        pose = np.arctan2(y,x)
        theta_deg = np.rad2deg(rel_pose)
        print(f'x=={rel_x}, y=={rel_y}, theta == {theta_deg}')
        final_pose = [shortest[i+1].x,shortest[i+1].y]
        test([rel_x, rel_y, rel_pose], hyperparams,final_pose)
        
    shortest2 = g.shortest_path(vg.Point(1.5,2.5), vg.Point(2.5, 1.5))
    print(shortest2)
    for i in range(len(shortest)):
        
        x = shortest[i+1].y - shortest[i].y
        y = shortest[i+1].x - shortest[i].x
        d = np.sqrt(x**2 + y**2)
        rel_pose = round(np.arctan2(y,x) - pose,2)
        rel_x = round(d*np.cos(rel_pose),2)
        rel_y = round(d*np.sin(rel_pose),2)
        pose = np.arctan2(y,x)
        theta_deg = np.rad2deg(rel_pose)
        print(f'x=={rel_x}, y=={rel_y}, theta == {theta_deg}')
        final_pose = [shortest[i+1].x,shortest[i+1].y]
        test([rel_x, rel_y, rel_pose], hyperparams,final_pose)

    run_action('stop')
    reset_car_state()


