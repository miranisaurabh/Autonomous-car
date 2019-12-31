import requests
import numpy as np
import time
from scipy import stats
from itertools import count
import sys

HOST      = '192.168.1.39'
PORT 	  = '8000'
autologin = 1
# BASE_URL is variant use to save the format of host and port
BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

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
distance_moved = np.array([0.23, 0.9, 1.75, 2.32])

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

    phy_angle = 90 - angle*180/np.pi

    return phy_angle

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

def test(target, hyperparams):

    fixed_timestep = hyperparams['fixed_timestep']
    wheel_base = 0.15

    ki_model = BicycleKinematicModel(0.0, 0.0, wheel_base, fixed_timestep=fixed_timestep)  # the wheel base of the car is 15cm

    delta_x = 0.0 - target[0]
    delta_y = 0.0 - target[1]
    
    consts = {
        'kalpha': 5,
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
        dig_speed = int(velocity_to_speed(abs(v)))
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
        'fixed_timestep': 0.17
    }

##    reset_car_state()
##    test([-1, 0, 0], hyperparams)
    
##    reset_car_state()
##    test([0, 1, np.radians(90)], hyperparams)
##
##    reset_car_state()
##    test([0, 1, -np.radians(90)], hyperparams)

##    reset_car_state()
##    test([0, 1, -np.radians(90)], hyperparams)    
##
##    reset_car_state()
##    test([1, 1, np.radians(45)], hyperparams)
##
    reset_car_state()
    test([1, 0, 0], hyperparams)        

    run_action('stop')
    reset_car_state()
