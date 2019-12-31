import requests
import numpy as np
import time
from scipy import stats
from itertools import count
import sys
import http

from pyzbar import pyzbar
import imutils
import cv2

HOST      = '192.168.1.39'
PORT    = '8000'
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

  
speeds_digital = np.array([25, 50, 75, 100])
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

def are_we_there_qr(w):
    return w>= 125 

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

def drive_qr_pose(speed,final_width):

        while True:
                query_img = QueryImage(HOST)
                qrcodes = query_img.getQRcode()

                if not qrcodes:
                        continue
                qrcode = qrcodes[0]
                (x, y, w, h) = qrcode.rect
                print(f'x={x}, y={y}, w={w}, h={h}')                          
                center_x = x + w/2
                print(f'center = {center_x}')
                if w>=final_width:
                        run_action('stop')
                        return
                
                delta_center = center_x - 320
                str_angle = delta_center/320
##                dig_angle = int(phy_to_dig_steering_angle(str_angle))
                dig_angle = int(90 + str_angle*90)
                dig_angle = np.clip(dig_angle,45,135)

                run_fwturn(dig_angle)
                run_speed(str(speed))
                run_action('forward')

##                time.sleep(0.2)

##                run_action('stop')

                



    


def test(target, hyperparams, waypoint,final_width):

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

        query_img = QueryImage(HOST)
        qrcodes = query_img.getQRcode()
        
        if qrcodes:
            print('QR code found')
            qrcode = qrcodes[0]
            qrCodeData = qrcode.data.decode("utf-8")
            print(f'Data = {qrCodeData}')
            if qrCodeData == waypoint:
                print(f'{qrCodeData} found...Now approaching')            
                drive_qr_pose(speed=30,final_width=final_width)
                return
        else:
            print('No QR codes found')

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
    
   
    run_action('stop')
    reset_car_state()
    run_fwturn(90)
    
##    while True:
##            query_img = QueryImage(HOST)
##            qrcodes = query_img.getQRcode()
##            if not qrcodes:
##                    print('ERROR: No barcode visible')
##            else:
##                    print('Barcode found')
##                    qrcode = qrcodes[0]
##                    (x, y, w, h) = qrcode.rect
##                    print(f'x={x}, y={y}, w={w}, h={h}')
##                    center_x = x + w/2
##                    print(f'center = {center_x}')
##    drive_qr_pose(speed = 30)
    
    reset_car_state()
    test([0.535, 0.02, 0], hyperparams, waypoint="NIL",final_width=125)
    
    reset_car_state()
    test([0.45, 0.5, 1.1], hyperparams, waypoint="Landmark 1",final_width=220)
    
##    reset_car_state()
##    test([0.4, 0.4, 1.1], hyperparams, waypoint="Landmark 2",final_width=125)
    
    reset_car_state()
    test([0.45,0.5, 1.1], hyperparams, waypoint="Landmark 2",final_width=125)       

    reset_car_state()
    test([1,0.5, np.radians(90)], hyperparams, waypoint="Landmark 3",final_width=200)    

    reset_car_state()
    test([0.1,0.6, 1.57], hyperparams, waypoint="GOAL",final_width=160)

##    reset_car_state()
##    test([0.6,0.5, 1.1], hyperparams, waypoint="GOAL",final_width=160)       
    
    run_action('stop')
    reset_car_state()    

##    print("[INFO] Found {} barcode: {}".format(qrType, qrData))
    


    

    

