import requests
import numpy as np
import time
from scipy import stats
from scipy.linalg import block_diag
from itertools import count
import sys
import http

import scipy
from pyzbar import pyzbar
import imutils
import cv2

HOST      = '192.168.1.39'
PORT    = '8000'
autologin = 1
# BASE_URL is variant use to save the format of host and port
BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

circle = np.array([45, 65, 0.2222])

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
  #print('url: %s'% url)
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
  #print('url: %s'% url)
  # Set speed
  __request__(url)

def run_fwturn(angle):

  url = BASE_URL + 'run/?action=fwturn:' + str(angle)
  #print('url: %s'% url)
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

  
##speeds_digital = np.array([25, 50, 75, 100])
##distance_moved = np.array([0.23, 0.9, 1.75, 2.32])

speeds_digital = np.array([25, 50, 75, 100])
distance_moved = np.array([0.18, 0.56, 1.0, 1.3])

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
        

def drive_in_circle(speed, angle, time_step):
    
    run_fwturn(angle)
    run_speed(str(speed))
    run_action('forward')

    time.sleep(time_step)
    run_action('stop')


def get_odometry():
    
    distance = 0.1
    heading_change = np.radians(10)
    return distance, heading_change

def odometry_prediction(x, odometry):
    
    distance = odometry[0]
    heading_change = odometry[1]

    x_next = [0, 0, 0]

    
    x_next[0] = x[0] + distance*np.cos(x[2])
    x_next[1] = x[1] + distance*np.sin(x[2])
    x_next[2] = x[2] + heading_change

    return np.array(x_next)

def get_odometry_new():

    speed = circle[0]
    time = circle[2]
    v = speed_to_velocity(speed)
    distance = v * time
    heading_angle = np.radians(90-circle[1]) #Check sign
    delta_theta = distance/0.15*np.tan(heading_angle)

    return distance, delta_theta

def get_odometry_neg():

    speed = circle[0]
    time = circle[2]
    v = speed_to_velocity(speed)
    distance = v * time
    heading_angle = np.radians(90-circle[1]) #Check sign
    delta_theta = distance/0.15*np.tan(heading_angle)

    return distance, -1*delta_theta

def odometry_prediction_new(x, odometry):

    distance = odometry[0]
    delta_theta = odometry[1]
    
    x_next = [0, 0, 0]

    theta = x[2] + delta_theta
    x_next[0] = x[0] + distance*np.cos(theta)
    x_next[1] = x[1] + distance*np.sin(theta)
    x_next[2] = theta

    print('--------------------')
    #print(str(x))
    print(f'We are @ x={x_next[0]}, y={x_next[1]}, theta={x_next[2]}')
    print('----------------------')

    return np.array(x_next)

def Fx_mat(x, odometry):

    distance = odometry[0]
    theta = x[2] + odometry[1]

    Fx = np.array([
        [1, 0, -distance*np.sin(theta)],
        [0, 1, distance*np.cos(theta)],
        [0, 0, 1]
    ]) 

    return Fx 

def oneQRcode():

    query_img = QueryImage(HOST)
    qrcodes = query_img.getQRcode()

    if not qrcodes:
        return []

    print(f'{len(qrcodes)} QR code(s) found')
    qrcode = qrcodes[0]

    return qrcode

def get_QRcode_id(QR_name,now_seen_at,QR_memory,QR_first_seen):

    #Check if this is the landmark we saw earlier using distance and angle
    for QR_id, name in QR_memory.items():

        if  name == QR_name:
            QR_was_detected_at = QR_first_seen[QR_id]

            #Now check the distance
            delta_angle = matlab_angdiff(now_seen_at[2] , QR_was_detected_at[2])
            print(f'delta_angle = {delta_angle}')
            print(f'{np.abs(now_seen_at[2] - QR_was_detected_at[2])}')
            if np.abs(now_seen_at[0] - QR_was_detected_at[0]) < 0.3 \
                 and np.abs(now_seen_at[1] - QR_was_detected_at[1]) < 0.4 \
                      and np.abs(delta_angle) < 1.0:

                      print(f'Again observing {QR_id}th feature')

                    #This is the QR code now seen which we saw earlier
                      return QR_id

    #If still here means this is a new QR code
    #Check if this is the very 1st QR code
    if list(QR_memory.keys()):
        #This means not the very 1st QR code
        QR_next_id = max(list(QR_memory.keys())) + 1
    else:
        QR_next_id = 0

    print(f'Now observing {QR_next_id}th feature')
    return QR_next_id             

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

def get_camera_data(QR_memory, QR_first_seen, x_estimated):

    QR_code = oneQRcode()

    if not QR_code:
        return None

    #Feature extraction
    (x, y, w, h) = QR_code.rect
    center_x = x + w/2.0
    feature_data = get_feature(w,center_x)
    
    QR_name = QR_code.data.decode()
    now_seen_at = x_estimated[:3]

    


    #Get the QRcode ID
    QRcode_id = get_QRcode_id(QR_name, now_seen_at, QR_memory, QR_first_seen)

    print(f'Observing {QRcode_id}th feature -> {QR_name}')
    
    #Return depth, angle and QRcode ID, QR_name
    return feature_data[0], feature_data[1], QRcode_id, QR_name

def predict_landmark(xr_predicted,QR_id,x_estimated):

    #Get the location of landmark in State vector
    start_id = 3 + QR_id * 2

    #Get the estimate
    QR_location_estimate = x_estimated[start_id:start_id+2]

    #Now predict depth(range) and bearing using the estimate
    delta_x = QR_location_estimate[0] - xr_predicted[0]
    delta_y = QR_location_estimate[1] - xr_predicted[1]

    landmark_pred = [0,0]
    landmark_pred[0] = np.sqrt(delta_x ** 2 + delta_y ** 2)
    phi = np.arctan2(delta_y,delta_x)
    landmark_pred[1] = xr_predicted[2] - phi  #Check sign

    print('|||||||||CHECK THIS||||||||||||')
    print(f'{phi} - {xr_predicted[2]} = {landmark_pred[1]}')
    print('|||||||||CHECK THIS||||||||||||')

    return np.array(landmark_pred)

def matlab_angdiff(alpha1,alpha2=None):

    if alpha2 is not None:
        delta_alpha = alpha1 - alpha2
    else:
        delta_alpha = alpha1

    #Constrain it to [-pi,pi]
    angdiff = np.mod(delta_alpha + np.pi, 2*np.pi) - np.pi

    return angdiff


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
  
   
    run_action('stop')
    reset_car_state()
    run_fwturn(90)
    flag = 0

    x_estimated = np.array([0,0,0], dtype=np.float32)
    #P_estimated = np.diag([0.0001, 0.0001, 0.000025])
    P_estimated = np.diag([0.000, 0.000, 0.0000])
    xr_predicted = x_estimated
    
    #QR_memory stores all the QR codes seen before in order they were seen
    QR_memory = {}
    #QR_first_seen stores the locations where the QR code was first seen. Traverse using QRcode_id 
    QR_first_seen ={}

    odometry = get_odometry_new()       #Change1
    speed_circle = circle[0]
    angle_circle = circle[1]
    timestep_circle = circle[2]

    max_iterations = 190

    for iteration in range(max_iterations):

        print(f'This is the {iteration}th iteration')
        print(f'Estimation: {x_estimated}')

        camera_data = get_camera_data(QR_memory, QR_first_seen, xr_predicted)

        #print(f'Camera data is: {camera_data}')



        drive_in_circle(speed_circle, int(angle_circle), timestep_circle)

        if flag == 0:
            
            if iteration > 95:
                
                angle_circle = 180 - angle_circle
                odometry = get_odometry_neg()
                xr_predicted = np.array([0,0,0], dtype=np.float32)
                print('------------------------------------------------------')
                print('------------------------------------------------------')
                print('||||||||||||||||||||||||||||||||||||||||||||||||||||||')
                print('||||||||||||||||||||||||||||||||||||||||||||||||||||||')
                print('||||||||||||||||||||||||||||||||||||||||||||||||||||||')
                print('------------------------------------------------------')
                print('------------------------------------------------------')
##                x_array = np.array([0,0,0], dtype=np.float32)
##                x_estimated = np.concatenate((x_array, x_estimated[3:]))
                flag = 1
            
        

        #Seperate the robot and landmark states

        xr_estimated = x_estimated[:3]          #ERROR1
        xl_estimated = x_estimated[3:]

        Prr_estimated = P_estimated[:3,:3]

        if iteration == 0:

            Prl_estimated = np.zeros((3,1))
            Pll_estimated = np.zeros((1,1))            

        else:

            Prl_estimated = P_estimated[:3,3:]
            Pll_estimated = P_estimated[3:,3:]

        #Now start predicting
        xr_predicted = odometry_prediction_new(xr_predicted,odometry) #Change1

        # Get the F matrix (Jacobian)
        Fx = Fx_mat(xr_estimated, odometry)

        #Update Prr matrix
        Prr_predicted = Fx @ Prr_estimated @ Fx.T + np.diag([0.1**2, 0.05**2, 0.5**2]) ##Noise added

        #Update Prv matrix
        Prl_predicted = np.matmul(Fx, Prl_estimated)

        #Landmark variables remain unchanged
        Pll_predicted = Pll_estimated
        xl_predcited = xl_estimated

        #Now combine robot and landmark states
        x_predicted = np.concatenate((xr_predicted,xl_predcited)) #ERROR1
        P_predicted = np.vstack((
            np.hstack((Prr_predicted,Prl_predicted)),
            np.hstack((Prl_predicted.T, Pll_predicted))
        ))

        print('...Done with prediction...')
        
        #Clear all previously used variables to avoid dimension error
        delta_feature = None
        S = None
        K = None

        if camera_data is not None:

            feature_data = camera_data[:2]
            QR_id = camera_data[2]
            QR_name = camera_data[3]

##            if QR_id == 2:
##              run_action('stop')
##              reset_car_state()
##              break

            #Check if already seen this QRcode
            if QR_id not in QR_memory:

                #Add this QRcode to memory
                QR_memory[QR_id] = QR_name
                QR_first_seen[QR_id] = xr_predicted

                #Add data to our estimation model
                depth = feature_data[0]
                bearing = feature_data[1] - xr_predicted[2]  #Check for sign #World Frame

                bearing_deg = bearing/np.pi*180
                print(f'Bearing = {bearing_deg}')

                #Estimate the position of the QR code
                QR_position = [xr_predicted[0] + depth*np.cos(bearing), xr_predicted[1] - depth*np.sin(bearing)]  #Check sign

                print(str(QR_position))    
                #Record the length of the State matrix
                n = len(x_predicted)

                #Add this position to the State matrix
                x_estimated = np.append(x_predicted,QR_position)

                #Add this landmark to all the Jacobains and covariance matrix
                #Inverse Observation model
                alpha = feature_data[1]
                theta = xr_predicted[2]

                #Check sign here also
                Gl = np.array([
                    [np.cos(alpha - theta), -depth*np.sin(alpha - theta)],
                    [0.0 - np.sin(alpha - theta), -depth*np.cos(alpha - theta)]
                ])

                Gr = np.array([
                    [1, 0, depth*np.sin(alpha - theta)],
                    [0, 1, 0.0 + depth*np.cos(alpha - theta)]
                ])

                Upper_matrix = np.hstack((np.diag([1] * n),np.zeros((n,2))))
                Lower_matrix = np.hstack((Gr,np.zeros((2,n-3)),Gl))
                Matrix  = np.vstack((Upper_matrix,Lower_matrix))

                #Caclulate the new covariance matrix

                P_estimated = Matrix @ block_diag(P_estimated,np.zeros((2,2))) @ Matrix.T

                print(f'...Feature number {QR_id} added...')


            #Since the QR code is seen before, we'll now update Kalman filter
            else:

                landmark_pred = predict_landmark(xr_predicted, QR_id, x_estimated)

                #How well did we predict?
                #Check sign here as well as in funtion 
                delta_feature = np.array([0.0,0.0])
                delta_feature[0] = feature_data[0] - landmark_pred[0]
                delta_feature[1] = matlab_angdiff(feature_data[1],landmark_pred[1])

                print(f'h(l)= {feature_data[0]} - {landmark_pred[0]} = {delta_feature[0]}\n')
                print(f'h(a) = {feature_data[1]} - {landmark_pred[1]} = {delta_feature[1]}')

                #Position of the landmark
                start_id = 3 + QR_id*2
                xl = x_estimated[start_id:start_id + 2]
                xr = x_estimated[:2]
                Px_Py = xl - xr

                norm = np.linalg.norm(Px_Py)
                #Caculate the H matrix

                #H matrix for landmarks
                Hl = np.array([
                    [Px_Py[0]/norm, Px_Py[1]/norm],
                    [0.0 - (Px_Py[1]/(norm ** 2)), Px_Py[0]/(norm ** 2)]
                ])
##                Hl = np.array([
##                    [0.0, 0.0],
##                    [0.0, 0.0]
##                ])                
##
                #H Matrix for robot #Sign
                Hr = np.array([
                    [0.0 - (Px_Py[0]/norm), 0.0 - (Px_Py[1]/norm), 0],
                    [0.0 - (Px_Py[1]/(norm ** 2)), 0.0 + (Px_Py[0]/(norm ** 2)), 0.0 + 1]
                ]) #Edit_v6
##                Hr = np.array([
##                    [0.0 , 0.0 , 0.0],
##                    [0.0 , 0.0 , 0.0]
##                ])

                #Combine the Hl and Hr matrix into H matrix
                H = np.zeros((2, len(xl_predcited)))

                H[:, start_id-3:QR_id*2+2] = Hl
                H = np.hstack((Hr,H))

                #Calcualte the S matrix
                S = H @ P_predicted @ H.T + np.diag([2.00**2, 4.02**2]) #Noise added here #Edit2_v6

                #Calculate Kalman Gain
                K = P_predicted @ H.T @ np.linalg.inv(S)

                #Measurement update computation
                x_estimated = x_predicted + K @ delta_feature.T
                x_predicted = x_estimated      #ERROR1
                P_estimated = P_predicted - K @ S @ K.T
                P_predicted = P_estimated


    print('---------END----------')
    print(f'x_estimated:\n{x_estimated}\n')
    print('-------SIgma Matrix-------')
    print(f'\n {P_estimated}')

    



            
















        

            





            
    


    

    

