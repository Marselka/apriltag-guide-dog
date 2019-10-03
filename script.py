import apriltag
import cv2
import time
import numpy as np
import serial
import yaml

with open('./cam_intrinsics/camchain-homemarosbagscam.yaml') as f:
    intrinsics = yaml.safe_load(f)['cam0']['intrinsics']

dimension = 0.1728
step_size = 0.06


port = '/dev/ttyACM0'
cam_device = 0

def arduino_echo(string):
    out = ''
    while ser.inWaiting() > 0:
        out += str(ser.read(1).decode())        
    if out != '':
        print (string + out)
        
def inv(pose):
    pose_inv = np.eye(4)
    pose_inv[:3,:3] = pose[:3,:3].T
    pose_inv[:3,3] = - pose_inv[:3,:3] @ pose[:3,3]
    return pose_inv

def get_pose():
    res = None
    pose = None
    et, frame = cam.read()
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    result = detector.detect(img)
    if len(result) > 0:
        res = detector.detection_pose(result[0], intrinsics, dimension)
        res = res[0]
        pose = inv(res)
    return res, pose

def cmd(steps0, steps1):
    if steps0 > steps1:
        steps = steps0
        direction = 0
    else:
        steps = steps1
        direction = 1
        
    if direction == 0:
        if np.abs(steps) > 0:
            foot = '0'
        else:
            foot = '1'
    else:
        if np.abs(steps) > 0:
            foot = '2'
        else:
            foot = '3'
    return ('v' + ' ' + foot + ' ' + str(steps)).encode()

def create_command(pose):
    displ = - pose[:3,3][::2]
    steps0, steps1 = np.rint(displ/step_size)
    steps0, steps1 = int(steps0), int(steps1)
    command = cmd(steps0, steps1)
    return command

def debug_info(i, command):
    print('Catched frames:', i)
    print('Command:', command.decode())
    time.sleep(0.1)
    arduino_echo('Arduino answer: ')
    print('============================')

ser = serial.Serial(
    port=port,
    baudrate=9600,
    #parity=serial.PARITY_ODD,
    #stopbits=serial.STOPBITS_TWO,
    #bytesize=serial.SEVENBITS
)

cam = cv2.VideoCapture(cam_device)
options = apriltag.DetectorOptions(families='tag36h11',
                                 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=True,
                                 debug=False,
                                 quad_contours=True)

detector = apriltag.Detector()

ser.isOpen()

i = 0

while(1):
    try:
        res, pose = get_pose()
        if pose is not None:
            i += 1
            command = create_command(pose)
            ser.write(command)
            debug_info(i, command)
    except KeyboardInterrupt:
        print('Interrupted')
        ser.close()
        cam.release()
        exit(0)