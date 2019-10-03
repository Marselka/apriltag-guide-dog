# apriltag-guide-dog

<img src="https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11/tag36_11_00586.png" width=100>

This is a part of a walking robot project.
This part performs estimation of camera frame pose relative to AprilTag and sends commands to Arduino, including number of steps to go either forward-backward or right-left to correct robot's position. 

Python3 script is launched on Raspberry Pi 3 Model B.
Dependencies: `cv2`, `apriltag`, `serial`, `yaml`

Arduino code is just for checking right workflow: toogling led and sending response.
