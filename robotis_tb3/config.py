import numpy as np

#wheel variables
leftwheelpos = np.array([0, 0.144, 0.033]) #coordinates of wheel in xyz
rightwheelpos = np.array([0, -0.144, 0.0033])
wheelseperationL = leftwheelpos[1] - rightwheelpos[1] #distance between wheel
wheelR = 0.033 #wheel radius

#motors of turtlebot (velocity)
kv = 5.0 #kv, servo gain
friction = 0.1 #friction of joints
armature = 0.01 #extra rotational interia (Instant Speed)
ctrlrange = [-7.88, 7.88] #maximum range
wheelspeedmax = ctrlrange[1] #maximum output

maxvelocity = wheelR * wheelspeedmax  #m/s maximum velocity turtle can move
wmax = 2 * wheelR * wheelspeedmax / wheelseperationL #maximum rad/s on spot

dt = 0.02 #control period s
physics_dt = 0.002
steps_perctrl = int(dt / physics_dt)

lidarpos = np.array([-0.049, 0, 0.17]) #position of lidar
lidaryaw = 0.0
lidarbeams = 36 
lidarstep = np.deg2rad(10)
lidarmaxrange = 3.5
camerapos= np.array([0.07, 0, 0.15]) #camera 'rgb'
camerayaw = 0.0 
camerafovy = 60 #deg

#sensor names in the xml
encnames = ["enc_left", "enc_right"]
velnames = ["vel_left", "vel_right"]