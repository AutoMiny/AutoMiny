# visual_gps

localize a model car using colored lamp at roof

##calibration Steps:

* find distortion_model of camera and calibrate camera (calibrate_camera package in ROS)

* calibrate system to find desired colors (using cmvison package in ROS)

* find the X and Y scale factors of image  (real map / image)

##run steps:

* read calibrated image (using usb_cam and image_proc packages in ROS)

* find the colored lamps (using cmvison package in ROS)

* filter the blobs and find the biggest one (using detect_roof_rectangles package in ROS)

* use optimal rigid body algorithm and kalman filter to find the transformation between map and position of camera which defines the location of camera/car (using fake_gps package in ROS) 

