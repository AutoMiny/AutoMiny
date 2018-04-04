# car_robot_model

This package is adapted from the seat_car_simulator project to provide a robot description of the model car, containing all links and transformations of its components. It also provides a 3D model which can be visualized in RViz.  
The original source code was developed at IRI, Institut de Robòtica i Informàtica Industrial, CSIC-UPC:  
[http://www.iri.upc.edu](http://www.iri.upc.edu)  
[https://gitlab.iri.upc.edu/seat_adc/seat_car_simulator](https://gitlab.iri.upc.edu/seat_adc/seat_car_simulator)


## Show model in RViz

Before starting RViz, ensure you sourced the the workspace of the car_robot_model package. Otherwise RViz would not be able to load the meshes of the car.

    $ source catkin_ws/devel/setup.bash
    $ rviz

Then click on Add -> By display type -> rviz/RobotModel, to show the 3D model.

