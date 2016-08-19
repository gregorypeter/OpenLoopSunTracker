* Open loop tracking for CPV test setup *
* Michael Lipski			*
* Summer 2016				*

Repository contains the sketches for open-loop tracking.  Pulls date, time, latitude and longitude from the 
GPS unit and calculates the position of the sun.  Solar coordinates then get transformed into coordinates in the plane of the microcell, in terms of mm of displacement from the test jig "origin".

Copy folders from "libraries" into your "Arduino/libraries" folder and restart the IDE before using.

* *  Sketch Descriptions  * *
feedforward_binary: Open-loop tracking; Zaber Binary Protocol; f20 lens data
feedforward_binary_2: Open-loop tracking; Zaber Binary Protocol; f80 lens data
feedforward_binary_test: Indoor positioning test with optics; Zaber Binary Protocol; f20 lens data
feedforward_binary_test_2: Indoor positioning test with optics; Zaber Binary Protocol; f80 lens data
feedforward_initial: Open-loop tracking; Zaber ASCII Protocol; f20 lens data
feedforward_test: Indoor positioning test with optics; Zaber ASCII Protocol; f20 lens data
gps_reader_lcd: Outputs date, time, lat. and long. from gps to LCD screen
pitch_heading_test: Uses 6DOF IMU to find panel tilt and heading
pitch_heading_test_lcd; Uses 6DOF IMU to find panel tilt and heading; outputs to LCD screen
solar2planar: Demonstrates coordinate transform from solar coords to planar coords; f20 lens data
solar2planar2: Demonstrates coordinate transform from solar coords to planar coords; f80 lens data