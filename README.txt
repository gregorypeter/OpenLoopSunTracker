* Open loop tracking for CPV test setup *
* Michael Lipski			*
* Summer 2016				*

Repository contains the sketches for open-loop tracking.  Pulls date, time, latitude and longitude from the 
GPS unit and calculates the position of the sun.  Solar coordinates then get transformed into coordinates in the plane of the microcell, in terms of mm of displacement from the test jig "origin".

Copy folders from "libraries" into your "Arduino/libraries" folder and restart the IDE before using.