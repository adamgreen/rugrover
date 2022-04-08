/*  Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Viewing frustrum of the ST VL53L1X range finders that I want to place on my bot. 

// ************
//  Parameters
// ************
// Angle at which sensors on mounted on either side of the central sensor. 
mountAngle = 19.0;
// Radius from center of robot base, at which the sensors will be mounted. 
mountRadius = 160.0/2 - 40.0;
// The diagonal field of view for the sensor. 
fovDiagonal = 27;
// Assuming square pixels, 1:1 aspect ration, then calculate horizontal/vertical FoV. 
fov = sqrt(fovDiagonal^2/2);
// Draw frustrum out to this distance: 1.3m.
dist = 1.3 * 1000;
// Size of sensor in X/Y dimensions, starting point of frustrum. 
sensorSize = 1.0;




frustrum(35, -mountAngle);
frustrum(40, 0);
frustrum(35, mountAngle);

frustrum(35, 180-mountAngle);
frustrum(35, 180+mountAngle);




module frustrum(radius, angle) {
    outerSize = 2*dist*tan(fov/2);

    rotate([0, 0, angle]) {
        hull() {
            translate([0, radius, 0])
                cube([sensorSize, 0.001, sensorSize], center=true);
            translate([0, radius+dist, 0])
                cube([outerSize, 0.001, outerSize], center=true);
        }
    }
}



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 1;
