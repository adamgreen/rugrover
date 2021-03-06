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
// A test rectangle to see how flexible PETG prints are at specified thickness.

// ************
//  Parameters
// ************
thickness = 4.0;



cube([160.0, 40.0, thickness], center=true);



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 1;
