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
// Show everything on my RugRover robot - Chassis, bumper skirt, external components like motors, sensors, etc.
include <Common.scad>


rotate([0, 180, 180]) {
    base();
    // Draw all of the extra parts, those not 3D printed, such as motors, caster, sensors, etc.
    %union() {
        translate([motorXOffset, 0, motorZOffset]) motor();
        translate([-motorXOffset, 0, motorZOffset]) rotate([0, 0, 180]) motor();
        translate([wheelXOffset, 0, wheelZOffset]) wheel();
        translate([-wheelXOffset, 0, wheelZOffset]) rotate([0, 0, 180]) wheel();
        translate([0, casterYOffset, casterZOffset]) rotate([0, 180, 0]) caster();
        translate([0, 0, cliffDetectorMountThickness])
            placeCliffDetectors()
                cliffDetector();
        placeBumperSwitches();
        translate([0, 0, -nRF52DK_MountingHeight])
            placeNRF52DK()
                nRF52DK_PCB();
        placeNRF52DK()
            nRF52DK_MountingHoles(diameter=nRF52DK_MountingHoleDiameter*1.5, height=nRF52DK_MountingHeight);
        translate([0, batteryYOffset, baseThickness])
            batteryPack3_2();
        flexibleBumperStandoffs();
        shield();
    }
    %bumperSkirt();
    counterWeightHolder();

    // Draw ground plane to see if all wheels fall on it.
    *translate([0, 0, wheelZOffset+wheelDiameter/2-0.001])
        cube([baseDiameter*2, baseDiameter*2, 0.001], center=true);
}
