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
// Variables and modules used to model the various parts of my RugRover bot.
include <ExternalParts.scad>

// ************
//  Parameters
// ************
// Tight clearance.
clearanceTight = 0.2;
// Loose clearance.
clearanceLoose = 0.3;


// The diameter of the circular robot base.
baseDiameter = 160.0;
// The thickness of the circular robot base.
baseThickness = 5.0;
// The thickness of the motor mount.
motorMountThickness = 2.5;
// Clearance between robot base and wheel.
wheelClearance = 2.0;
// Additional clearance to have between caster wheel and edge of robot base.
casterClearance = 1.0;
// Clearance around the motor for the mount.
motorMountClearance = clearanceLoose;
// How much larger should the caster mount be than the caster plate itself.
casterMountClearance = 2.0;
// The radius of the rounded edges of the caster mount.
casterMountRadius = 1.0;
// Raise the cliff detctors by this amount to make room for the header pins
// sticking out the back of the PCB.
cliffDetectorMountThickness = 3.0;
// The amount of the cliff detector PCB to the left hanging off of the mount, where header
// pins are located.
cliffDetectorMountOffset = 2.54;
// Angles at which the 3 bumper switches should be placed around the robot base.
bumperSwitchAngles = [-25, 151, -151];
// How many degrees each bumper switch should be roated to get lever facing correct direction.
bumperSwitchFlips = [180, 0, 180];
// The length of the slots used to mount the buper switches onto the base.
bumperSwitchSlotLength = 11.5;
// Angles to locate the mounting holes for the bumper.
bumperMountAngles = [0, 120, -120];
// Offset of the bumper mounting holes from the outside diameter.
bumperMountOffset = 22.0;
// Bumper mounting hole diameters for M3 bolts.
bumperMountDiameter = 3.0 + clearanceLoose;
// Bumper mounting countersink hole size to allow access through caster mount block.
bumperMountCountersinkDiameter = 6.0;
// How far from the robot base should the nRF52DK PCB be mounted.
nRF52DK_MountingHeight = 6.0;
// Diameter of the hole used to attach standoffs for the nRF52DK PCB.
nRF52DK_MountingHoleDiameter = 3.0;
// Diameter of hole in middle of base for pulling cables through.
cableHoleDiameter = 25.0;
// Where the battery pack should be placed on the bottom of the bot, in the Y axis.
batteryYOffset = -38;
// Dimensions of slots used to place straps through to hold battery to underside of chassis.
batteryStrapSlotWidth = 2.5;
batteryStrapSlotLength = 5.0;
// The ratio of battery diameters to either side of the X center line, the battery strap
// slots should be placed. 1.5 would put close to battery and larger pulls it out a bit.
batteryStrapSlotXRatio = 1.7;
// The extra amount to slide the battery strap slot down in the Y axis from the inside edge
// of the battery pack.
batteryStrapSlotYOffset = 12.0;
// Dimensions of ridges placed around battery pack to keep it from moving in X/Y plane.
batteryPackRidgeInsideLength = 10.0;
batteryPackRidgeOutsideLength = 20.0;
batteryPackRidgeWidth = 2.0;
batteryPackRidgeHeight = batteryDiameter/4;
batteryPackRidgeClearance = clearanceLoose;
// How much larger should the diameter of the bumper skirt be than the chassis.
bumperDiameterClearance = 4.0;
// How thick should the bumper skirt be?
// UNDONE: Will depend on slicer settings.
bumperThickness = 0.3 * 4;
// Thickness of the top of the  bumper skirt.
bumperTopThickness = 2.0;
// How far above the ground should the skirt be? Should be low enough to detect things that
// could hit battery pack, motors, etc.
bumperGroundClearance = 20.0;
// Diameter of flexible bumper standoffs.
bumperStandoffDiameter = 6.0;
// How tall are the flexible bumper standoffs.
bumperStandoffHeight = 43.0;
// The amount of extra top on the bumper skirt there should be to cover the flexible standoffs.
bumperTopClearance = 2.0;
// The desired diameter of the roll of tungsten to be placed at the back of the bot.
counterWeightDiameter = 16.0;
// The thickness of the holder to contain the roll of tungsten at the back of the bot.
counterWeightHolderThickness = 1.0;
// The ratio of the height of the holder to its width.
counterWeightHolderRatio = 2.0;
// The number of degrees that holder should take up at the back of the bot.
counterWeightHolderAngle = 100.0;
// The inside of the holder will be hollowed out this much less in degrees on either end.
counterWeightHolderInsideOffsetAngle = 1.0;
// Amount of clearance to place around bumper switch.
counterWeightHolderSwitchClearance = 2.5;
// Amount of clearance to place around flexible bumper standoff.
counterWeightHolderBumperHoleClearance = 5.0;




// *******************
//  Calculated Values
// *******************
// Determine the offset of the outside of the wheel needed to make sure that it is fully enclosed
// by the base.
encloseOffset = sqrt((baseDiameter/2)^2-(wheelDiameter/2)^2);
// Offset wheels so that they fall fully within the base.
wheelXOffset = encloseOffset-wheelThickness/2;
// Offset motors so that output shaft is flush with outside of wheel.
motorXOffset = encloseOffset-(motorGearboxCollarDepth+motorGearboxShaftLength);
// Motors should sit on top of the base.
motorZOffset = motorGearboxDiameter/2 + baseThickness;
// Offset wheels up off the base the same amount as the motors.
wheelZOffset = motorZOffset;
// Make sure that caster wheel stays within confines of body so that it never touches wrap around bumper.
casterYOffset = baseDiameter/2 - casterArmXOffset - casterDiameter/2 - casterClearance;
// Total caster height.
casterTotalHeight = casterPlateThickness + casterBearingHeight + casterArmZOffset + casterDiameter/2;
// Amount of wheel below the robot base.
wheelHeight = wheelDiameter/2 + wheelZOffset;
// How much taller is the caster in comparison to the drive wheels.
casterZOffset = wheelHeight - casterTotalHeight;
// Height of the casterMount.
casterMountHeight = casterZOffset - baseThickness;
// Inner diameter of the bumper skirt.
bumperID = baseDiameter + bumperDiameterClearance;
// Outer diameter of the bumper skirt.
bumperOD = bumperID + 2*bumperThickness;
// The Z offset where the bottom of the skirt should be.
bumperZOffset = wheelHeight - bumperGroundClearance;
// How tall should the bumper skirt be? Determined by ground clearance and standoff height.
bumperHeight = wheelHeight-bumperGroundClearance+bumperStandoffHeight;
// The diameter of the large hole in the top of the bumper skirt.
bumperTopID = baseDiameter-bumperMountOffset*2-bumperStandoffDiameter-2*bumperTopClearance;
// Outside width of the holder.
counterWeightHolderOutsideWidth = counterWeightDiameter + 2*counterWeightHolderThickness;
// Radius on the base on which the cylinder will be centered.
counterWeightHolderOffsetRadius = baseDiameter/2-counterWeightHolderOutsideWidth/2;




// Circular robot base with mounting for motors, caster, sensors, etc.
module base() {
    difference() {
        union() {
            cylinder(h=baseThickness, d=baseDiameter);
            motorMounts();
            casterMount();
            placeCliffDetectors()
                mountForCliffDetectors();
            batteryPackRidges();
        }
        // Things to remove from the circular base.
        wheelCutout(0);
        wheelCutout(1);
        casterMountHoles();
        placeCliffDetectors()
            holesForCliffDetectors();
        bumperSwitchCutouts();
        pcbMountHoles();
        cableHole();
        bumperMountHoles();
        batterySlotCutouts();
    }
}

// Cutout slots on the side to fit the wheels.
module wheelCutout(leftWheel) {
    xOffset = baseDiameter/4+encloseOffset-wheelThickness-wheelClearance;
    rotate([0, 0, leftWheel * 180.0])
        translate([xOffset, 0, baseThickness/2])
            cube([baseDiameter/2, wheelDiameter+2*wheelClearance, baseThickness+0.02], center=true);
}

module motorMounts() {
    motorMount();
    rotate([0, 0, 180]) motorMount();
}

// Vertical motor mount with holes for gearbox collar, output shaft, and screw holes.
module motorMount() {
    xOffset = motorXOffset+motorMountThickness/2;
    mountWidth = motorGearboxDiameter + 2*motorMountClearance + 2*motorMountThickness;

    translate([xOffset, 0, motorZOffset])
        rotate([0, 90, 0])
            difference() {
                cube([motorGearboxDiameter, mountWidth, motorMountThickness], center=true);
                motorCollarCutout();
                motorScrewCutout(1.0);
                motorScrewCutout(-1.0);

            }
    motorMountSupport(1.0);
    motorMountSupport(-1.0);
}

// Cutout hole in motor mount to fit the collar at end of gearbox. This hole is made extra large to
// give some wiggle/clearance room.
module motorCollarCutout() {
    cylinder(h=motorMountThickness+0.02, d=motorGearboxCollarDiameter+4*motorMountClearance, center=true);
}

// Cutout holes used to screw the motor to the mount.
module motorScrewCutout(offset) {
    size = motorGearboxHoleDiameter + clearanceLoose;

    translate([-motorMountClearance, offset*motorGearboxHoleOffsetRadius, 0])
        cylinder(h=motorMountThickness+0.02, d=size, center=true);
}

// Triangular supports to strengthen the motor mounts.
module motorMountSupport(offset) {
    yOffset = motorMountThickness/2+motorGearboxDiameter/2+motorMountClearance;
    translate([motorXOffset, offset*yOffset, baseThickness])
        rotate([90, 0, 0])
            linear_extrude(motorMountThickness, center=true)
                polygon([[0, 0], [-motorGearboxDiameter, 0], [0, motorGearboxDiameter]]);
}

// Block with screw holes to which the caster is mounted so that the caster wheel sits even with
// the main drive wheels.
module casterMount() {
    width = casterPlateHeight+casterMountClearance;
    length = casterPlateWidth+casterMountClearance;

    translate([0, casterYOffset, casterMountHeight/2+baseThickness])
        roundedRect(width, length, casterMountHeight, casterMountRadius);
}

module roundedRect(width, length, height, radius) {
    hull()
        for (x=[-1:2:1])
            for (y=[-1:2:1])
                translate([x*(width/2-radius), y*(length/2-radius), 0])
                    cylinder(h=height, r=radius, center=true);
}

// The 4 holes used to mount the caster.
module casterMountHoles() {
    for (x=[-1:2:1])
        for (y=[-1:2:1])
            casterMountHole(x, y);
}

// Holes to be placed in caster mount block.
module casterMountHole(x, y) {
    xOff = x*casterHoleYSpacing/2;
    yOff = casterYOffset + y*casterHoleXSpacing/2;
    thickness = baseThickness + casterMountHeight+0.02;

    translate([xOff, yOff, thickness/2-0.01])
        cylinder(h=thickness, d=casterHoleDiameter, center=true);
    translate([xOff, yOff, -0.001])
        rotate([180, 0, 0])
            threadedInsertHole();
}

// Threaded inserts require a hole of diameter 5.23mm down to 5.05mm over length (6.35) + 0.04mm = 6.39
// Cylinder will be centered on origin, facing upwards, with top aligned to z=0.
module threadedInsertHole() {
    topDiameter = 5.23;
    bottomDiameter = 5.05;
    length = 6.35 + 0.04;

    translate([0, 0, -length])
        cylinder(h=length, d1=bottomDiameter, d2=topDiameter);
}

// Places its child (a cliffDetector or the holes onto which a cliff detector is mounted.)
module placeCliffDetectors() {
    // If you push the middle of the cliff detector out to the edge of the base, then the
    // ends will hang over the base by this amount.
    overHang = sqrt((baseDiameter/2)^2 + (cliffDetectorHeight/2)^2) - baseDiameter/2;
    // Translate the cliff detector to the right by the amount before rotating it into position in
    // front of the wheel.
    xOffset = -cliffDetectorWidth + baseDiameter/2 - overHang;
    // [x2, y2] is the position of the innermost part of the wheel cutout. This is what we want
    // the cliff detector to mount up against.
    x2 = wheelXOffset-wheelThickness/2-wheelClearance;
    y2 = (wheelDiameter/2+wheelClearance);
    // Calculate angle x axis to the point on this innermost part of the wheel cutout.
    angleToInnerEdgeOfCutout = atan2(y2, x2);
    insideRadius = sqrt(x2^2+y2^2);
    // Calculate angle subtended by cliff detector from its middle to the edge that should be flush
    // with the wheel cutout.
    angleFromCenterToEnd = atan2(cliffDetectorHeight/2, insideRadius);
    angle=angleToInnerEdgeOfCutout + angleFromCenterToEnd;

    rotate([0, 0, -angle])
        translate([xOffset, -cliffDetectorHeight/2, baseThickness])
            children();
    rotate([0, 0, 180+angle])
        translate([xOffset, -cliffDetectorHeight/2, baseThickness])
            children();
}

// Mount for cliff detector to make room for back of header pins.
module mountForCliffDetectors() {
    width = cliffDetectorWidth - cliffDetectorMountOffset;

    translate([cliffDetectorMountOffset, 0, 0])
        cube([width, cliffDetectorHeight, cliffDetectorMountThickness]);
}

// Holes for a cliff detector.
module holesForCliffDetectors() {
    thickness = baseThickness + cliffDetectorMountThickness;

    for (i=[-1:2:1])
        translate([cliffDetectorMountingHoleXOffset, cliffDetectorHeight/2+i*cliffDetectorMountingHoleYOffset, -baseThickness-0.01])
            cylinder(h=thickness+0.02, d=cliffDetectorMountingHoleDiameter, center=false);
}
// Place the buper switches around the periphery of the robot base, on its top surface.
module placeBumperSwitches() {
    for (i=[0:2]) {
        rotate([0, 0, bumperSwitchAngles[i]])
            translate([0, baseDiameter/2-bumperSwitchHeight/2-2.75, baseThickness+bumperSwitchThickness/2])
                rotate([0, bumperSwitchFlips[i], 0])
                    bumperSwitch();
    }
}

// Locate cutouts around the base onto which the bumper switches can be mounted.
module bumperSwitchCutouts() {
    for (a=[0:2]) {
        rotate([0, 0, bumperSwitchAngles[a]]) {
            for(i=[-1:2:1]) {
                translate([i*bumperSwitchHoleXOffset, baseDiameter/2-bumperSwitchSlotLength/2, baseThickness/2])
                    roundedSlot(bumperSwitchHoleDiameter, bumperSwitchSlotLength, baseThickness+0.02);
            }
        }
    }
}

// Make slot with rounded ends, centered on the origin.
module roundedSlot(endDiameter, length, thickness) {
    hull() {
        for(i = [-1:2:1]) {
            translate([0, i*length/2, 0])
                cylinder(h=thickness+0.02, d=endDiameter, center=true);
        }
    }
}

// The holes used to screw on the standoffs used to mount the nRF52DK PCB.
module pcbMountHoles() {
    translate([0, 0, baseThickness+0.01])
        placeNRF52DK()
            nRF52DK_MountingHoles(diameter=nRF52DK_MountingHoleDiameter, height=baseThickness+0.02);
}

// Place the nRF52DK and its mounting holes in the correct location.
module placeNRF52DK() {
    rotate([180, 0, 90])
        translate([-nRF52DK_Length/2, -nRF52DK_Width/2, 0])
            children();
}

// Place cylinders at the location of the mounting holes in the nRF52DK PCB.
module nRF52DK_MountingHoles(diameter, height) {
    for (i=nRF52DK_MountingHoles) {
        translate(i)
            cylinder(h=height, d=diameter);
    }
}

// This is the cable through which the cables will be routed from the bottom of the bot,
// up to the PCB on the top. This includes motor, encoder, cliff detector, and bumper
// switch cabling.
module cableHole() {
    translate([0, 0, -0.001])
        cylinder(h=baseThickness+0.002, d=cableHoleDiameter);
}

// Location of holes to mount rubber standoffs for bumper skirt.
module bumperMountHoles() {
    translate([0, 0, -0.001])
        placeBumperMountHoles()
            cylinder(h=baseThickness+0.002, d=bumperMountDiameter);
    // Need to countersink through the thick caster mount.
    translate([0, baseDiameter/2-bumperMountOffset, baseThickness-0.001])
        cylinder(h=casterMountHeight+0.002, d=bumperMountCountersinkDiameter);
}

// Rotates and offsets the bumper mounts/holes into the correct location.
module placeBumperMountHoles() {
    for (i=bumperMountAngles)
        rotate([0, 0, i])
            translate([0, baseDiameter/2-bumperMountOffset, 0])
                children();
}

// Cutouts for straps used to hold battery pack in place.
module batterySlotCutouts() {
    for(y=[-1:2:1])
        for(x=[-1:2:1])
            batterySlotCutout(x, y);
}

module batterySlotCutout(x, y) {
    yOffset = batteryYOffset+y*(-batteryStrapSlotLength/2+batteryLength/2-batteryStrapSlotYOffset);

    translate([x*batteryDiameter*batteryStrapSlotXRatio, yOffset, baseThickness/2+0.01])
        roundedSlot(endDiameter=batteryStrapSlotWidth, length=batteryStrapSlotLength, thickness=baseThickness+0.02);
}

// Add ridges around battery pack to keep it from moving in X/Y plane.
module batteryPackRidges() {
    batteryPackInsideRidge(1);
    batteryPackInsideRidge(-1);
    batteryPackOutsideRidge();
}

module batteryPackInsideRidge(x) {
    batteryPackWidth = 3*batteryDiameter;
    xOffset = x*(batteryPackWidth/2-batteryPackRidgeInsideLength/2);
    yOffset = batteryYOffset+1*(batteryLength/2+batteryPackRidgeWidth/2+batteryPackRidgeClearance);
    zOffset = batteryPackRidgeHeight/2+baseThickness;
    translate([xOffset, yOffset, zOffset])
        rotate([0, 0, 90])
            roundedSlot(batteryPackRidgeWidth, batteryPackRidgeInsideLength, batteryPackRidgeHeight);
}

module batteryPackOutsideRidge() {
    batteryPackWidth = 3*batteryDiameter;
    yOffset = batteryYOffset+(-1)*(batteryLength/2+batteryPackRidgeWidth/2+batteryPackRidgeClearance);
    zOffset = batteryPackRidgeHeight/2+baseThickness;
    translate([0, yOffset, zOffset])
        rotate([0, 0, 90])
            roundedSlot(batteryPackRidgeWidth, batteryPackRidgeOutsideLength-batteryPackRidgeWidth, batteryPackRidgeHeight);
}

module flexibleBumperStandoffs() {
    translate([0, 0, -bumperStandoffHeight])
        placeBumperMountHoles()
            cylinder(h=bumperStandoffHeight, d=bumperStandoffDiameter);
}




// Separate bumper skirt part that goes around the chassis.
module bumperSkirt() {
    bumperSkirtSides();
    bumperSkirtTop();
}

module bumperSkirtSides() {
    translate([0, 0, -bumperHeight/2+bumperZOffset]) {
        difference() {
            cylinder(h=bumperHeight, d=bumperOD, center=true);
            cylinder(h=bumperHeight+0.002, d=bumperID, center=true);
        }
    }
}

module bumperSkirtTop() {
    translate([0, 0, -bumperTopThickness/2-(bumperHeight-bumperZOffset)]) {
        difference() {
            cylinder(h=bumperTopThickness, d=bumperOD, center=true);
            cylinder(h=bumperTopThickness+0.002, d=bumperTopID, center=true);
            placeBumperMountHoles()
                cylinder(h=bumperTopThickness+0.002, d=bumperMountDiameter, center=true);
        }
    }
}




// Separate curved cylindrical holder that goes on back of robot to hold the tungsten counter weight.
module counterWeightHolder() {
    difference() {
        union() {
            counterWeightCylinder(counterWeightHolderAngle, counterWeightDiameter+2*counterWeightHolderThickness);
            counterWeightHolderMount();
        }
        translate([0, 0, 0.001])
            counterWeightCylinder(counterWeightHolderAngle-2*counterWeightHolderInsideOffsetAngle, counterWeightDiameter);
        counterWeightHolderHoleForSwitch();
    }
}

module counterWeightCylinder(spanAngle, width) {
    height = counterWeightHolderRatio * width;
    rotate([0, 0, 90-spanAngle/2]) {
        difference() {
            rotate_extrude(angle=spanAngle)
                translate([counterWeightHolderOffsetRadius, 0])
                    scale([1.0, counterWeightHolderRatio])
                        circle(d=width);
            translate([0, 0, height/2])
                cube([baseDiameter, baseDiameter, height], center=true);
        }
    }
}

// Cutout for a hole that makes room for the bolts that mount the back bumper switch.
module counterWeightHolderHoleForSwitch() {
    rotate([0, 0, bumperSwitchAngles[0]])
        translate([0, counterWeightHolderOffsetRadius, -counterWeightHolderSwitchClearance/2+0.001])
            cube([2*(bumperSwitchHoleXOffset + counterWeightHolderSwitchClearance), 1.25*counterWeightHolderOutsideWidth, counterWeightHolderSwitchClearance], center=true);
}

// This is the flat rectangular piece (to match the caster mount) that holds counter weight holder to base.
module counterWeightHolderMount() {
    width = casterPlateHeight+casterMountClearance;
    length = casterPlateWidth+casterMountClearance;

    difference() {
        translate([0, casterYOffset, -counterWeightHolderThickness/2])
            roundedRect(width, length, counterWeightHolderThickness, casterMountRadius);
        counterWeightHolderMountHoles();
        counterWeightHolderHoleForBumperStandoff();
    }
}

// The 2 holes used to mount the counter weight holder to the base, line with 2 of the caster holes.
module counterWeightHolderMountHoles() {
    for (x=[-1:2:1])
        counterWeightHolderMountHole(x);
}

// Holes to be placed for mounting the counter weight holder to the base.
module counterWeightHolderMountHole(x) {
    xOff = x*casterHoleYSpacing/2;
    yOff = casterYOffset + -1*casterHoleXSpacing/2;
    thickness = counterWeightHolderThickness+0.02;

    translate([xOff, yOff, -thickness/2+0.01])
        cylinder(h=thickness, d=casterHoleDiameter, center=true);
}

// Hole to be placed in the counter weight holder to accomodate the flexible bumper standoff.
module counterWeightHolderHoleForBumperStandoff() {
    thickness = counterWeightHolderThickness+0.02;

    rotate([0, 0, bumperMountAngles[0]])
        translate([0, baseDiameter/2-bumperMountOffset, -thickness/2+0.01])
            cylinder(h=thickness, d=bumperMountDiameter+counterWeightHolderBumperHoleClearance, center=true);
}




// Piece to be attached inside of the counter weight holder to keep the tungsten putty from oozing out.
// I will probably tack weld it inplace with soldering iron.
module counterWeightHolderBottom() {
    spanAngle = counterWeightHolderAngle-1.5*counterWeightHolderInsideOffsetAngle;
    difference() {
        rotate([0, 0, 90-spanAngle/2])
            rotate_extrude(angle=spanAngle)
                translate([counterWeightHolderOffsetRadius, -counterWeightHolderThickness/2-counterWeightHolderSwitchClearance])
                    square([counterWeightDiameter, counterWeightHolderThickness], center=true);
        counterWeightHolder();
    }
}




// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 1;
