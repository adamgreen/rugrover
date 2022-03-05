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
// The chassis for my RugRover bot.

// ************
//  Parameters
// ************
// Motor part dimensions.
//  Gearbox built in 3 pieces where piece separation is represented by 2 rings in this  model.
//  Also has 2 tapped holes for mounting. It also includes the output D-shaft and the collar
//  where the output shaft enters the gearbox.
motorGearboxDiameter = 20.0;
motorGearboxLength = 17.3;
motorGearboxRingOffset = 3.0;
motorGearboxRingWidth = 0.1;
motorGearboxRingDepth = 0.1;
motorGearboxHoleOuter = 17.08;
motorGearboxHoleInner = 12.86;
motorGearboxHoleDiameter = 2.5;     //(motorGearboxHoleOuter - motorGearboxHoleInner)/2;
motorGearboxHoleOffsetRadius = 7.5; //(motorGearboxHoleOuter + motorGearboxHoleInner)/2/2;
motorGearboxHoleDepth = 2.0;
motorGearboxCollarDiameter = 7.0;
motorGearboxCollarDepth = 3.0;
motorGearboxShaftDiameter = 4.0;
motorGearboxShaftIndentDiameter = 3.5;
motorGearboxShaftLength = 14.5;
//  The electric motor attached to the back of the gearbox. Includes shaft sticking out the back
//  along with the electrical connections.
motorDiameter = motorGearboxDiameter;
motorFlatWidth = 15.4;
motorLength = 25.0;
motorCollarDiameter = 6.05;
motorCollarDepth = 1.9-0.7;
motorShaftDiameter = 2.0;
motorShaftLength = 6.0;

// Wheel dimensions. 
wheelDiameter = 80.65;
wheelThickness = 10.0;

// Caster dimensions. 
//  The wheel itself.
casterDiameter = 29.85;
casterInnerDiameter = 22.40;
casterThickness = 13.2;
casterInnerThickness = casterThickness - 2 * 5.25;
//  The plate which has the thrust bearing and attaches to the bot body.
casterPlateWidth = 38.2;
casterPlateHeight = 31.95;
casterPlateThickness = 1.3;
//  Placement of 4 holes used to mount plate to bot body.
casterHoleDiameter = 4.1;
casterHoleXSpacing = (33.65 + 25.2) / 2;
casterHoleYSpacing = (18.88 + 27.15) / 2;
//  Thrust bearing between plate and arm that holds the wheel.
casterBearingTopDiameter = 28.42;
casterBearingBottomDiameter = 23.0;
casterBearingHeight = 4.1 - casterPlateThickness;
//  Arm that holds the wheel.
casterArmTopDiameter = 25.55;
casterArmBottomRadius = 10.0 / 2;
casterArmBottomWidth = 18.0;
casterArmTopZOffset = 1.0;
casterArmXOffset = 10.0;
casterArmZOffset = 25.0;
casterArmCutoutTopThickness = 1.2;
casterArmCutoutTopWidth1 = 17.4;
casterArmCutoutTopWidth2 = 22.0;
casterArmCutoutBottomWidth = 15.0;

// Dimensions of the VL6180X based cliff detector.
cliffDetectorWidth = 12.7;
cliffDetectorHeight = 17.8;
cliffDetectorMountingHoleDiameter = 2.2;
cliffDetectorMountingHoleXOffset = cliffDetectorWidth - 2.5;
cliffDetectorMountingHoleYOffset = cliffDetectorHeight/2 - 2.5;

// Rough dimensions of bumper switches.
bumperSwitchWidth = 19.8;
bumperSwitchHeight = 10.6;
bumperSwitchThickness = 6.2;
bumperSwitchHoleDiameter = 2.5;
bumperSwitchHoleXOffset = 9.5/2;
bumperSwitchHoleYOffset = -bumperSwitchHeight/2 + 2.9;

// Dimensions of the Nordic nRF52DK PCB.
nRF52DK_Length = 101.6;
nRF52DK_Width = 63.5;
nRF52DK_Thickness = 1.6;
// Location of the mounting holes in the nRF52DK PCB. 
nRF52DK_MountingHoles=[[15.24, 50.8+5, 0], [13.97, 2.54+5, 0], [66, 7.62+5, 0], [66, 35.56+5, 0], [96.52, 2.54+5, 0]];


// Tight clearance.
clearanceTight = 0.2;
// Loose clearance. 
clearanceLoose = 0.3;



// The diameter of the circular robot base.
baseDiameter = 160.0;
// The thickness of the circular robot base.
baseThickness = 2.5;
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
// How far from the robot base should the nRF52DK PCB be mounted. 
nRF52DK_MountingHeight = 5.0;
// Diameter of the hole used to attach standoffs for the nRF52DK PCB. 
nRF52DK_MountingHoleDiameter = 3.0;
// Diameter of hole in middle of base for pulling cables through. 
cableHoleDiameter = 25.0;




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
}
// Draw ground plane to see if all wheels fall on it.
*translate([0, 0, wheelZOffset+wheelDiameter/2-0.001]) 
    cube([baseDiameter*2, baseDiameter*2, 0.001], center=true);




// Circular robot base with mounting for motors, caster, sensors, etc.
module base() {
    difference() {
        union() {
            cylinder(h=baseThickness, d=baseDiameter);
            motorMounts();
            casterMount();
            placeCliffDetectors() 
                mountForCliffDetectors();
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
        cube([width, length, casterMountHeight], center=true);
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
    for (i=bumperMountAngles)
        rotate([0, 0, i])
            translate([0, baseDiameter/2-bumperMountOffset, -0.001])
                cylinder(h=baseThickness+0.002, d=bumperMountDiameter);
    // Need to countersink through the thick caster mount.
    translate([0, baseDiameter/2-bumperMountOffset, baseThickness-0.001])
        cylinder(h=casterMountHeight+0.002, d=bumperMountCountersinkDiameter);
}




// Pololu's Metal Gearmotor 20Dx43L mm 6V CB with Extended Motor Shaft
// https://www.pololu.com/category/214/6v-carbon-brush-cb-20d-gearmotors
module motor() {
    // UNDONE: Add some comments about how the motor parts are to be oriented.
    translate([-motorGearboxLength, 0, 0]) {
        motorGearbox();
        motorMotor();
    }
}

module motorGearbox() {
    translate([motorGearboxLength/2, 0, 0]) {
        rotate([0, 90, 0]) {
            difference() {
                cylinder(h=motorGearboxLength, d=motorGearboxDiameter, center=true);
                motorGearboxRing(1.0);
                motorGearboxRing(-1.0);
                motorGearboxMountingHoles(1.0);
                motorGearboxMountingHoles(-1.0);
            }
            motorGearboxCollar();
            motorGearboxShaft();
        }
    }
}

module motorGearboxRing(offset) {
    offsetFromCenter = offset* (motorGearboxLength/2 - motorGearboxRingOffset);
    translate([0, 0, offsetFromCenter]) {
        difference() {
            cylinder(h=motorGearboxRingWidth, d=motorGearboxDiameter+0.002, center=true);
            cylinder(h=motorGearboxRingWidth+0.002, d=motorGearboxDiameter-motorGearboxRingDepth, center=true);
        }
    }
}

module motorGearboxMountingHoles(offset) {
    translate([0, offset*motorGearboxHoleOffsetRadius, motorGearboxLength/2])
        cylinder(h=motorGearboxHoleDepth*2, d=motorGearboxHoleDiameter, center=true);
}

module motorGearboxCollar() {
    translate([0, 0, motorGearboxLength/2])
        cylinder(h=motorGearboxCollarDepth*2, d=motorGearboxCollarDiameter, center=true);
}

module motorGearboxShaft() {
    translate([0, 0, motorGearboxLength/2 + motorGearboxCollarDepth]) {
        difference() {
            cylinder(h=motorGearboxShaftLength, d=motorGearboxShaftDiameter);
            translate([0, motorGearboxShaftDiameter/2+motorGearboxShaftIndentDiameter/2, motorGearboxShaftLength/2])
                cube([motorGearboxShaftDiameter, motorGearboxShaftDiameter, motorGearboxShaftLength+0.02], center=true);
        }
    }
}

module motorMotor() {
    translate([-motorLength/2, 0, 0]) {
        rotate([0, -90, 0]) {
            difference() {
                cylinder(h=motorLength, d=motorDiameter, center=true);
                motorFlat(1.0);
                motorFlat(-1.0);
            }
            motorBackCollar();
            motorBackShaft();
        }
    }
}

module motorFlat(offset) {
    translate([offset*(motorDiameter/2+motorFlatWidth/2), 0, 0])
        cube([motorDiameter, motorDiameter+0.02, motorLength+0.02], center=true);
}

module motorBackCollar() {
    translate([0, 0, motorLength/2])
        cylinder(h=motorCollarDepth*2, d=motorCollarDiameter, center=true);
}

module motorBackShaft() {
    translate([0, 0, motorLength/2+motorCollarDepth])
        cylinder(h=motorShaftLength, d=motorShaftDiameter);
}



// Rough model of Pololu Mini-hub Wheel w/ Inserts for 3mm and 4mm shafts.
// https://www.pololu.com/product/3690
module wheel() {
    rotate([0, 90, 0])
        cylinder(h=wheelThickness, d=wheelDiameter, center=true);
}



// Rough model of Adafruit's Supporting Swivel Caster Wheel 1.3" diamter.
// https://www.adafruit.com/product/2942
module caster() {
    rotate([0, 0, -90]) {
        translate([0, 0, -casterPlateThickness/2]) {
            translate([-casterArmXOffset, 0, -casterBearingHeight-casterPlateThickness/2-casterArmZOffset])
                casterWheel();
            casterPlate();
            casterBearing();
            casterArm();
        }
    }
}

module casterWheel() {
    rotate([0, 0, 90]) {
        casterWheelInner();
        casterWheelOuter();
    }
}

module casterWheelInner() {
    rotate([0, 90, 0])
        cylinder(h=casterInnerThickness, d=casterInnerDiameter, center=true);
}

module casterWheelOuter() {
    rotate([0, 90, 0])
        difference() {
            cylinder(h=casterThickness, d=casterDiameter, center=true);
            cylinder(h=casterThickness+0.02, d=casterInnerDiameter, center=true);
        }
}

module casterPlate() {
    difference() {
        cube([casterPlateWidth, casterPlateHeight, casterPlateThickness], center=true);
        for (x=[-1:2:1])
            for (y=[-1:2:1])
                casterPlateHole(x, y);
    }
}

module casterPlateHole(x, y) {
    xOff = casterHoleXSpacing/2;
    yOff = casterHoleYSpacing/2;
    translate([x * xOff, y*yOff, 0])
        cylinder(h=casterPlateThickness+0.02, d=casterHoleDiameter, center=true);
}

module casterBearing() {
    translate([0, 0, -casterBearingHeight/2-casterPlateThickness/2])
        cylinder(h=casterBearingHeight, d1=casterBearingBottomDiameter, d2=casterBearingTopDiameter, center=true);
}

module casterArm() {
    translate([0, 0, -casterBearingHeight-casterPlateThickness/2])
        difference() {
            casterArmOutside();
            casterArmCutout();
        }
}

module casterArmOutside() {
    hull() {
        cylinder(h=0.1, d=casterArmTopDiameter, center=true);
        translate([-casterArmXOffset, 0, -casterArmZOffset])
            rotate([90, 0, 0])
                cylinder(h=casterArmBottomWidth, r=casterArmBottomRadius, center=true);
        translate([-casterArmXOffset-casterArmBottomRadius, 0, -casterArmTopZOffset])
            cube([0.1, casterArmBottomWidth, 0.1], center=true);
    }
}

module casterArmCutout() {
    hull() {
        translate([-casterArmXOffset-casterArmBottomRadius-0.1, 0, -casterArmTopZOffset-casterArmCutoutTopThickness])
            cube([0.1, casterArmCutoutTopWidth1, 0.1], center=true);
        translate([-casterArmXOffset-casterArmBottomRadius-0.1, 0, -casterArmZOffset-casterArmBottomRadius-0.1])
            cube([0.1, casterArmCutoutBottomWidth, 0.1], center=true);
        translate([casterArmXOffset+casterArmBottomRadius+0.1, 0, -casterArmTopZOffset-casterArmCutoutTopThickness])
            cube([0.1, casterArmCutoutTopWidth2, 0.1], center=true);
        translate([casterArmXOffset+casterArmBottomRadius+0.1, 0, -casterArmZOffset-casterArmBottomRadius-0.1])
            cube([0.1, casterArmCutoutBottomWidth, 0.1], center=true);
    }
}



// Pololu's VL6180X Time-of-Flight Distance Sensor Carrier
// https://www.pololu.com/product/2489
// Will be aligned at the origin in the positive portion of the x, y, and z axis. 
// SMDs components are facing upwards.
module cliffDetector() {
    import("vl6180x-carrier.stl");
}



// A rough esitmate of the bumper switch, centered at the origin.
// Pololu Snap-Action Switch with 15.6mm Bump Lever: 3-Pin, SPDT, 5A
// https://www.pololu.com/product/1405
module bumperSwitch() {
    bumperBodyWithMountingHoles();
    bumperLever();
}

// The body of the bumper switch with mounting holes but no lever.
module bumperBodyWithMountingHoles() {
    difference() {
        cube([bumperSwitchWidth, bumperSwitchHeight, bumperSwitchThickness], center=true);
        for(i = [-1:2:1]) {
            translate([i*bumperSwitchHoleXOffset, bumperSwitchHoleYOffset, 0])
                cylinder(h=bumperSwitchThickness+0.02, d=bumperSwitchHoleDiameter, center=true);
        }
    }
}

// The lever on the bumper switch. 
module bumperLever() {
    translate([-bumperSwitchWidth/2+3.0, bumperSwitchHeight/2, 0])
        linear_extrude(4.0, center=true)
            polygon([[0, 0], [0, 1.1], [12.2, 1.1], [15.45, 1.1+1.65], [18.8, 1.1],
                     [18.8, 0.8], [15.45, 1.1+1.65-0.3], [12.2, 0.8], [0.3, 0.8], [0.3, 0]]);
}



// The nRF52DK PCB 3D. 
// Sitting in positive X, Y, Z quadrant with coin holder sticking down through bottom.
module nRF52DK_PCB() {
    translate([0, 0, nRF52DK_Thickness])
        import("nRF52-DK.stl");
}
    



// Sets the smoothness of arcs/circles.
$fs = 0.1;
$fa = 1;
