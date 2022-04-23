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
// Models for external parts from Pololu, Adafruit, etc to be mounted on RugRover bot.


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

// Dimensions of the AA cells in the Pololu battery packs.
batteryDiameter = 14.6;
batteryLength = 50.4;

// Dimensions of my RugRover shield that to be placed on top of the nRF52 DK.
// How much further above the nRF52 DK should my RugRover shield be placed.
shieldZOffset = 11.7;
// Offset my RugRover shield STL model to sit right above the nRF52 DK.
shieldXOffset = -0.25;
shieldYOffset = -16.0;



// Pololu's Metal Gearmotor 20Dx43L mm 6V CB with Extended Motor Shaft
// https://www.pololu.com/category/214/6v-carbon-brush-cb-20d-gearmotors
// The face of the gearbox is at the origin.
module motor() {
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



// Pololu's Rechargeable NiMH Battery Pack: 6.0 V, 2200 mAh, 5x1 AA Cells, JR Connector
// https://www.pololu.com/product/2223
module batteryPack5() {
    translate([0, 0, batteryDiameter/2]) {
        rotate([90, 0, 0]) {
            battery(0, -2);
            battery(0, -1);
            battery(0, 0);
            battery(0, 1);
            battery(0, 2);
        }
    }
}

// Pololu's Rechargeable NiMH Battery Pack: 6.0 V, 2200 mAh, 3+2 AA Cells, JR Connector
// https://www.pololu.com/product/2224
module batteryPack3_2() {
    translate([0, 0, batteryDiameter/2]) {
        rotate([90, 0, 0]) {
            battery(0, -1);
            battery(0, 0);
            battery(0, 1);
            battery(0.85, -0.5);
            battery(0.85, 0.5);
        }
    }
}

// A single cell from a pack.
module battery(row, col) {
    translate([col*batteryDiameter, row*batteryDiameter, 0])
        cylinder(h=batteryLength, d=batteryDiameter, center=true);
}

// My RugRover shield PCB with parts installed.
module shield() {
    translate([shieldXOffset, shieldYOffset, -nRF52DK_MountingHeight-shieldZOffset])
        rotate([180, 0, 90])
            import("../hardware/shield.stl");
}
