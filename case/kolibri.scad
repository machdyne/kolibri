// Kolibri Case
// Copyright (c) 2023 Lone Dynamics Corporation

$fn = 30;

box_width = 18.5;
box_length = 44;
box_height = 12;
box_wall = 1;

mdd_box();

//translate([1,0,10.10])
//	mdd_lid();

module mdd_box() {
	
	union() {
		
		difference() {
		
			roundedcube(box_width, box_length, box_height, 2.5);
		
			// board cutout
			translate([box_wall,box_wall+2,box_wall]) {
				roundedcube(box_width-(box_wall*2),
					box_length-(box_wall*2)-4,
					box_height, 2.5);
			}
		
			// USB-A
			translate([box_width/2-12.1/2,38,box_wall+2.5])
				cube([12.1, 6, 5]);
		
			// PMOD
			translate([box_width/2-15.6/2,0,box_wall+4])
				cube([15.6, 6, 15]);
		
			// sliding LID cutout
			translate([0.75,-1,12-2]) {
				roundedcube(17,
					44,
					1, 2.5);
			}

		}
	
		// pin catch
		translate([1,1,0])
			cube([16.5,8.5-1,5]);
	
	}
	
}

module mdd_lid() {

	difference () {
		union() {
			
			color([0.3,0.3,0.2])
				roundedcube(16.5,
					43,
					0.75, 2.5);
	
			color([0.6,0.6,0.2])
				translate([1,1,0.5])
				roundedcube(14.5,
					39,
					1.25, 2.5);

		}
	
		rotate([0,0,90])
			translate([box_length/2,-box_width/2-0.5,1.5])
				linear_extrude(1)
					text("K O L I B R I", size=4, halign="center",
						font="Ubuntu:style=Bold");

	}
	
}

// https://gist.github.com/tinkerology/ae257c5340a33ee2f149ff3ae97d9826
module roundedcube(xx, yy, height, radius)
{
    translate([0,0,height/2])
    hull()
    {
        translate([radius,radius,0])
        cylinder(height,radius,radius,true);

        translate([xx-radius,radius,0])
        cylinder(height,radius,radius,true);

        translate([xx-radius,yy-radius,0])
        cylinder(height,radius,radius,true);

        translate([radius,yy-radius,0])
        cylinder(height,radius,radius,true);
    }
}
