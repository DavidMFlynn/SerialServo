// ************************************************************
// Robot Arm Model
// David M. Flynn 
// Created: 6/11/2018
//
// Simple 3 degree of freedom arm
// Given an object (the blue sphere) at Obj_x,Obj_y,Obj_z
// Calculate the angle of the 3 joints (BaseRot_a,J1Rot_a,J2Rot_a)
// to place the object in the center of the gripper
// Recommended: Animation FPS=30, Steps=100 or more
// ************************************************************
//
// some usefull functions
function Oscillate(Dist) = (1-abs($t*2-1))*Dist; // move from 0 to Dist and back to 0 every cycle
function XY2Deg(X,Y) = -atan2(X,Y); // convert an X,Y to an angle 0=+y,90=-x,180/-180=-y,-90=+x
function Dist(X,Y) = sqrt(pow(X,2)+pow(Y,2)); // how far to x,y
//
//     C
//     /\
//   b/  \a
// A ----- B
//     c
// given the length of the sides calculate an inside angle
function AngleA(a,b,c) = acos( (pow(b,2)+pow(c,2)-pow(a,2)) / (2 * b * c));
function AngleB(a,b,c) = acos( (pow(a,2)+pow(c,2)-pow(b,2)) / (2 * a * c));
function AngleC(a,b,c) = acos( (pow(a,2)+pow(b,2)-pow(c,2)) / (2 * a * b));

//$t=0.0; // comment out to enable animation

// Position of object
Obj_x=Oscillate(200)-50;
Obj_y=Oscillate(100)+150;
Obj_z=Oscillate(100)+100; // from base plate or table surface

// The object.
translate([Obj_x,Obj_y,Obj_z]) color("Blue") sphere(d=20);

// Size of table base, just for show
	Table_h=3;
	Table_xy=200;

// Size of the joint, "A" has the servo and encoder, "B" is the turning part
	JointA_d=70;
	JointA_h=40;
	JointA_fn=14;
	JointB_d=90;
	JointB_h=38;
	JointB_fn=10;

// Length of the arm sections
	RodAB_l=240; // length from J1 to J2
	RodBC_l=230; // length from J2 center line to moddle of gripper
	
Tube_d=19;
BaseToJ1_z=JointB_d/2;
BaseToJ1_x=-JointA_h/2;
J1_CL_z=JointA_h+JointB_h+BaseToJ1_z;

module DrawJoint(Rot_a=0){
	// Base
	cylinder(d=JointA_d,h=JointA_h,$fn=JointA_fn);
	
	// the turning part
	translate([0,0,JointA_h]) rotate([0,0,Rot_a]){
		cylinder(d=JointB_d,h=JointB_h,$fn=JointB_fn);
		
		// Indicator: Which way am I pointing?
		translate([0,JointB_d/2-10,JointB_h]) rotate([0,0,-30]) color("Tan") cylinder(d=10,h=1,$fn=3);
	}
} // DrawJoint

module DrawBase(Rot_a=0){
	//translate([0,0,Table_h/2]) cylinder(d=Table_xy,h=Table_h,$fn=8,center=true);
	translate([0,0,-70]) cylinder(d1=120,d2=80,h=70,$fn=12);
	DrawJoint(Rot_a=Rot_a);
} // DrawBase

// do the math
BaseRot_a=XY2Deg(Obj_x,Obj_y);
Adj_z=Obj_z-J1_CL_z; // Z relitive to J1 rotation center
d=Dist(Obj_x,Obj_y);  // distance in XY plane
hypJ2=Dist(d,Adj_z); // end to J1 distance
Targ_a=XY2Deg(d,Adj_z);
J2Rot_a=180-AngleC(RodAB_l,RodBC_l,hypJ2);
J1Ang_B=AngleB(RodAB_l,RodBC_l,hypJ2);
J1Rot_a=Targ_a+J1Ang_B;

// Show the angles to the user.
echo(BaseRot=BaseRot_a);
echo(J1Rot_a=J1Rot_a);
echo(J2Rot_a=J2Rot_a);

// Draw the arm
module Draw3DoFArm(BaseRot_a=BaseRot_a,J1Rot_a=J1Rot_a,J2Rot_a=J2Rot_a){
	DrawBase(Rot_a=BaseRot_a);

	translate([0,0,JointA_h+JointB_h]) rotate([0,0,BaseRot_a]) translate([BaseToJ1_x,0,BaseToJ1_z])
		rotate([0,90,0]) rotate([0,0,90]){
			
		// the joint attached to the rotating base	
		DrawJoint(Rot_a=J1Rot_a);
			
		rotate([0,0,J1Rot_a]) translate([0,0,JointA_h+JointB_h/2]) {
			// rod connection J1 and J2
			rotate([-90,0,0]) color("Silver") cylinder(d=Tube_d,h=RodAB_l);
			translate([0,RodAB_l,JointA_h/2])rotate([0,180,0]){
				// mid-arm joint
				DrawJoint(Rot_a=J2Rot_a);
				// rod from J2 to middle of the gripper
				translate([0,0,JointA_h+JointB_h/2])rotate([0,0,J2Rot_a]){
					rotate([-90,0,0]) color("Silver") cylinder(d=Tube_d,h=RodBC_l);
}}}}}

//Draw3DoFArm();
//Draw3DoFArm(0,0,0);

// Lo-Rez Arm parts
module LoRezBase(){
	cylinder(d1=120,d2=80,h=70,$fn=12);
	translate([0,0,70]) cylinder(d=JointA_d,h=JointA_h,$fn=JointA_fn);
} // LoRezBase

//LoRezBase(); // height is 70+JointA_h

module LoRezJoint(){
	cylinder(d=JointB_d,h=JointB_h,$fn=JointB_fn);
		
		// Indicator: Which way am I pointing?
		translate([0,JointB_d/2-11,JointB_h]) rotate([0,0,-30]) color("Tan") cylinder(d=10,h=1,$fn=3);
} // LoRezJoint

//LoRezJoint();







