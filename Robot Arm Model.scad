// Robot Arm Model

Table_h=3;
Table_xy=100;

	JointA_d=70;
	JointA_h=40;
	JointB_d=90;
	JointB_h=38;

	RodAB_l=240;
	RodBC_l=230;
	
BaseToJ1_z=40;
BaseToJ1_x=-JointA_h/2;
J1_CL_z=JointA_h+JointB_h+BaseToJ1_z;

module DrawJoint(Rot_a=0){
	cylinder(d=JointA_d,h=JointA_h,$fn=12);
	translate([0,0,JointA_h]) rotate([0,0,Rot_a]){
		cylinder(d=JointB_d,h=JointB_h,$fn=6);
		translate([0,JointB_d/2-10,JointB_h]) rotate([0,0,-30]) color("Tan") cylinder(d=10,h=1,$fn=3);
	}
} // DrawJoint

module DrawBase(Rot_a=0){
	translate([0,0,Table_h/2]) cylinder(d=Table_xy,h=Table_h,$fn=8,center=true);
	
	DrawJoint(Rot_a=Rot_a);
} // DrawBase

$t=0.0;

function XY2Deg(X,Y) = -atan2(X,Y);
function Dist(X,Y) = sqrt(pow(X,2)+pow(Y,2));

//     C
//     /\
//   b/  \a
// A ----- B
//     c
function AngleA(a,b,c) = acos( (pow(b,2)+pow(c,2)-pow(a,2)) / (2 * b * c));
function AngleB(a,b,c) = acos( (pow(a,2)+pow(c,2)-pow(b,2)) / (2 * a * c));
function AngleC(a,b,c) = acos( (pow(a,2)+pow(b,2)-pow(c,2)) / (2 * a * b));

Obj_x=100;
Obj_y=-125;
Obj_z=550; // from base plate or table surface
Adj_z=Obj_z-J1_CL_z; // Z relitive to J1 rotation center

translate([Obj_x,Obj_y,Obj_z]) color("Blue") sphere(d=20);

BaseRot_a=XY2Deg(Obj_x,Obj_y);
d=Dist(Obj_x,Obj_y);  // distance in XY plane
hypJ2=Dist(d,Adj_z); // end to J1 distance
Targ_a=XY2Deg(d,Adj_z);
J2Rot_a=180-AngleC(RodAB_l,RodBC_l,hypJ2);
J1Ang_B=AngleB(RodAB_l,RodBC_l,hypJ2);

//echo(J1Ang_B=J1Ang_B);
J1Rot_a=Targ_a+J1Ang_B;
echo(BaseRot=BaseRot_a);
echo(J1Rot_a=J1Rot_a);
//echo(H_Dist=d);
echo(J2Rot_a=J2Rot_a);
//BaseRot_a=$t*360;




//J1Rot_a=abs($t*2-1)*90; // -170..170
//J1Rot_a=-56; // -120..120
//J2Rot_a=(1-abs($t*2-1))*90-45;
//J2Rot_a=90; // -160..160

DrawBase(Rot_a=BaseRot_a);

translate([0,0,JointA_h+JointB_h]) rotate([0,0,BaseRot_a]) translate([BaseToJ1_x,0,BaseToJ1_z]) rotate([0,90,0]) rotate([0,0,90]){

	DrawJoint(Rot_a=J1Rot_a);
	rotate([0,0,J1Rot_a]) translate([0,0,JointA_h+JointB_h/2]) {
		rotate([-90,0,0]) color("Silver") cylinder(d=10,h=RodAB_l);
		translate([0,RodAB_l,JointA_h/2])rotate([0,180,0])
		{
			DrawJoint(Rot_a=J2Rot_a);
			translate([0,0,JointA_h+JointB_h/2])rotate([0,0,J2Rot_a]){
				rotate([-90,0,0]) color("Silver") cylinder(d=10,h=RodBC_l);
			}
		}
		
	}
	
}

















