// ******************************************************
// Case and Mount for Serial Servo Rev B PCB
// by Dave Flynn
// Created:8/13/2018
// Revision: 1.0.1 8/19/2018
// Units: mm
// ******************************************************
// History:
// 1.0.1 8/19/2018 Moved mounting locations.
// 1.0.0 8/13/2018 First Code
// ******************************************************
// Notes:
// ******************************************************
// for STL output
// SSCaseBack();
// TubeMount();
// ******************************************************

include<CommonStuffSAEmm.scad>

$fn=90;
Overlap=0.05;
IDXtra=0.2;

PCB_x=34.4;
PCB_y=62.2;
PCB_z=1.8; // board only
PCB_Back_Parts_z=5;
PCB_Front_Parts_z=13.5;

CaseWall=2;
RC_BoltSpace=42;
Tube_OD=25.4;
RC_Spacer_h=4;

module RoundRect(X=60,Y=40,Z=10,R=5){
	hull(){
		translate([-X/2+R,-Y/2+R,0]) cylinder(r=R,h=Z);
		translate([X/2-R,-Y/2+R,0]) cylinder(r=R,h=Z);
		translate([-X/2+R,Y/2-R,0]) cylinder(r=R,h=Z);
		translate([X/2-R,Y/2-R,0]) cylinder(r=R,h=Z);
		
	} // hull
	
} // RoundRect

module SSCaseBack(){
	PCB_Clip_y=14;
	BoltBoss_yt=19;
	BoltBoss_yb=10;
	
	Case_h=PCB_Back_Parts_z+PCB_z+PCB_Front_Parts_z+CaseWall;
	
	module BoltBoss(XTra=1){
		difference(){
			hull(){
				translate([2.5+XTra,-3.5,0]) cube([Overlap,7,Case_h]);
				translate([0,0,Case_h-8]) cylinder(d=7,h=8);
			} // hull
			translate([0,0,Case_h])  Bolt4Hole(depth=9);
		} // diff
	}// BoltBoss
	
	difference(){
		RoundRect(X=PCB_x+CaseWall*2,Y=PCB_y+CaseWall*2,Z=Case_h,R=1);
		
		translate([0,0,CaseWall]) RoundRect(X=PCB_x,Y=PCB_y,Z=PCB_Back_Parts_z+PCB_z+PCB_Front_Parts_z+2+Overlap,R=0.1);
		
		
	} // diff
	// Bolts
	translate([-RC_BoltSpace/2,PCB_y/2-BoltBoss_yt,0])  BoltBoss(XTra=1);
	translate([RC_BoltSpace/2,PCB_y/2-BoltBoss_yt,0]) rotate([0,0,180])  BoltBoss(XTra=1);
	translate([RC_BoltSpace/2,-PCB_y/2+BoltBoss_yb,0]) rotate([0,0,180]) BoltBoss(XTra=1);
	translate([-RC_BoltSpace/2,-PCB_y/2+BoltBoss_yb,0])  BoltBoss(XTra=1);
	
	// bottom stops
	translate([-PCB_x/2,-PCB_y/2+PCB_Clip_y,Overlap]) cylinder(r=CaseWall,h=CaseWall+PCB_Back_Parts_z);
	translate([-PCB_x/2,PCB_y/2-PCB_Clip_y,Overlap]) cylinder(r=CaseWall,h=CaseWall+PCB_Back_Parts_z);
	translate([PCB_x/2,-PCB_y/2+PCB_Clip_y,Overlap]) cylinder(r=CaseWall,h=CaseWall+PCB_Back_Parts_z);
	translate([PCB_x/2,PCB_y/2-PCB_Clip_y,Overlap]) cylinder(r=CaseWall,h=CaseWall+PCB_Back_Parts_z);
	
	/*
	translate([0,0,CaseWall+PCB_Back_Parts_z+PCB_z+IDXtra]){
	translate([-PCB_x/2,-PCB_y/2+PCB_Clip_y,Overlap]) cylinder(r1=CaseWall,r2=0.5,h=CaseWall+PCB_Back_Parts_z);
	translate([-PCB_x/2,PCB_y/2-PCB_Clip_y,Overlap]) cylinder(r1=CaseWall,r2=0.5,h=CaseWall+PCB_Back_Parts_z);
	translate([PCB_x/2,-PCB_y/2+PCB_Clip_y,Overlap]) cylinder(r1=CaseWall,r2=0.5,h=CaseWall+PCB_Back_Parts_z);
	translate([PCB_x/2,PCB_y/2-PCB_Clip_y,Overlap]) cylinder(r1=CaseWall,r2=0.5,h=CaseWall+PCB_Back_Parts_z);
	}
	/**/
} // SSCaseBack

//SSCaseBack();



module TubeMount(){
	TM_h=8;
	
	difference(){
		union(){
			cylinder(d=Tube_OD+4,h=TM_h);
			translate([-(RC_BoltSpace+8)/2,Tube_OD/2-6,0]) cube([RC_BoltSpace+8,6,TM_h]);
			
			/*
			// Spacers
			translate([RC_BoltSpace/2,Tube_OD/2+RC_Spacer_h,TM_h/2])rotate([90,0,0])hull()
			{
				cylinder(d=7,h=RC_Spacer_h+Overlap);
				translate([3,-TM_h/2,0])cube([1,TM_h/2+3.5,7]);
				translate([-3.5,-TM_h/2,0])cube([TM_h/2+3.5,1,7]);
			}
			mirror([1,0,0])
			translate([RC_BoltSpace/2,Tube_OD/2+RC_Spacer_h,TM_h/2])rotate([90,0,0])hull()
			{
				cylinder(d=7,h=RC_Spacer_h+Overlap);
				translate([3,-TM_h/2,0])cube([1,TM_h/2+3.5,7]);
				translate([-3.5,-TM_h/2,0])cube([TM_h/2+3.5,1,7]);
			}
			*/
		} // union
		
		// Tube cut
		translate([-10,0,-Overlap]) cube([20,20,TM_h+Overlap*2]);
		
		// Bolts
		translate([RC_BoltSpace/2,Tube_OD/2+RC_Spacer_h,TM_h/2])rotate([-90,0,0])  Bolt4ClearHole();
		translate([-RC_BoltSpace/2,Tube_OD/2+RC_Spacer_h,TM_h/2])rotate([-90,0,0])  Bolt4ClearHole();
		
		// Tube
		translate([0,0,-Overlap]) cylinder(d=Tube_OD+IDXtra,h=TM_h+Overlap*2);
	} // diff
	
} // TubeMount

//TubeMount();


