package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RobotAutoDriveLCmd extends Command{
	private int flag = 0;	//0 - scale is on left side, 1 - scale is on right side, 
							//2 - scale is on left side and aggressive is on
	private int step = 0;
	//scale distances and speeds
	private final static double SET_DISTANCE1 = 640;//640; //got straight distance to scale
	private final static double SET_DISTANCE2 = 580;//535; //got straight distance to middle point between scale and switch
	private final static double SET_DISTANCE3 = 450;//335; //got straight distance to  other side of scale
	private final static double SET_VEL1 = 0.5;
	private final static double SET_VEL2 = 0.8;
	
	private final static double SET_TURN_VEL = 0.5;
	private final static double TURN_ANGLE1 = 90;
	private final static double TURN_ANGLE2 = 40;  //face to scale
	private final static double SET_DISTANCE4 = 0;//335; //got straight distance to  other side of scale
	private static int delayCnt = 0;
	private static int fwdCnt = 0;
	public RobotAutoDriveLCmd() {

		System.out.println("Left position  + Select" + flag);
	}
	public void setFlag(int select) {
		flag = select;
	}
	
	// Called just before this Command runs the first time
		@Override
		protected void initialize() {
			step = 0;
			// all the distance need debug and test
			if (flag == 0) { //go straight to scale if scale on the left side
				//just go straight , 
				Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE1);
				Robot.drvByDistanceCmd.start();
			}else if(flag == 1) {
				// go straight and turn right and go straight again if scale on the right side
				Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE2);
				Robot.drvByDistanceCmd.start();
			}else if(flag == 2) {
				// will do arm lift up
				
				Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE1);
				
				Robot.drvByDistanceCmd.start();
			}
			
		}

		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			if ( (flag == 0) && (step == 0)){
				//turn 45 degree right 
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL,TURN_ANGLE2);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}
			else if ( (flag == 0) && (step == 1)){
				// go straight a little bit again
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE4);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			
			
			if ((flag == 1)  && (step == 0)){
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL,TURN_ANGLE1);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}else if ((flag == 1)  && (step == 1)) {
				//turn 45 to scale
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE3);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			
			
			if (flag == 2 && step == 0) {
				if (Robot.drvByDistanceCmd.isFinished()) {

					System.out.println("Step 0");
					Robot.drvByDistanceCmd.cancel();
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL,TURN_ANGLE2);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
				
			}
			else if ( (flag == 2) && (step == 1)){
				// go straight a little bit again
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL2,SET_DISTANCE4);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			else if (flag == 2 && step == 2) {
				// arm lift up to scale
				if (Robot.drvByDistanceCmd.isFinished()) {
					System.out.println("Step 2");
					Robot.move2ScaleFromReset = new Move2ScaleFromReset();  // to scale
					Robot.move2ScaleFromReset.start();
					step  = 3; // got next step
					delayCnt = 0;
					System.out.println("Step 2 End");
				}
			}
			else if((flag == 2)&& (step == 3)) {
				System.out.println("Starting step 3");
				
				if (Robot.move2ScaleFromReset.isFinished()) {
					//move back a little back
					/*
					Robot.drvByTimeNoCtrlCmd = null;
					Robot.drvByTimeNoCtrlCmd = new DrvByTimeNoCtrlCmd(-0.4, 700);  // move back 1 second 
					Robot.drvByTimeNoCtrlCmd.start();
					step = 4;*/
					Robot.armMiddlePositionCmd = null;
					Robot.armMiddlePositionCmd = new ArmMiddlePositionCmd();  // to scale
					Robot.armMiddlePositionCmd.start();
					step  = 5; // got next step
				}
			}
			else if ( (flag == 2) && (step == 4)){
				System.out.println("Starting step 4");
				if (Robot.drvByTimeNoCtrlCmd.isFinished()) {
					Robot.drvByTimeNoCtrlCmd.cancel();
					
					Robot.armMiddlePositionCmd = null;
					Robot.armMiddlePositionCmd = new ArmMiddlePositionCmd();  // to scale
					Robot.armMiddlePositionCmd.start();
					step  = 5; // got next step
				}
			} 
			
			
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			if ((flag == 0) && (step == 2)) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					System.out.println("done");
					return true;
				}
			}else if((flag == 1) && (step == 2)) {
			
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					return true;
				}
				
			}else if ((flag == 2) && (step == 5)) {
				if (Robot.armMiddlePositionCmd.isFinished()) {
					Robot.armMiddlePositionCmd.cancel();
					System.out.println("done");
					step = 6;
					
					return true;
				}
			}
			return false;
		
		}

		// Called once after isFinished returns true
		@Override
		protected void end() {
			
			
		}

		// Called when another command which requires one or more of the same
		// subsystems is scheduled to run
		@Override
		protected void interrupted() {
		}
}
