package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RobotAutoDriveMCmd extends Command{
	//Middle Position will only attempt switch
	private int flag;	//1 - switch is Left, 2 - switch is Right
	private int step = 0;
	private final static double SET_DISTANCE1 = 100;//100; //go forward a little bit
	private final static double SET_DISTANCE2 = 200;//200; //go straight distance to right
	private final static double SET_DISTANCE3 = 500;//400  /go straight to scale
	
	//switch distances
	private final static double SWITCH_DISTANCE1 = 20;
	private final static double SWITCH_DISTANCE_RIGHT = 125;	//155
	private final static double SWITCH_DISTANCE_RIGHT2 = 200;
	private final static double SWITCH_DISTANCE_LEFT = 164;	//164
	private final static double SWITCH_DISTANCE_LEFT2 = 240;
	private final static double SWITCH_DISTANCE_TIME = 1000;   //unit is ms
	
	private final static double SET_VEL = 0.80;
	private final static double SET_VEL2 = 0.4;
	private final static double SET_VEL3 = 0.45;
	private final static double SET_TURN_VEL = 0.45;
	private final static double TURN_ANGLE1 = 90;		//right turn
	private final static double TURN_ANGLE2 = -90;	//left turn
	private final static double TURN_ANGLE3 = 36.5;
	private final static double TURN_ANGLE4 = -36.5;
	private final static double TURN_ANGLE5 = 45.5;	//35.5
	private final static double TURN_ANGLE6 = -45.5;	//35.5
	
	public RobotAutoDriveMCmd() {
		System.out.println("Middle Position  + Select" + flag);
	}
	public void setFlag(int select) {
		flag = select;
	}
	// Called just before this Command runs the first time
		@Override
		protected void initialize() {
			step = 0;
			// all the distance need debug and test
			// now we only fix to go straight + turn right + go straight to cross the line 
			Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL, SWITCH_DISTANCE1);
			Robot.drvByDistanceCmd.start();
			
		}

		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			if (flag == 1 && step == 0){
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE6);  //turn to left
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}else if (flag == 1 && step == 1) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL, SWITCH_DISTANCE_LEFT2);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			else if (flag == 1 && step == 2) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE5);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 3; // got next step
				}
			} else if (flag == 1 && step == 3) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByTimeGyroCmd = new DrvByTimeGyroCmd(SET_VEL3,SWITCH_DISTANCE_TIME);
					Robot.drvByTimeGyroCmd.start();
					step = 4;
				}
			} else if (flag == 1 && step == 4) {
				if (Robot.drvByTimeGyroCmd.isFinished()) {
					System.out.println("STEP 4 AUTO");
					Robot.drvByTimeGyroCmd.cancel();
					//drop box
					Robot.switchDropAutoCmd = new SwitchDropAutoCmd();
					Robot.switchDropAutoCmd.start();
					step = 5;
				}
			} else if (flag == 2 && step == 0){
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE3);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}else if (flag == 2 && step == 1) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL, SWITCH_DISTANCE_RIGHT2);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			else if (flag == 2 && step == 2) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE4);  //turn to left
					Robot.turnByGyroCmd.start();
					step  = 3; // got next step
				}
			}else if (flag == 2 && step == 3) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByTimeGyroCmd = new DrvByTimeGyroCmd(SET_VEL3,SWITCH_DISTANCE_TIME);
					Robot.drvByTimeGyroCmd.start();
					step = 4;
				}
			} else if (flag == 2 && step == 4) {
				//System.out.println("STEP 4");
				if (Robot.drvByTimeGyroCmd.isFinished()) {
					Robot.drvByTimeGyroCmd.cancel();
					System.out.println("STEP 4 AUTO");
					//go straight again
					Robot.switchDropAutoCmd = new SwitchDropAutoCmd();
					Robot.switchDropAutoCmd.start();
					step = 5;
				}
			} else if (step == 5) {
				if (Robot.switchDropAutoCmd.isFinished()) {
					System.out.println("STEP 5");
					Robot.move2ScaleFromReset2 = new Move2ScaleFromReset2();  // to scale
					Robot.move2ScaleFromReset2.start();
					step = 6;
				}
			} else if (step == 6) {
				if (Robot.move2ScaleFromReset2.isFinished()) {
					Robot.move2ScaleFromReset2 = null;
					Robot.armMiddlePositionCmd = new ArmMiddlePositionCmd();  // to scale
					Robot.armMiddlePositionCmd.start();
					step  = 7; // got next step
				}
			}
			//System.out.println("Step == " + step);
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			if (step == 7) {
				if (Robot.armMiddlePositionCmd.isFinished()) {
					Robot.armMiddlePositionCmd.cancel();
					System.out.println("done");
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
