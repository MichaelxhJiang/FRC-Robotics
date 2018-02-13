package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RobotAutoDriveMCmd extends Command{
	private int flag = 0;
	private int step = 0;
	private final static double SET_DISTANCE1 = 100;//100; //go forward a little bit
	private final static double SET_DISTANCE2 = 100;//200; //go straight distance to right
	private final static double SET_DISTANCE3 = 100;//400  /go straight to scale
	private final static double SET_VEL = 0.3;
	private final static double SET_TURN_VEL = 0.3;
	private final static double TURN_ANGLE1 = 90;
	private final static double TURN_ANGLE2 = -90;
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
			Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE1);
			Robot.drvByDistanceCmd.start();
			
			
		}

		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			if (step == 0){
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE1);  //turn to left
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}else if (step == 1) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE2);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
			else if (step == 2) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL, TURN_ANGLE2);  //turn to left
					Robot.turnByGyroCmd.start();
					step  = 3; // got next step
				}
			}else if (step == 3) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE3);
					Robot.drvByDistanceCmd.start();
					step = 4;
				}
			}
			//System.out.println("Step == " + step);
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			if (step == 4) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
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
