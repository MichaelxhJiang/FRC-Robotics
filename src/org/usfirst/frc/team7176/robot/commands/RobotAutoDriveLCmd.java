package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class RobotAutoDriveLCmd extends Command{
	private int flag = 0;
	private int step = 0;
	private final static double SET_DISTANCE1 = 100;//635; //got straight distance to scale
	private final static double SET_DISTANCE2 = 100;//535; //got straight distance to middle point between scale and switch
	private final static double SET_DISTANCE3 = 100;//335; //got straight distance to  other side of scale
	private final static double SET_VEL = 0.3;
	private final static double SET_TURN_VEL = 0.3;
	private final static double TURN_ANGLE = 90;
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
				//just go straight , switch on the left side
				Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE1);
				Robot.drvByDistanceCmd.start();
			}else if(flag == 1) {
				// go straight and turn right and go straight again if scale on the right side
				Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE2);
				Robot.drvByDistanceCmd.start();
			}
			
		}

		// Called repeatedly when this Command is scheduled to run
		@Override
		protected void execute() {
			if ((flag == 1)  && (step == 0)){
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					Robot.turnByGyroCmd = new TurnByGyroCmd(SET_TURN_VEL,TURN_ANGLE);  //turn to right
					Robot.turnByGyroCmd.start();
					step  = 1; // got next step
				}
			}else if ((flag == 1)  && (step == 1)) {
				if (Robot.turnByGyroCmd.isFinished()) {
					Robot.turnByGyroCmd.cancel();
					//go straight again
					Robot.drvByDistanceCmd = new DrvByDistanceCmd(SET_VEL,SET_DISTANCE3);
					Robot.drvByDistanceCmd.start();
					step = 2;
				}
			}
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			if (flag == 0) {
				if (Robot.drvByDistanceCmd.isFinished()) {
					Robot.drvByDistanceCmd.cancel();
					System.out.println("done");
					return true;
				}
			}else if(flag == 1) {
				if (step == 2) {
					if (Robot.drvByDistanceCmd.isFinished()) {
						Robot.drvByDistanceCmd.cancel();
						return true;
					}
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
