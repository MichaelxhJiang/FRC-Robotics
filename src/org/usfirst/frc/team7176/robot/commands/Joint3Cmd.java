package org.usfirst.frc.team7176.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7176.robot.Robot;

public class Joint3Cmd  extends Command {
	private int curPos = 0;
	private int setPos = 0;
	private int stepCnt = 1; // based on 20ms period
	private int stepNum = 0;
	private double stepValue = 0;
	private int targetPos = 0;
	public Joint3Cmd(int currentPosition, int setPosition, int time) {
		requires(Robot.joint3Subsystem);
		curPos = currentPosition;
		setPos = setPosition;
		stepNum = time/20;
		stepValue = ((double)setPos - curPos)/stepNum;
		stepCnt = 0;
		targetPos = (int)(curPos +  stepValue * stepCnt);
		
	//	System.out.println("Set Pos = " + setPos + "  Cur Pos = "+ curPos + "  step value = " + stepValue);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.joint3Subsystem.setSetpoint(targetPos);
		Robot.joint3Subsystem.enable();

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (stepCnt != stepNum) {
			stepCnt ++;	
		}
		
		if (stepCnt < stepNum) {
			targetPos = (int) (curPos +  stepValue * stepCnt);	
			Robot.joint3Subsystem.setSetpoint(targetPos);
		}else if(stepCnt >= stepNum) {
			targetPos = setPos;
			Robot.joint3Subsystem.setSetpoint(targetPos);
		}
		//System.out.println("Step Cnt = " + stepCnt + "  target = " + targetPos);
		
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if ((stepCnt == stepNum) && Robot.joint3Subsystem.onTarget()) {
			//Robot.joint3Subsystem.disable();  //for arm maybe we could not disable the PID to keep arm at position
			System.out.println("On target");
			return true;
		}else {
		
			return false;
		}
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