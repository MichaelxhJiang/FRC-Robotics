package org.usfirst.frc.team7176.robot.commands;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

/**
 * An example command.  You can replace me with your own command.
 */
public class PullArmCmd extends Command {
	
	private int controlStepCnt = 0;
	//private int drvEncoderCnt = 0;
	private static Encoder pullArmEncoder = RobotMap.hookEncoder;

	private static double time = 0;
	private int stepNum = 0;
	private int setPos = 0;	
	private int stepValue = 0;
	private int stepCnt = 0;
	private int targetPos = 0;
	private static double drvCmd = 0;
	private int curPos = 0;
	
	public PullArmCmd(int curPosition, int setPosition, int time) {
//		requires(Robot.pullArmSubsystem);
		setPos = setPosition;
		stepNum = time/20;
		curPos = curPosition;
		stepValue = (int)(((double)setPos - curPos)/stepNum);
		stepCnt = 0;
		targetPos = (int)(curPos +  stepValue * stepCnt);
		
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
//		Robot.pullArmSubsystem.setSetpoint(targetPos);
//		Robot.pullArmSubsystem.enable();

	}

	// Called repeatedly when this Command is scheduled to run
	//suppose is around 20ms period
	@Override
	protected void execute() {
		if (stepCnt != stepNum) {
			stepCnt ++;	
		}
		
		if (stepCnt < stepNum) {
			targetPos = (int) (curPos +  stepValue * stepCnt);	
	//		Robot.pullArmSubsystem.setSetpoint(targetPos);
		}else if(stepCnt >= stepNum) {
			targetPos = setPos;
//			Robot.pullArmSubsystem.setSetpoint(targetPos);
		}
		System.out.println("pull arm Step Cnt = " + stepCnt + "  target = " + targetPos);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if ((stepCnt == stepNum) && Robot.joint1Subsystem.onTarget()) {
			
			System.out.println("Pull arm on position");
			return true;
		}else {
		
			return false;
		}
	}

	// Called once after isFinished returns tru
	@Override
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}

