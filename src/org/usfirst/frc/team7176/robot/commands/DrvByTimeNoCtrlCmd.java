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
import org.usfirst.frc.team7176.robot.subsystems.GoStraightByGyroSubSystem;
/**
 * An example command.  You can replace me with your own command.
 */
public class DrvByTimeNoCtrlCmd extends Command {
	private int controlStepCnt = 0;

	private static double time = 0;
			
	private static double drvCmd = 0;
	
	public DrvByTimeNoCtrlCmd(double setPower, double _time) {
		// Use requires() here to declare subsystem dependencies
		controlStepCnt = 0;
		drvCmd = setPower;
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("time: " + time + " power: " + drvCmd);
		RobotMap.leftMotor1.set(drvCmd);
		RobotMap.leftMotor2.set(drvCmd);
		RobotMap.rightMotor1.set(-drvCmd);
		RobotMap.rightMotor2.set(-drvCmd);
	}

	// Called repeatedly when this Command is scheduled to run
	//suppose is around 20ms period
	@Override
	protected void execute() {
		controlStepCnt ++;
		if (controlStepCnt >= time/20) {
			RobotMap.leftMotor1.set(0);
			RobotMap.leftMotor2.set(0);
			RobotMap.rightMotor1.set(0);
			RobotMap.rightMotor2.set(0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (controlStepCnt >= time/20) {
			RobotMap.leftMotor1.set(0);
			RobotMap.leftMotor2.set(0);
			RobotMap.rightMotor1.set(0);
			RobotMap.rightMotor2.set(0);
			System.out.println(" move by TIME job done");
			return true;
			
		} else {
			
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
