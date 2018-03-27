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

import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * An example command.  You can replace me with your own command.
 */
public class CalibratePullForceCmd extends Command {
	private static boolean calibratingPullForce = false;
	private static int calibratingStepCnt = 0;
	
	public CalibratePullForceCmd() {
		//requires(Robot.pullArmSubsystem);
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//disable all the joint1 and pull arm system
		if (Robot.joint1Subsystem != null) {
			Robot.joint1Subsystem.disable();
			RobotMap.jointMotor1.set(0);
			System.out.println("disable joint1");
		}
		/*
		if (Robot.pullArmSubsystem != null) {
			Robot.pullArmSubsystem.disable();
			RobotMap.hookMotor1.set(ControlMode.PercentOutput, 0);
			RobotMap.hookMotor2.set(ControlMode.PercentOutput, 0);
		}*/
		calibratingStepCnt = 0;
		calibratingPullForce = true;
	}

	// Called repeatedly when this Command is scheduled to run
	//suppose is around 20ms period
	@Override
	protected void execute() {
		calibratingStepCnt ++;
		if (calibratingStepCnt == 20) {
			RobotMap.hookMotor1.set(ControlMode.PercentOutput, 0.3);
			RobotMap.hookMotor2.set(ControlMode.PercentOutput, 0.3);
			System.out.println("start pull ");
			
		}else if (calibratingStepCnt > 20) {
			if (Math.abs(RobotMap.jointEncoder1.getRaw() - Robot.SET_PICKREADY_POS) < 50) {
	//			Robot.pullArmCmd = new PullArmCmd(RobotMap.hookEncoder.getRaw(), RobotMap.hookEncoder.getRaw(), 20);
	//			Robot.pullArmCmd.start();
				Robot.joint1Subsystem.enable();
				calibratingPullForce = false;
				System.out.println("calibrating job down ");
			}
		}
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return !calibratingPullForce;
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

