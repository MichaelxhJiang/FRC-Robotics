package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

public class HookLiftCmd2 extends Command {
	TalonSRX hookMotor1 = RobotMap.hookMotor1;
	TalonSRX hookMotor2 = RobotMap.hookMotor2;
	
	int timer = 0;
	
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//hookMotor1.configPeakCurrentDuration(9000, 10000);
		//hookMotor1.configPeakCurrentLimit(40, 10000);
		//hookMotor2.configPeakCurrentDuration(9000, 10000);
		//hookMotor2.configPeakCurrentLimit(40, 10000);
		hookMotor1.enableCurrentLimit(false);
		hookMotor2.enableCurrentLimit(false);
		hookMotor1.set(ControlMode.PercentOutput, -1.0);
		hookMotor2.set(ControlMode.PercentOutput, -1.0);
	}
 
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println(timer);
		timer++;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return timer >= 50;		//1 second
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		hookMotor1.set(ControlMode.PercentOutput, 0);
		hookMotor2.set(ControlMode.PercentOutput, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}