package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbingCmd2 extends Command {
	TalonSRX hookMotor1 = RobotMap.hookMotor1;
	TalonSRX hookMotor2 = RobotMap.hookMotor2;
	private boolean blnRelease = false;
	int timer = 0;
	private boolean blnResetArm = false;
	private int stepCnt = 0;
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//hookMotor1.configPeakCurrentDuration(9000, 10000);
		//hookMotor1.configPeakCurrentLimit(40, 10000);
		//hookMotor2.configPeakCurrentDuration(9000, 10000);
		//hookMotor2.configPeakCurrentLimit(40, 10000);
		hookMotor1.enableCurrentLimit(false);
		hookMotor2.enableCurrentLimit(false);

		//first move joint2 forward to release hook
//		Robot.joint2Cmd = new Joint2Cmd(RobotMap.jointEncoder2.getRaw(), -1647, 1000);
//		Robot.joint2Cmd.start();
		
		hookMotor1.set(ControlMode.PercentOutput, -1.0);
		hookMotor2.set(ControlMode.PercentOutput, -1.0);
		stepCnt = 0;
		System.out.println("start climbing");
	}
 
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		if (RobotMap.hookEncoder.getRaw() < Robot.climbingEncoderPos1) {
			// already reach the highest position, stop all the motors
			hookMotor1.set(ControlMode.PercentOutput, 0);
			hookMotor2.set(ControlMode.PercentOutput, 0);
			System.out.println("Stop climbing");
		}else if((RobotMap.hookEncoder.getRaw() > Robot.climbingEncoderPos2)) {
			hookMotor1.set(ControlMode.PercentOutput, -1);
			hookMotor2.set(ControlMode.PercentOutput, -1);
			System.out.println("Climbing again");
		}

		
		
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;		
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
