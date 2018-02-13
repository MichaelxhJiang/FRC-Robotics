package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.subsystems.TurnByGyroSubsystem;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class TurnByGyroCmd extends Command{
	private static Encoder leftEncoder = RobotMap.leftEncoder;
	private static Encoder rightEncoder = RobotMap.rightEncoder;
	private static ADXRS450_Gyro gyroSPI = RobotMap.gyroSPI;
	
	private static int controlStepCnt = 0;
	private static double turnAngle = 0;
	public TurnByGyroCmd(double drvCmd, double angle) {
		// Use requires() here to declare subsystem dependencies
		
		
		leftEncoder.reset();
		rightEncoder.reset();
		gyroSPI.reset();
		controlStepCnt = 0;
		Robot.turnByGyroSubsystem = new TurnByGyroSubsystem(drvCmd);
		Robot.turnByGyroSubsystem.disable();
		turnAngle = angle;
		
	}
	// Called just before this Command runs the first time
		@Override
		protected void initialize() {
			leftEncoder.reset();
			rightEncoder.reset();
			gyroSPI.reset();
		}

		// Called repeatedly when this Command is scheduled to run
		//suppose is around 20ms period
		@Override
		protected void execute() {
			controlStepCnt ++;
			//delay around 200ms to reset gyro and encoder, maybe could be reduced
			if (controlStepCnt == 10) {
				Robot.turnByGyroSubsystem.setSetpoint(turnAngle);  //
				
				Robot.turnByGyroSubsystem.enable();
			}
		}

		// Make this return true when this Command no longer needs to run execute()
		@Override
		protected boolean isFinished() {
			if ((controlStepCnt > 10) && Robot.turnByGyroSubsystem.onTarget()) {
				System.out.println("turn job done");
				controlStepCnt = 0;
				Robot.turnByGyroSubsystem.disable();
				Robot.turnByGyroSubsystem.stopMotor();
				return true;
				
			}
			return false;
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
