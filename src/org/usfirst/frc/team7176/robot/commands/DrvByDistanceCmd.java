/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7176.robot.commands;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.subsystems.GoStraightByGyroSubSystem;
/**
 * An example command.  You can replace me with your own command.
 */
public class DrvByDistanceCmd extends Command {
	private final static double WHEEL_D = 15.24;  //wheel diameter in cm
	private final static int CIRCLE_CNT = 1440;
	private int controlStepCnt = 0;
	private int drvEncoderCnt = 0;
	private static Encoder leftEncoder = RobotMap.leftEncoder;
	private static Encoder rightEncoder = RobotMap.rightEncoder;
	private static ADXRS450_Gyro gyroSPI = RobotMap.gyroSPI;
	
	public DrvByDistanceCmd(double drvCmd, double distance) {
		// Use requires() here to declare subsystem dependencies
		
		drvEncoderCnt = (int)(distance * CIRCLE_CNT /(Math.PI * WHEEL_D));
		leftEncoder.reset();
		rightEncoder.reset();
		gyroSPI.reset();
		controlStepCnt = 0;
		Robot.goStraightByGyroSubSystem = new GoStraightByGyroSubSystem(drvCmd);
		Robot.goStraightByGyroSubSystem.disable();
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		controlStepCnt = 0;
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
			Robot.goStraightByGyroSubSystem.setSetpoint(0);  //reset gyro = 0;
			
			Robot.goStraightByGyroSubSystem.enable();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (((Math.abs(leftEncoder.getRaw()) + Math.abs(rightEncoder.getRaw()))/2) >= drvEncoderCnt) {
			Robot.goStraightByGyroSubSystem.disable();
			Robot.goStraightByGyroSubSystem.stopMotor();
			System.out.println("go straight job done");
			controlStepCnt = 0;
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
