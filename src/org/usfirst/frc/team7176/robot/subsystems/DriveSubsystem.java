/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7176.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import org.usfirst.frc.team7176.robot.*;



/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class DriveSubsystem extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	Spark left1 = RobotMap.leftMotor1;
	Spark left2 = RobotMap.leftMotor2;
	Spark right1 = RobotMap.rightMotor1;
	Spark right2 = RobotMap.rightMotor2;

	
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public void driveRobot(double leftPower, double rightPower) {
	
		left1.set(leftPower * Robot.gear);
		left2.set(leftPower * Robot.gear);
		right1.set(rightPower * Robot.gear);
		right2.set(rightPower * Robot.gear);

		
	}
}
