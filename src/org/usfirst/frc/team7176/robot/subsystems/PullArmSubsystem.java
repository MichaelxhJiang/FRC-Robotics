package org.usfirst.frc.team7176.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team7176.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
public class PullArmSubsystem extends PIDSubsystem { // This system extends PIDSubsystem

	Encoder encoder = RobotMap.hookEncoder;

	public PullArmSubsystem() {
		super("Joint1", 0.001, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(50);
		getPIDController().setContinuous(false);
		//setInputRange(-32767,32767);
		setOutputRange(-1.0,1.0);
		
	}
	
    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
    	return encoder.getRaw(); // returns the sensor value that is providing the feedback for the system
    }

    protected void usePIDOutput(double output) {
   	
    	RobotMap.hookMotor1.set(ControlMode.PercentOutput, output);
    	RobotMap.hookMotor2.set(ControlMode.PercentOutput, output);
    }


}
