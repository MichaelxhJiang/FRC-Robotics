package org.usfirst.frc.team7176.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team7176.robot.RobotMap;
import edu.wpi.first.wpilibj.Encoder;
public class Joint3PIDSubsystem extends PIDSubsystem { // This system extends PIDSubsystem
	VictorSP motor = RobotMap.jointMotor3;
	Encoder encoder = RobotMap.jointEncoder3;

	public Joint3PIDSubsystem() {
		super("Joint3", 0.01, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(1);
		getPIDController().setContinuous(false);
		setInputRange(-32767,32767);
		setOutputRange(-1.0,1.0);
	}
	
    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
    	return encoder.getRaw(); // returns the sensor value that is providing the feedback for the system
    }

    protected void usePIDOutput(double output) {
    	motor.pidWrite(output); // this is where the computed output value from the PIDController is applied to the motor
    	
    }


}
