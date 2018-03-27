package org.usfirst.frc.team7176.robot.subsystems;


import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team7176.robot.RobotMap;

//import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Encoder;
public class Joint1PIDSubsystem extends PIDSubsystem { // This system extends PIDSubsystem
	VictorSP motor = RobotMap.jointMotor1;
	Encoder encoder = RobotMap.jointEncoder1;

	public Joint1PIDSubsystem() {
		
		// for 188 motor with 7 count, kp = 0.015
		// for 188 motor with 3 count , kp = 0.025
		//for 188 motor with us digital 360 * 4 encoder, kp = 0.03 with encoder on output shaft
		super("Joint1", 0.02, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(2);
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
    	motor.pidWrite(output); // this is where the computed output value from the PIDController is applied to the motor
    //	System.out.println("Joint 1 PID output = " + output);
    //	motor.pidWrite(1.0);
    }


}
