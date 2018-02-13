package org.usfirst.frc.team7176.robot.subsystems;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Spark;

public class TurnByGyroSubsystem extends PIDSubsystem {
	private static Spark leftMotor1 = RobotMap.leftMotor1;
	private static Spark leftMotor2 = RobotMap.leftMotor2;
	private static Spark rightMotor1 = RobotMap.rightMotor2;
	private static Spark rightMotor2 = RobotMap.rightMotor2;
	
	private static ADXRS450_Gyro gyroSPI = RobotMap.gyroSPI;
	
	
	private static double drvCmd = 0;
	public TurnByGyroSubsystem(double driveVel) {
		super("TurnByGyro", 0.03, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(5.0);		//1 degree error
		getPIDController().setContinuous(false);
		setInputRange(-360,360);  //angle degree
		setOutputRange(-driveVel,driveVel); 
		drvCmd = driveVel;

	}
	
    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
    	return gyroSPI.getAngle(); // returns the sensor value that is providing the feedback for the system
    }

    protected void usePIDOutput(double output) {
    		leftMotor1.pidWrite(output); // this is where the computed output value from the PIDController is applied to the motor
        	leftMotor2.pidWrite(output);
        	rightMotor1.pidWrite(output);
        	rightMotor2.pidWrite(output);
      }
    
    public void stopMotor() {
    	rightMotor1.set(0);
    	rightMotor2.set(0);
    	leftMotor1.set(0); // this is where the computed output value from the PIDController is applied to the motor
    	leftMotor2.set(0);
    	
    }
 

}
