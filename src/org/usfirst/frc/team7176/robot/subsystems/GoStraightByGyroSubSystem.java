package org.usfirst.frc.team7176.robot.subsystems;

import org.usfirst.frc.team7176.robot.RobotMap;


import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
public class GoStraightByGyroSubSystem extends PIDSubsystem{
	private static Spark leftMotor1 = RobotMap.leftMotor1;
	private static Spark leftMotor2 = RobotMap.leftMotor2;
	private static Spark rightMotor1 = RobotMap.rightMotor2;
	private static Spark rightMotor2 = RobotMap.rightMotor2;
	
	private static ADXRS450_Gyro gyroSPI = RobotMap.gyroSPI;
	
	private static double pidOutputRange = 0.5;
	
	private static double drvCmd = 0;

	public GoStraightByGyroSubSystem(double driveVel) {
		super("GoStraightByGyro", 0.09, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(0.5);
		getPIDController().setContinuous(false);
		setInputRange(-360,360);  //angle degree
		setOutputRange(-pidOutputRange,pidOutputRange); 
		drvCmd = driveVel;
		gyroSPI.reset();

	}
	
    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
    	return gyroSPI.getAngle(); // returns the sensor value that is providing the feedback for the system
    }

    protected void usePIDOutput(double output) {

    	leftMotor1.pidWrite(drvCmd + output); // this is where the computed output value from the PIDController is applied to the motor
    	leftMotor2.pidWrite(drvCmd + output);
    	rightMotor1.pidWrite(-drvCmd + output);
    	rightMotor2.pidWrite(-drvCmd + output);
    }
    
    public void stopMotor() {
    	rightMotor1.set(0);
    	rightMotor2.set(0);
    	leftMotor1.set(0); // this is where the computed output value from the PIDController is applied to the motor
    	leftMotor2.set(0);
    	
    }
    
    public void setSpeed(double speed) {
    	drvCmd = speed;
    }
    
    public void setPIDRange(double range) {
    	pidOutputRange = range;
    }
}
