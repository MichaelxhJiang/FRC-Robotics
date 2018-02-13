/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7176.robot;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drive train subsystem.
	 public static  Spark leftMotor1 = new Spark(0);;
	 public static Spark leftMotor2 = new Spark(1);;
	 public static Spark rightMotor1  = new Spark(2);
	 public static Spark rightMotor2 = new Spark(3);

	 public static VictorSP jointMotor1 = new VictorSP(4);
	 public static VictorSP jointMotor2 = new VictorSP(5);
	 public static VictorSP jointMotor3 = new VictorSP(6);
	 
	 public static Encoder leftEncoder = new Encoder(0,1,false,EncodingType.k4X);
	 public static Encoder rightEncoder = new Encoder(2,3,false,EncodingType.k4X);
	 
	 public static Encoder jointEncoder1 = new Encoder(4, 5, false, EncodingType.k4X);;
	 public static Encoder jointEncoder2 = new Encoder(6, 7, false, EncodingType.k4X);;
	 public static Encoder jointEncoder3 = new Encoder(8, 9, false, EncodingType.k4X);;
	 
	 	 
	 public static ADXRS450_Gyro gyroSPI = new ADXRS450_Gyro();  
	 
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	 
	 
	 public static void init() {
		
		 
	  }
}
