/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7176.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

import org.usfirst.frc.team7176.robot.commands.DriveCommand;
import org.usfirst.frc.team7176.robot.commands.DrvByDistanceCmd;
import org.usfirst.frc.team7176.robot.commands.Joint1Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint2Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;
import org.usfirst.frc.team7176.robot.commands.Move2ScaleFromReset;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveLCmd;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveMCmd;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveRCmd;
import org.usfirst.frc.team7176.robot.commands.TurnByGyroCmd;
import org.usfirst.frc.team7176.robot.commands.ArmResetCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosReleaseCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosForScaleCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosForExchangeCmd;
import org.usfirst.frc.team7176.robot.commands.ArmMiddlePositionCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPickReadyCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPickupCmd;
import org.usfirst.frc.team7176.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.GoStraightByGyroSubSystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint1PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint2PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint3PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.TurnByGyroSubsystem;
import org.usfirst.frc.team7176.robot.KinematicsFunction;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project
 */
public class Robot extends TimedRobot {
	private long autoStartTime = 0;
	
	public static RobotMap robotMap = new RobotMap();
	public static final DriveSubsystem drvSubsystem	= new DriveSubsystem();
	public static final Joint1PIDSubsystem joint1Subsystem = new Joint1PIDSubsystem();
	public static final Joint2PIDSubsystem joint2Subsystem = new Joint2PIDSubsystem();
	public static final Joint3PIDSubsystem joint3Subsystem = new Joint3PIDSubsystem();
	public static Joint1Cmd joint1Cmd;
	public static Joint2Cmd joint2Cmd;
	public static Joint3Cmd joint3Cmd;
	public static Move2ScaleFromReset move2ScaleFromReset;
	public static ArmResetCmd armResetCmd;
	public static ArmPickReadyCmd armPickReadyCmd;
	public static ArmPickupCmd armPickupCmd;
	public static ArmPosForExchangeCmd armPosForExchangeCmd;
	public static ArmPosForScaleCmd armPosForScaleCmd;
	public static ArmPosReleaseCmd armPosReleaseCmd;
	public static ArmMiddlePositionCmd armMiddlePositionCmd;
	
	
	public static OI m_oi = new OI();
	
	public static double j1ResetAngle = 0;
	public static double j2ResetAngle = 0;
	public static int j1ResetPos = 0;
	public static int j2ResetPos = 0;
	public static int j3ResetPos = 0;
	public static double armPosX = 0;
	public static double armPosY = 0;
	public static int j1EncoderPos = 0;
	public static int j2EncoderPos = 0;
	public static int j3EncoderPos = 0;
    public static double j1TurnAngle = 0;
    public static double j2TurnAngle = 0;
    public final static double RESET_X = -7.0;
    public final static double RESET_Y = 0.0;
    public final static double SCALE_X = -38.0;
    public final static  double SCALE_Y = 109.2;
    public final static double SWITCH_X = -60.85;
    public final static double SWITCH_Y = -8.60;
    public final static double READYPICKUP_X = -70.77;
    public final static double READYPICKUP_Y = -55.63;
    public final static  double MIDDLE_X = -11;
    public final static double MIDDLE_Y = 1.95;
    public final static  double PICKUP_X = READYPICKUP_X + 10;
    public final static double PICKUP_Y = READYPICKUP_Y - 5;
    public final static int CIRCLE_CNT = 2256;
    public final static int SHOVEL_CIRCLE_CNT = 1988;
    public final static double LOW_GEAR = 0.3;
    public final static double FULL_GEAR = 1.0;
	public static double gear = LOW_GEAR;
	
	
	public static DrvByDistanceCmd drvByDistanceCmd;
	public static TurnByGyroCmd turnByGyroCmd;
	public static RobotAutoDriveLCmd robotAutoDriveLCmd;
	public static RobotAutoDriveRCmd robotAutoDriveRCmd;
	public static RobotAutoDriveMCmd robotAutoDriveMCmd;
	public static TurnByGyroSubsystem turnByGyroSubsystem; 
	
	public static GoStraightByGyroSubSystem goStraightByGyroSubSystem; 
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	DriveCommand driveCommand;
	private boolean testFlag = false;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		robotAutoDriveLCmd = new RobotAutoDriveLCmd();
		robotAutoDriveMCmd = new RobotAutoDriveMCmd();
		robotAutoDriveRCmd = new RobotAutoDriveRCmd();
		
		m_chooser.addDefault("Pos 1 -Left", robotAutoDriveLCmd);
		m_chooser.addObject("Pos 2 - Middle", robotAutoDriveMCmd);
		m_chooser.addObject("Pos 3 - Right", robotAutoDriveRCmd);
		SmartDashboard.putData("Position Selection", m_chooser);
		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();
		RobotMap.gyroSPI.calibrate();
		
		Timer.delay(1.0);
		j1ResetPos = RobotMap.jointEncoder1.getRaw();
		j2ResetPos = RobotMap.jointEncoder2.getRaw();
		j3ResetPos = RobotMap.jointEncoder3.getRaw();
		armPosX = RESET_X;
		armPosY = RESET_Y;
		double[] thelta = KinematicsFunction.getJointAngle(RESET_X, RESET_Y);
        
        j1ResetAngle = thelta[0];
        j2ResetAngle = thelta[1];

        j1TurnAngle = thelta[0] - j1ResetAngle;
        j2TurnAngle = thelta[1] - j2ResetAngle;

        j1ResetPos = 0;
        j2ResetPos = 0;
        j1EncoderPos = (int)(j1ResetPos - (j1TurnAngle / (2 * Math.PI) * CIRCLE_CNT));
        j2EncoderPos = (int)(j2ResetPos - (j2TurnAngle / (2 * Math.PI) * CIRCLE_CNT)); 
		
      //for camera
      	//CameraServer.getInstance().addAxisCamera("10.71.76.37");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		if (drvByDistanceCmd != null) drvByDistanceCmd.cancel();
		if (turnByGyroCmd != null)  turnByGyroCmd.cancel();
		
		if (robotAutoDriveLCmd != null) {
			robotAutoDriveLCmd.cancel();
			
			
		}
		if (robotAutoDriveMCmd != null) {
			robotAutoDriveMCmd.cancel();
			
			
		}
		if (robotAutoDriveRCmd != null) {
			robotAutoDriveRCmd.cancel();
			
			
		}
		
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autoStartTime = System.currentTimeMillis();
		m_autonomousCommand = m_chooser.getSelected();
		//receive FMS system command
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		//the command will look like LRL, LLL, RLR ... to tell your which side is your switch and scale
		// the first letter for switch, the L-left, R- Right
		//here we need know which side of scale is our color
		//0 -- for switch
		// 1- for scale
		// 2 - for other side switch
		System.out.println(gameData);
		
		
		
		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();
		RobotMap.gyroSPI.reset();
		//arm hold current position
		joint1Cmd = new Joint1Cmd(RobotMap.jointEncoder1.getRaw(), 0, 500);
		joint2Cmd = new Joint2Cmd(RobotMap.jointEncoder2.getRaw(), 0, 500);
		joint3Cmd = new Joint3Cmd(RobotMap.jointEncoder3.getRaw(), 0, 500);
		joint1Cmd.start();
		joint2Cmd.start();
		joint3Cmd.start();
		
		
		String autoSelected = SmartDashboard.getString("Position Selection", ""); 
		System.out.println("Select = " + m_autonomousCommand);
		if (m_autonomousCommand == robotAutoDriveLCmd) { 
			if(gameData.charAt(1) == 'L')
			{
				robotAutoDriveLCmd.setFlag(0);
				System.out.println("L");
			} else {
				robotAutoDriveLCmd.setFlag(1);
				System.out.println("R");
			}
			robotAutoDriveLCmd.start();
			System.out.println("Left Position");
		}
		else if (m_autonomousCommand == robotAutoDriveMCmd) {
			robotAutoDriveMCmd.setFlag(0);
			robotAutoDriveMCmd.start();
			System.out.println("Middle Position");
		}
		else{
			
			if(gameData.charAt(1) == 'R')
			{
				robotAutoDriveRCmd.setFlag(0);
				System.out.println("R");
			} else {
				robotAutoDriveRCmd.setFlag(1);
				System.out.println("R");
			}
			robotAutoDriveRCmd.start();
			System.out.println("Right Position");
		}

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		long time = System.currentTimeMillis() - autoStartTime;	//time since auto started
		SmartDashboard.putNumber("Auto Running Time ", time/1000);
		if (time <= 14000 ) {
			Scheduler.getInstance().run();
		}else {
			Scheduler.getInstance().disable();
		}
		
		SmartDashboard.putNumber("Gyro angle ", RobotMap.gyroSPI.getAngle());
		SmartDashboard.putNumber("left encoder reading", RobotMap.leftEncoder.getRaw());
		SmartDashboard.putNumber("right encoder reading",RobotMap. rightEncoder.getRaw());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		//start receive the joystick drv command 
		driveCommand = new DriveCommand();
		driveCommand.start();
		/*
		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();*/
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//pov(0) -- -1 not pressed, return 0 degree-- up position, 90- right, 180 - down - 270 left,  
		// getThrottel -- right trigger button 0 -- not pressed , 1 - pressed
		// getTwist  - left trigger button   0 -- not pressed, 1 -- pressed
		 
		if ( m_oi.joyStick.getTwist() < 0.6){  // Right trigger button
			gear --;
			if (gear <=0) gear = LOW_GEAR;
		}
		
		if (m_oi.joyStick.getRawButton(5)) {
			gear ++;
			if (gear >= 1) gear = FULL_GEAR;
		}
			

		if(m_oi.joyStick.getRawButtonReleased(8))
		{
			
			//test auto lift up task here Move2ScaleFromReset
			stopAllHighLevelCmd();
			move2ScaleFromReset = new Move2ScaleFromReset();
			move2ScaleFromReset.start();
			testFlag = true;

		}
		if(m_oi.joyStick.getRawButtonReleased(7))		{
			stopAllHighLevelCmd();
			armMiddlePositionCmd = new ArmMiddlePositionCmd();
			armMiddlePositionCmd.start();

			
		}
		if(m_oi.joyStick.getRawButtonReleased(1))		{
			stopAllHighLevelCmd();
			armPickReadyCmd = new ArmPickReadyCmd();
			armPickReadyCmd.start();
		}
		if(m_oi.joyStick.getRawButtonReleased(2))		{
			stopAllHighLevelCmd();
			armPickupCmd = new ArmPickupCmd();
			armPickupCmd.start();
		}
		
		if(m_oi.joyStick.getRawButtonReleased(3))		{
			/*
			stopAllHighLevelCmd();
			armResetCmd = new ArmResetCmd();
			armResetCmd.start();*/
			
		}
		
		
		if(m_oi.joyStick.getRawButtonReleased(4))		{
			stopAllHighLevelCmd();
			armPosForScaleCmd = new ArmPosForScaleCmd();
			armPosForScaleCmd.start();
			
		}
		
		if(m_oi.joyStick.getRawButtonReleased(6))		{ //RB
			stopAllHighLevelCmd();
			armPosReleaseCmd = new ArmPosReleaseCmd();
			armPosReleaseCmd.start();
			
		}
	
		SmartDashboard.putNumber("encoder1 reading", RobotMap.jointEncoder1.getRaw());
		SmartDashboard.putNumber("encoder2 reading", RobotMap.jointEncoder2.getRaw());
		SmartDashboard.putNumber("encoder3 reading", RobotMap.jointEncoder3.getRaw());
	
		SmartDashboard.putNumber("gear = ", gear);
		SmartDashboard.putNumber("Gyro angle ", RobotMap.gyroSPI.getAngle());
		SmartDashboard.putNumber("left encoder reading", RobotMap.leftEncoder.getRaw());
		SmartDashboard.putNumber("right encoder reading",RobotMap. rightEncoder.getRaw());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	// stop all the previous command
	
	private void stopAllHighLevelCmd() {
		if (move2ScaleFromReset != null) move2ScaleFromReset.cancel();
		if (armResetCmd != null) armResetCmd.cancel();
		if (armPickReadyCmd != null) armPickReadyCmd.cancel();
		if (armPickupCmd != null) armPickupCmd.cancel();
		if (armPosForExchangeCmd != null) armPosForExchangeCmd.cancel();
		if (armPosForScaleCmd != null) armPosForScaleCmd.cancel();
		if (armPosReleaseCmd != null) armPosReleaseCmd.cancel();
		if (armMiddlePositionCmd != null) armMiddlePositionCmd.cancel();
	}
}