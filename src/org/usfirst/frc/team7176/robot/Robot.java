/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7176.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;


import org.usfirst.frc.team7176.robot.commands.DriveCommand;
import org.usfirst.frc.team7176.robot.commands.DrvByDistanceCmd;
import org.usfirst.frc.team7176.robot.commands.DrvByTimeGyroCmd;
import org.usfirst.frc.team7176.robot.commands.DrvByTimeNoCtrlCmd;
import org.usfirst.frc.team7176.robot.commands.HookArmPositionCmd;
import org.usfirst.frc.team7176.robot.commands.HookLiftCmd;
import org.usfirst.frc.team7176.robot.commands.HookLiftCmd2;
import org.usfirst.frc.team7176.robot.commands.Joint1Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint2Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;
import org.usfirst.frc.team7176.robot.commands.Move2ScaleFromReset;
import org.usfirst.frc.team7176.robot.commands.Move2ScaleFromReset2;
import org.usfirst.frc.team7176.robot.commands.PullArmCmd;
import org.usfirst.frc.team7176.robot.commands.ReleaseHookCmd;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveLCmd;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveMCmd;
import org.usfirst.frc.team7176.robot.commands.RobotAutoDriveRCmd;
import org.usfirst.frc.team7176.robot.commands.SwitchDropAutoCmd;
import org.usfirst.frc.team7176.robot.commands.TurnByGyroCmd;
import org.usfirst.frc.team7176.robot.commands.ArmResetCmd;
import org.usfirst.frc.team7176.robot.commands.CalibratePullForceCmd;
import org.usfirst.frc.team7176.robot.commands.ClimbingCmd;
import org.usfirst.frc.team7176.robot.commands.ClimbingCmd2;
import org.usfirst.frc.team7176.robot.commands.ArmThrowCubeCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosForScaleCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosHoldCmd;
import org.usfirst.frc.team7176.robot.commands.ArmReleaseCubeCmd;
import org.usfirst.frc.team7176.robot.commands.ArmMiddlePositionCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPickReadyCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPickReadyHCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPickupCmd;
import org.usfirst.frc.team7176.robot.commands.ArmPosForExchangeCmd;
import org.usfirst.frc.team7176.robot.subsystems.DriveSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.GoStraightByGyroSubSystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint1PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint2PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.Joint3PIDSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.PullArmSubsystem;
import org.usfirst.frc.team7176.robot.subsystems.TurnByGyroSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.usfirst.frc.team7176.robot.KinematicsFunction;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project
 */
public class Robot extends TimedRobot {
	private static final boolean AGGRESSIVE = true;	//put block on scale = true
	private static final boolean SWITCH = false;	//put block on switch instead of scale
	private long autoStartTime = 0;
	public static RobotMap robotMap = new RobotMap();
	public static OI m_oi = new OI();
	//subsystem
	public static final DriveSubsystem drvSubsystem	= new DriveSubsystem();
	public static final Joint1PIDSubsystem joint1Subsystem = new Joint1PIDSubsystem();
	public static final Joint2PIDSubsystem joint2Subsystem = new Joint2PIDSubsystem();
	public static final Joint3PIDSubsystem joint3Subsystem = new Joint3PIDSubsystem();
	public static TurnByGyroSubsystem turnByGyroSubsystem;
	public static GoStraightByGyroSubSystem goStraightByGyroSubSystem; 
//	public static PullArmSubsystem pullArmSubsystem = new PullArmSubsystem();
	//command
	public static Joint1Cmd joint1Cmd;
	public static Joint2Cmd joint2Cmd;
	public static Joint3Cmd joint3Cmd;
	public static Move2ScaleFromReset move2ScaleFromReset;
	public static Move2ScaleFromReset2 move2ScaleFromReset2;
	public static ArmResetCmd armResetCmd;
	public static ArmPickReadyCmd armPickReadyCmd;
	public static ArmPickupCmd armPickupCmd;
	public static ArmPosHoldCmd armPosHoldCmd;
	public static ArmPosForScaleCmd armPosForScaleCmd;
	public static ArmThrowCubeCmd armThrowCubeCmd;
	public static ArmReleaseCubeCmd armReleaseCubeCmd;
	public static ArmMiddlePositionCmd armMiddlePositionCmd;
	public static ArmPosForExchangeCmd armPosForExchangeCmd;
	public static ArmPickReadyHCmd armPickReadyHCmd;
	public static HookArmPositionCmd hookArmPositionCmd;
	public static ReleaseHookCmd releaseHookCmd;
	

	public static DrvByTimeNoCtrlCmd drvByTimeNoCtrlCmd;
	public static DrvByTimeGyroCmd drvByTimeGyroCmd;
	public static DrvByDistanceCmd drvByDistanceCmd;
	public static TurnByGyroCmd turnByGyroCmd;
	public static RobotAutoDriveLCmd robotAutoDriveLCmd;
	public static RobotAutoDriveRCmd robotAutoDriveRCmd;
	public static RobotAutoDriveMCmd robotAutoDriveMCmd;
	public static SwitchDropAutoCmd switchDropAutoCmd;
	
	public static HookLiftCmd hookLiftCmd;
	public static HookLiftCmd2 hookLiftCmd2;
	//public static PullArmCmd pullArmCmd;
	
	//arm setting
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
    public final static double RESET_X = -14.0;
    public final static double RESET_Y = 2.0;
    public final static double SCALE_X = -24.0;  //-15, test -24
    public final static  double SCALE_Y = 120;
    public final static double SWITCH_X = -50.85;
    public final static double SWITCH_Y = 20.20;
    public final static double READYPICKUP_X = -71.85;//-60.77;
    public final static double READYPICKUP_Y = -60 + 7 + 2 + 1;//-61.50;
    public final static  double MIDDLE_X = -24.0;
    public final static double MIDDLE_Y = 17.5;
    public final static  double PICKUP_X = READYPICKUP_X;
    public final static double PICKUP_Y = READYPICKUP_Y - 5.0;  //6
    public final static int CIRCLE_CNT_J1 = 300 * 4;//(int)(7 * 4 * 188 * 72.0 / 42.0 );   //1140 = 360 *4
    public final static int CIRCLE_CNT_J2 = (int)( 3 * 4 * 188);
    public final static int SHOVEL_CIRCLE_CNT = 1988;
    public final static double LOW_GEAR = 0.50;
    public final static double FULL_GEAR = 0.8;
	public static double gear = LOW_GEAR;
	public final static double READYPICKUP_XH = READYPICKUP_X;
	public final static double READYPICKUP_YH = READYPICKUP_Y + 5;
	
	public final static int JOINT1_HLM = 700;//(5000 = 188 motor with 7 count , ??? for 188 motor with 3 count; 880 for 188 motor with US digital 1140 encoder
	public final static int JOINT1_LLM = 0;
	public final static int JOINT2_HLM = 0;
	public final static int JOINT2_LLM = -1976;
	public final static int JOINT3_HLM = 997;
	public final static int JOINT3_LLM = -568;

	public static boolean inMovingCarBack = false;
	//arm position enum
	public enum ArmPosition {
	    SCALE, SWITCH, PICKUP, PICKUP_H,
	    PORTAL, RESET, MIDDLE_POS,HOOK_POS,NOWHERE 
	}
	public static ArmPosition armPosition = ArmPosition.RESET;
	
	public static boolean autoPickUpFlag = false;
	public static boolean inPickUpProcess = false;
	
	public static final int PULL_ARM_POS = (int)(1440 * 2.5);
	
	public static int climbingEncoderPos_ResetArm = - 4096 * 15;
	public static int climbingEncoderPos1 = -4096 * 24;
	public static int climbingEncoderPos2 = -4096 * 23;
	public static ClimbingCmd  climbingCmd;
	public static ClimbingCmd2  climbingCmd2;
	public static final int SET_PICKREADY_POS = 405;
	
//	public static CalibratePullForceCmd calibratePullForceCmd;
	
	private int flagValue1 = 0; 
	
	//for testing
	public static long testingStartTime = 0;
	public static long testRunTime = 0;
	public static double testDistance = 0;
	
	
	public static double detectDisLeft = 0.8;
	public static double detectDisRight = 0.8;
	
	private AnalogInput leftIR;
	private AnalogInput rightIR;
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	
	SendableChooser<Boolean> m_AutoPickUp = new SendableChooser<>();
	
	DriveCommand driveCommand;
	
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
		
		m_AutoPickUp.addDefault("Disable_AuotPickUp", false);
		m_AutoPickUp.addObject("Enable_AuotPickUp", true);
		SmartDashboard.putData("Auto Pickup Selection", m_AutoPickUp);
		
		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();
		RobotMap.hookEncoder.reset();
		RobotMap.gyroSPI.calibrate();
        leftIR = new AnalogInput(0);
        rightIR = new AnalogInput(1);
        
		Timer.delay(1.0);
		
		j1ResetPos = RobotMap.jointEncoder1.getRaw();
		j2ResetPos = RobotMap.jointEncoder2.getRaw();
		j3ResetPos = RobotMap.jointEncoder3.getRaw();
		armPosX = RESET_X;
		armPosY = RESET_Y;
		double[] thelta = KinematicsFunction.getJointAngle(RESET_X, RESET_Y);
        
        j1ResetAngle = thelta[0];
        j2ResetAngle = thelta[1];// - 3.0/180 * Math.PI;

        j1TurnAngle = thelta[0] - j1ResetAngle;
        j2TurnAngle = thelta[1] - j2ResetAngle;

        j1ResetPos = 0;
        j2ResetPos = 0;
        j1EncoderPos = (int)(j1ResetPos - (j1TurnAngle / (2 * Math.PI) * CIRCLE_CNT_J1));
        j2EncoderPos = (int)(j2ResetPos - (j2TurnAngle / (2 * Math.PI) * CIRCLE_CNT_J2)); 
		
        armPosition = ArmPosition.RESET;
        
      //for camera
      	CameraServer.getInstance().addAxisCamera("10.71.76.37");
        //CameraServer.getInstance().startAutomaticCapture();  this one is for USB camera to robotRIO
        
      //for test command
      	
		//SmartDashboard.putData("Test Turn 90 Command", new TurnByGyroCmd(0.5,90));
	//	SmartDashboard.putData("Test Turn -90 Command", new TurnByGyroCmd(0.5,-90));
	//	SmartDashboard.putData("Test go straight Command", new DrvByDistanceCmd(1.0,450));
		
		
		
      	//SmartDashboard.putData("Pull Calibrate Cmd", new CalibratePullForceCmd());
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
		if (turnByGyroSubsystem != null) turnByGyroSubsystem.disable();
		if (goStraightByGyroSubSystem != null) goStraightByGyroSubSystem.disable();
		
		if (drvByTimeNoCtrlCmd != null) {
			drvByTimeNoCtrlCmd.cancel();
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
		SmartDashboard.putString("Game Data: ", gameData);
		//the command will look like LRL, LLL, RLR ... to tell your which side is your switch and scale
		// the first letter for switch, the L-left, R- Right
		//here we need know which side of scale is our color
		//0 -- for switch
		// 1- for scale
		// 2 - for other side switch
		if (gameData.length() < 3) gameData = "LXL";
		System.out.println(gameData);

		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();
		RobotMap.gyroSPI.reset();
		//arm hold current position
		joint1Cmd = new Joint1Cmd(RobotMap.jointEncoder1.getRaw(), 0, 20);
		joint2Cmd = new Joint2Cmd(RobotMap.jointEncoder2.getRaw(), 0, 20);
		joint3Cmd = new Joint3Cmd(RobotMap.jointEncoder3.getRaw(), 0, 20);
		joint1Cmd.start();
		joint2Cmd.start();
		joint3Cmd.start();
		
		
	//	String autoSelected = SmartDashboard.getString("Position Selection", ""); 
		System.out.println("Select = " + m_autonomousCommand);
		if (m_autonomousCommand == robotAutoDriveLCmd) { 
			if(gameData.charAt(1) == 'L')
			{
				if (!AGGRESSIVE) {
					robotAutoDriveLCmd.setFlag(0);
				}else {
					robotAutoDriveLCmd.setFlag(2);
				}
				
				System.out.println("L");
			} else if(gameData.charAt(1) == 'R'){
				robotAutoDriveLCmd.setFlag(1);
				System.out.println("R");
			}
			else {	//by default go to Left
				robotAutoDriveLCmd.setFlag(0);
				System.out.println("L");
			}
			robotAutoDriveLCmd.start();
			System.out.println("Left Position");
		}
		else if (m_autonomousCommand == robotAutoDriveMCmd) {
			if (gameData.charAt(0) == 'L') {
				robotAutoDriveMCmd.setFlag(1);
				robotAutoDriveMCmd.start();
			} else {
				robotAutoDriveMCmd.setFlag(2);
				robotAutoDriveMCmd.start();
			}
			
			System.out.println("Middle Position");
		}
		else{
			
			if(gameData.charAt(1) == 'R')
			{
				if (!AGGRESSIVE) {
					robotAutoDriveRCmd.setFlag(0);
				}else {
					robotAutoDriveRCmd.setFlag(2);
				}
				
				System.out.println("R");
			} else if(gameData.charAt(1) == 'L'){
				robotAutoDriveRCmd.setFlag(1);
				System.out.println("L");
			}else {
				robotAutoDriveRCmd.setFlag(0);
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
		if (time <= 15000 ) {
			Scheduler.getInstance().run();
		}else {
			//Scheduler.getInstance().disable();
		}
		
		SmartDashboard.putNumber("Gyro angle ", RobotMap.gyroSPI.getAngle());
		SmartDashboard.putNumber("left encoder reading", RobotMap.leftEncoder.getRaw());
		//SmartDashboard.putNumber("right encoder reading",RobotMap. rightEncoder.getRaw());
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
		
		
		
		//do not reset encoder otherwise we will lost joint angle
		//below code is just for test
		/*
		RobotMap.leftMotor1.set(0);
		RobotMap.leftMotor2.set(0);
		RobotMap.rightMotor1.set(0);
		RobotMap.rightMotor2.set(0);
		
		RobotMap.jointEncoder1.reset();
		RobotMap.jointEncoder2.reset();
		RobotMap.jointEncoder3.reset();
		//arm hold current position
		joint1Cmd = new Joint1Cmd(RobotMap.jointEncoder1.getRaw(), 0, 20);
		joint2Cmd = new Joint2Cmd(RobotMap.jointEncoder2.getRaw(), 0, 20);
		joint3Cmd = new Joint3Cmd(RobotMap.jointEncoder3.getRaw(), 0, 20);
		joint1Cmd.start();
		joint2Cmd.start();
		joint3Cmd.start();*/
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	
		autoPickUpFlag = m_AutoPickUp.getSelected();
	//	System.out.println("Auto Pick up = "  + autoPickUpFlag);
		detectDisLeft = ad2Dis(leftIR.getVoltage()) ;
		detectDisRight = ad2Dis(rightIR.getVoltage());
	//	SmartDashboard.putBoolean("Cube under Shovel = ", false);
		if ((armPosition == ArmPosition.PICKUP || armPosition == ArmPosition.PICKUP_H)) {
			if ((detectDisLeft < 0.23) && (detectDisRight < 0.23)) {
				SmartDashboard.putBoolean("Cube under Shovel = ", true);
				if (autoPickUpFlag) {
					stopAllHighLevelCmd();
     				armPickupCmd = new ArmPickupCmd();
					armPickupCmd.start();
					inPickUpProcess = true;
					//stop or slow down?
				//	RobotMap.leftMotor1.set(0.1);
				//	RobotMap.leftMotor2.set(0.1);
				//	RobotMap.rightMotor1.set(-0.1);
				//	RobotMap.rightMotor2.set(-0.1);
				}
			}
			else {
				SmartDashboard.putBoolean("Cube under Shovel = ", false);
			}
		}else {
			SmartDashboard.putBoolean("Cube under Shovel = ", false);
		}
		
		
		//pov(0) -- -1 not pressed, return 0 degree-- up position, 90- right, 180 - down - 270 left,  
		// getThrottel -- right trigger button 0 -- not pressed , 1 - pressed
		// getTwist  - left trigger button   0 -- not pressed, 1 -- pressed
		 
		if (m_oi.joyStick.getThrottle() > 0.5) {
			if (flagValue1 < 0.5) {
				flagValue1++;
				//stopAllHighLevelCmd();
				armThrowCubeCmd = new ArmThrowCubeCmd();
				armThrowCubeCmd.start();
				System.out.println("releae cube to scale");
			}
			
		}else {
			flagValue1 --;
			if (flagValue1 <=0) flagValue1 = 0;
		}
		
		if ( m_oi.joyStick.getTwist() > 0.6){  // Right trigger button
			gear = FULL_GEAR;
		}else {
			gear = LOW_GEAR; 
		}
		
		if (m_oi.joyStick.getRawButtonReleased(5)) {
			//for high power up cube
			stopAllHighLevelCmd();
			armPickReadyHCmd = new ArmPickReadyHCmd();
			armPickReadyHCmd.start();
			System.out.println("5");
		}
			

		if(m_oi.joyStick.getRawButtonReleased(7))
		{
			
			//test auto lift up task here Move2ScaleFromReset
			stopAllHighLevelCmd();
			move2ScaleFromReset = new Move2ScaleFromReset();
			move2ScaleFromReset.start();
			
			System.out.println("7");

		}
		if(m_oi.joyStick.getRawButtonReleased(8))		{
			stopAllHighLevelCmd();
			armMiddlePositionCmd = new ArmMiddlePositionCmd();
			armMiddlePositionCmd.start();
			System.out.println("8");
			
		}
		if(m_oi.joyStick.getRawButtonReleased(1))		{
			stopAllHighLevelCmd();
			armPickReadyCmd = new ArmPickReadyCmd();
			armPickReadyCmd.start();
			System.out.println("1");
		}
		if(m_oi.joyStick.getRawButtonReleased(2))		{
			stopAllHighLevelCmd();
			armPickupCmd = new ArmPickupCmd();
			armPickupCmd.start();
			inPickUpProcess = true;
			System.out.println("2");
		}
		
		if(m_oi.joyStick.getRawButtonReleased(3))		{
			stopAllHighLevelCmd();
			armPosHoldCmd = new ArmPosHoldCmd();
			armPosHoldCmd.start();
			System.out.println("3");
		}
		
		
		
		if(m_oi.joyStick.getRawButtonReleased(4))		{
			stopAllHighLevelCmd();
			armPosForScaleCmd = new ArmPosForScaleCmd();
			armPosForScaleCmd.start();
			System.out.println("4");
			
		}
		
		
		
		
		if(m_oi.joyStick.getRawButtonReleased(6))		{ //RB
			//stopAllHighLevelCmd();
			armReleaseCubeCmd = new ArmReleaseCubeCmd();
			armReleaseCubeCmd.start();
			System.out.println("6");
		}
		
		
		if (m_oi.joyStick2.getRawButtonReleased(4)) {
			stopAllHighLevelCmd();
			
			hookArmPositionCmd = new HookArmPositionCmd();
			hookArmPositionCmd.start();
			
			System.out.println("Hook Position");
		}
		if (m_oi.joyStick2.getRawButtonReleased(3)) {
			stopAllHighLevelCmd();
			if (climbingCmd != null) climbingCmd = null;
			climbingCmd2 = new ClimbingCmd2();
			climbingCmd2.start();
			System.out.println("Climbing again");
		}
		if (m_oi.joyStick2.getRawButtonReleased(2)) {
			stopAllHighLevelCmd();
			
			releaseHookCmd = new ReleaseHookCmd();
			releaseHookCmd.start();
			
			System.out.println("Released Hook");
		}
		if (m_oi.joyStick2.getRawButtonReleased(1)) {
			//here is for climbing
			RobotMap.hookEncoder.reset();
			climbingCmd = new ClimbingCmd();
			climbingCmd.start();
			
			System.out.println("Climbing");
		}
		if (m_oi.joyStick2.getRawButtonReleased(5)) {
			stopAllHighLevelCmd();
			armResetCmd = new ArmResetCmd();
			armResetCmd.start();
			
			
			System.out.println("reset arm");
		}
		
		
		SmartDashboard.putNumber("encoder1 reading", RobotMap.jointEncoder1.getRaw());
		SmartDashboard.putNumber("encoder2 reading", RobotMap.jointEncoder2.getRaw());
		SmartDashboard.putNumber("encoder3 reading", RobotMap.jointEncoder3.getRaw());
	
		SmartDashboard.putNumber("gear = ", gear);
		SmartDashboard.putNumber("Gyro angle ", RobotMap.gyroSPI.getAngle());
		SmartDashboard.putNumber("left encoder reading", RobotMap.leftEncoder.getRaw());
		SmartDashboard.putNumber("Hook encoder reading",RobotMap.hookEncoder.getRaw());
		
		SmartDashboard.putNumber("left IR Dis", detectDisLeft);
		SmartDashboard.putNumber("right IR Dis ", detectDisRight);
		SmartDashboard.putString("Arm Position", armPosition.toString());
	//	SmartDashboard.putNumber("Joint 1_1 output current ", RobotMap.joint1Motor_1.getOutputCurrent());
	//	SmartDashboard.putNumber("Joint 1_2 output current ", RobotMap.joint1Motor_2.getOutputCurrent());
		SmartDashboard.putNumber("test running time ", (double)testRunTime/1000.0);
		SmartDashboard.putNumber("Go traight distance", testDistance/100);
		
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
		if (armPosHoldCmd != null) armPosHoldCmd.cancel();
		if (armPosForScaleCmd != null) armPosForScaleCmd.cancel();
		if (armReleaseCubeCmd != null) armReleaseCubeCmd.cancel();
		if (armThrowCubeCmd != null) armThrowCubeCmd.cancel();
		if (armMiddlePositionCmd != null) armMiddlePositionCmd.cancel();
		if (armPosForExchangeCmd != null) armPosForExchangeCmd.cancel();
		if (armPickReadyHCmd != null) armPickReadyHCmd.cancel();
		//if (hookLiftCmd != null) hookLiftCmd.cancel();
		if (hookArmPositionCmd != null) hookArmPositionCmd.cancel();
		if (releaseHookCmd != null) releaseHookCmd.cancel();

		if (climbingCmd != null) climbingCmd.cancel();
		
		inPickUpProcess = false;

		armPosition = ArmPosition.NOWHERE;
	}
	
	private double ad2Dis(double adValue) {
		double irAD2Dis = 0;
		double temp = 0;
		temp = 21.6 /(adValue - 0.17);
		if ( (temp > 80) || (temp  < 0)) {
			irAD2Dis = 0.81;
		}
		else if( (temp  < 10) && (temp > 0)) {
			irAD2Dis = 0.09;
		}
		else {
			irAD2Dis = temp /100;
		}
		return irAD2Dis;
	}
}