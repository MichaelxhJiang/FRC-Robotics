package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.Robot.ArmPosition;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class ArmMiddlePositionCmd extends Command {
	private final static int RUN_TIME =2000; 
	private final static int DELAY_TIME = 450;
	private final int JOINT1_POS = 632;   //encoder position need test
	private final int JOINT2_POS = -1980;  // encoder position need test
	private final int JOINT3_POS = -30; //degree
	
	
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = RUN_TIME;
    private int movePlanStep = 0;
    
    

	public ArmMiddlePositionCmd() {
        
         moveStep = 0;
    
         moveTime = RUN_TIME;
         movePlanStep = moveTime / 20;
        
		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("Starting Arm Middle Position Cmd");
		//go to that encoder position, hard code some value here, encoder value and position value
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), JOINT1_POS, moveTime / 2 ); //  
       // Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), JOINT2_POS, moveTime);  //3s
	   // Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(JOINT3_POS/360.0 * Robot.SHOVEL_CIRCLE_CNT), moveTime - DELAY_TIME);
	    Robot.joint1Cmd.start();
	  //  Robot.joint2Cmd.start();
	 //   Robot.joint3Cmd.start();
        
        moveStep = 0;

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		System.out.println("moveStep = " + moveStep);
		 moveStep++;
		 if (moveStep == DELAY_TIME / 20) {
			 Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), JOINT2_POS, moveTime- DELAY_TIME);  //
			 Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(JOINT3_POS/360.0 * Robot.SHOVEL_CIRCLE_CNT), moveTime - DELAY_TIME);//
			 Robot.joint2Cmd.start();
			 Robot.joint3Cmd.start();
			 System.out.println("start moving Joint2, 3");
		 }

         if (moveStep >= movePlanStep)
         {
        	 Robot.armPosX = Robot.MIDDLE_X;
             Robot.armPosY = Robot.MIDDLE_Y;
         }
         else
         {
             
        	 
         }
	
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (moveStep >= movePlanStep) {
			
			System.out.println("middle position Job done");
			Robot.armPosition = ArmPosition.MIDDLE_POS;
			return true;
		}else {
		
			return false;
		}
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		
		
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}
}
