package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class HookArmPositionCmd extends Command {
	private static final int RUN_TIME = 1000;	//1 second(1000ms) 
	private static final int JOINT1_POS = (int) (619  + 5.0/360 * Robot.CIRCLE_CNT_J1) ;   // get from test  554
	private static final int JOINT2_POS = -1181;  // -892 
	private static final int JOINT3_POS = -263;
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = RUN_TIME;
    private int movePlanStep = 0;

    private final int EXE_TIME = 20;

	public HookArmPositionCmd() {
         
         moveStep = 0;
         
         moveTime = RUN_TIME;
         movePlanStep = moveTime / EXE_TIME;
        	
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		//go to that encoder position, hard code some value here, encoder value and position value
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), JOINT1_POS, moveTime);			//1030
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), JOINT2_POS, moveTime);
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), JOINT3_POS, moveTime);
        Robot.joint1Cmd.start();
        Robot.joint2Cmd.start();
        Robot.joint3Cmd.start();


	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		 moveStep++;
         if (moveStep >= movePlanStep)
         {
        	 //hook position
         }
         else
         {
             
        	 
         }
	
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (moveStep >= movePlanStep) {
			
			System.out.println("hook position Job done");
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
