package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class ReleaseHookCmd  extends Command{
	private static final int RUN_TIME = 1000;	//1 second(1000ms) to get reset positiom
	private static final int JOINT1_POS = (int)(663  + 5.0/360 * Robot.CIRCLE_CNT_J1);// 511 -- straight down;
	private static final int JOINT2_POS = -1364; //  -689 -- straight dwon;
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = 3000;
    private int movePlanStep = 0;

    private final int EXE_TIME = 20;

	public ReleaseHookCmd() {
         
         moveStep = 0;
         
         moveTime = RUN_TIME;
         movePlanStep = moveTime / EXE_TIME;
        
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		//go to that encoder position, hard code some value here, encoder value and position value
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), JOINT1_POS, moveTime);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), JOINT2_POS, moveTime);
       
        Robot.joint1Cmd.start();
        Robot.joint2Cmd.start();

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		 moveStep++;
         if (moveStep >= movePlanStep)
         {

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
