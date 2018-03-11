package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class HookArmPositionCmd extends Command {
	private static final int exeTime = 3000;	//1 second(1000ms) to get reset positiom
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = 3000;
    private int movePlanStep = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double currentX = 0;
    private double currentY = 0;
    private double [] moveStepX = new double[1000];
    private double [] moveStepY = new double[1000];
    private final int EXE_TIME = 20;

	public HookArmPositionCmd() {
		 currentX = Robot.armPosX;
         currentY = Robot.armPosY;
         targetX = Robot.MIDDLE_X;
         targetY = Robot.MIDDLE_Y;
         
         moveStep = 0;
         
         moveTime = exeTime;
         movePlanStep = moveTime / EXE_TIME;
         for (int i = 0; i < movePlanStep; i++){
             moveStepX[i] = currentX + (targetX - currentX) / movePlanStep * i;
             moveStepY[i] = currentY + (targetY - currentY) / movePlanStep * i;
         }
         //uppder elbow function getJointAngleL
         double[] thelta = KinematicsFunction.getJointAngleL(moveStepX[moveStep], moveStepY[moveStep]);

         Robot.j1TurnAngle = thelta[0] - Robot.j1ResetAngle;
         Robot. j2TurnAngle = thelta[1] - Robot.j2ResetAngle;
         
         Robot.j1EncoderPos = (int)(Robot.j1ResetPos - (Robot.j1TurnAngle / (2 * Math.PI) *Robot. CIRCLE_CNT));
         Robot.j2EncoderPos = (int)(Robot.j2ResetPos + (Robot.j2TurnAngle / (2 * Math.PI) * Robot.CIRCLE_CNT)); 		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		//go to that encoder position, hard code some value here, encoder value and position value
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), 1030, moveTime);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), -990, moveTime);
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(-15.0/360 * Robot.SHOVEL_CIRCLE_CNT), moveTime);
        Robot.joint1Cmd.start();
        Robot.joint2Cmd.start();
        Robot.joint3Cmd.start();
        Robot.armPosX = Robot.MIDDLE_X;
        Robot.armPosY = Robot.MIDDLE_Y;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		 moveStep++;
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
