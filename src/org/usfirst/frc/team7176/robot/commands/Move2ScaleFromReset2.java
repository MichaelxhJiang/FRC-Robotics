package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.Robot.ArmPosition;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class Move2ScaleFromReset2 extends Command {
	private final static int RUN_TIME = 2000;
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = 2000;
    private int movePlanStep = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double currentX = 0;
    private double currentY = 0;
    private double [] moveStepX = new double[1000];
    private double [] moveStepY = new double[1000];
    private final int EXE_TIME = 20;

	public Move2ScaleFromReset2() {
		 currentX = Robot.armPosX;
         currentY = Robot.armPosY;
         targetX = Robot.SCALE_X;
         targetY = Robot.SCALE_Y;
         
         moveStep = 0;
         
         moveTime = RUN_TIME;
         movePlanStep = moveTime / EXE_TIME;
         for (int i = 0; i < movePlanStep; i++){
             moveStepX[i] = currentX + (targetX - currentX) / movePlanStep * (i + 1);
             moveStepY[i] = currentY + (targetY - currentY) / movePlanStep * (i + 1);
         }
         
         double[] thelta = KinematicsFunction.getJointAngle(moveStepX[moveStep], moveStepY[moveStep]);

         Robot.j1TurnAngle = thelta[0] - Robot.j1ResetAngle;
         Robot. j2TurnAngle = thelta[1] - Robot.j2ResetAngle;
         
         Robot.j1EncoderPos = (int)(Robot.j1ResetPos - (Robot.j1TurnAngle / (2 * Math.PI) *Robot. CIRCLE_CNT_J1));
         Robot.j2EncoderPos = (int)(Robot.j2ResetPos + (Robot.j2TurnAngle / (2 * Math.PI) * Robot.CIRCLE_CNT_J2)); ;
         
        
		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		//go to that encoder position
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), Robot.j1EncoderPos, EXE_TIME);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), Robot.j2EncoderPos, EXE_TIME);
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(-30.0 /360.0 * Robot.SHOVEL_CIRCLE_CNT), 300);
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
        	 Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(- 75.0 /360.0 * Robot.SHOVEL_CIRCLE_CNT), 500);
        	 Robot.joint3Cmd.start();
         }
         else
         {
             
        	 double[] thelta = KinematicsFunction.getJointAngle(moveStepX[moveStep], moveStepY[moveStep]);
             
             Robot.j1TurnAngle = thelta[0] - Robot.j1ResetAngle;
             Robot. j2TurnAngle = thelta[1] - Robot.j2ResetAngle;
             
             Robot.j1EncoderPos = (int)(Robot.j1ResetPos - (Robot.j1TurnAngle / (2 * Math.PI) *Robot. CIRCLE_CNT_J1));
             Robot.j2EncoderPos = (int)(Robot.j2ResetPos + (Robot.j2TurnAngle / (2 * Math.PI) * Robot.CIRCLE_CNT_J2)); ;
             
             //go to that encoder position
             Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), Robot.j1EncoderPos, EXE_TIME);
             Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), Robot.j2EncoderPos, EXE_TIME);
             Robot.joint1Cmd.start();
             Robot.joint2Cmd.start();
             double[] pos = KinematicsFunction.getPositionXY(thelta[0], thelta[1]);
             Robot.armPosX = pos[0];
             Robot.armPosY = pos[1];
             //System.out.println("j1EnocdePos = " + Robot.j1EncoderPos);
             //System.out.println("j2EnocdePos = " + Robot.j2EncoderPos);
         }
	
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (moveStep >= movePlanStep) {
			
			System.out.println("rise up Job done");
			Robot.armPosition = ArmPosition.SCALE;
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
