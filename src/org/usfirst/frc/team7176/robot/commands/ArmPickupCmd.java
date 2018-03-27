package org.usfirst.frc.team7176.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.Robot.ArmPosition;
import org.usfirst.frc.team7176.robot.commands.Joint1Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint2Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;


public class ArmPickupCmd  extends Command {
	private final int DOWN_TIME = 460;
	private final int JOINT3_TIME = 800;
	private final int MOVEUP_TIME = 1500;
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;
	private int moveStep = 0;
    private int moveTime = 1000;
    private int movePlanStep = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double currentX = 0;
    private double currentY = 0;
    private double [] moveStepX = new double[1000];
    private double [] moveStepY = new double[1000];
    private final int EXE_TIME = 20;  // 20ms
    private int step = 0;
    private int armDownStep = 0;
    private final int MOVECAR_BACK_TIMER_CNT = 300/20;  //200ms
    private final double MOVECAR_BACK_POWER = -0.3;
    private int moveCarBackCnt = 0;

	public ArmPickupCmd() {
		step = 0;
		currentX = Robot.armPosX;
        currentY = Robot.armPosY;
        if ((Robot.detectDisLeft < 0.10) || (Robot.detectDisRight < 0.10)) {
        	targetX = Robot.READYPICKUP_X;
            targetY = Robot.READYPICKUP_Y - 9 + 2;
        }
        else {
        	targetX = Robot.READYPICKUP_X;
            targetY = Robot.READYPICKUP_Y - 14.5;
        }
        
        
       
        moveStep = 0;
        
        moveTime = DOWN_TIME;		//down 8-10cm first
        movePlanStep = moveTime / EXE_TIME;
        armDownStep = movePlanStep;
        for (int i = 0; i < movePlanStep; i++)
        {
            moveStepX[i] = currentX + (targetX - currentX) / movePlanStep *(i + 1);
            moveStepY[i] = currentY + (targetY - currentY) / movePlanStep * (i + 1);
        }
        
        
        
        // turn joint 3 and move back a little bit
        currentX = moveStepX[movePlanStep -1];
        currentY = moveStepY[movePlanStep -1];
        targetX = targetX + 0; //2
        targetY = targetY - 0; //2
        moveTime = JOINT3_TIME;
        int firstStep = movePlanStep;
        movePlanStep = movePlanStep  + moveTime / EXE_TIME;
        
        for (int i = firstStep; i < movePlanStep; i++)
        {
            moveStepX[i] = currentX + (targetX - currentX) / (movePlanStep - firstStep) * (i - firstStep + 1);
            moveStepY[i] = currentY + (targetY - currentY) / (movePlanStep - firstStep) * (i - firstStep + 1);
        }

        // then arm up
        currentX = moveStepX[movePlanStep -1];
        currentY = moveStepY[movePlanStep -1];
        targetX = Robot.SWITCH_X;
        targetY = Robot.SWITCH_Y;

        moveTime = MOVEUP_TIME;
        firstStep = movePlanStep;
        movePlanStep = movePlanStep  + moveTime / EXE_TIME;
        
        for (int i = firstStep; i < movePlanStep; i++)
        {
            moveStepX[i] = currentX + (targetX - currentX) / (movePlanStep - firstStep) * (i - firstStep + 1);
            moveStepY[i] = currentY + (targetY - currentY) / (movePlanStep - firstStep) * (i - firstStep + 1);
        }
        
        
        
        double[] thelta = KinematicsFunction.getJointAngleL(moveStepX[moveStep], moveStepY[moveStep]);
                 
        Robot.j1TurnAngle = thelta[0] - Robot.j1ResetAngle;
        Robot. j2TurnAngle = thelta[1] - Robot.j2ResetAngle;
        
        Robot.j1EncoderPos = (int)(Robot.j1ResetPos - (Robot.j1TurnAngle / (2 * Math.PI) *Robot. CIRCLE_CNT_J1));
        Robot.j2EncoderPos = (int)(Robot.j2ResetPos + (Robot.j2TurnAngle / (2 * Math.PI) * Robot.CIRCLE_CNT_J2)); ;
        
        
        double[] pos = KinematicsFunction.getPositionXY(thelta[0], thelta[1]);
        Robot.armPosX = pos[0];
        Robot.armPosY = pos[1];
		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		//go to that encoder position
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), Robot.j1EncoderPos, EXE_TIME);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), Robot.j2EncoderPos, EXE_TIME);
        Robot.joint1Cmd.start();
        Robot.joint2Cmd.start();
        
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		moveStep++;
        
		if (moveStep == armDownStep) {
			Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(130.0/360.0 * Robot.SHOVEL_CIRCLE_CNT), JOINT3_TIME);
	        Robot.joint3Cmd.start();
	        
	        // start move car back
	        RobotMap.leftMotor1.set(MOVECAR_BACK_POWER);
	        RobotMap.leftMotor2.set(MOVECAR_BACK_POWER);
	        RobotMap.rightMotor1.set(-MOVECAR_BACK_POWER);
	        RobotMap.rightMotor2.set(-MOVECAR_BACK_POWER);
	        moveCarBackCnt = 0;
	        Robot.inMovingCarBack = true;
		}
		
		if (Robot.inMovingCarBack) {
			moveCarBackCnt ++;
			System.out.println("Car moving back");
			RobotMap.leftMotor1.set(MOVECAR_BACK_POWER);
	        RobotMap.leftMotor2.set(MOVECAR_BACK_POWER);
	        RobotMap.rightMotor1.set(-MOVECAR_BACK_POWER);
	        RobotMap.rightMotor2.set(-MOVECAR_BACK_POWER);
			if (moveCarBackCnt > MOVECAR_BACK_TIMER_CNT) {
				RobotMap.leftMotor1.set(0);
		        RobotMap.leftMotor2.set(0);
		        RobotMap.rightMotor1.set(0);
		        RobotMap.rightMotor2.set(0);
		        Robot.inMovingCarBack = false;
			}
		}
		
        if (moveStep < movePlanStep)
        {
            
       	 	double[] thelta = KinematicsFunction.getJointAngleL(moveStepX[moveStep], moveStepY[moveStep]);
           // System.out.println("Pick UP Posisiton = " + moveStepX[moveStep] + "," + moveStepY[moveStep] + "  Step =" + moveStep);
       	 	
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
            step = 1;
        } else if (moveStep >= movePlanStep && step == 1) {
        	System.out.println("Adjust 1");
        	Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(j3Encoder.getRaw() - 12.0/360.0 * Robot.SHOVEL_CIRCLE_CNT), 150);
        	Robot.joint3Cmd.start();
        	step = 2;
        } else if (step == 2 && moveStep >= movePlanStep + 20) {
        	System.out.println("Adjust 2");
        	Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(j3Encoder.getRaw() + 12.0/360.0 * Robot.SHOVEL_CIRCLE_CNT), 150);
        	Robot.joint3Cmd.start();
        	step = 3;
        }
        
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if ((moveStep >= movePlanStep + 30) || (step == 3 && Robot.joint3Cmd.isFinished())) {
			//Robot.joint3Subsystem.disable();  //for arm maybe we could not disable the PID to keep arm at position
			System.out.println("Pick up ready Job done");
			Robot.inPickUpProcess = false;
			Robot.armPosition = ArmPosition.SWITCH;
			return true;
		}
		return false;
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

