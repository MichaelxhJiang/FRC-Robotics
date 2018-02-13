package org.usfirst.frc.team7176.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team7176.robot.KinematicsFunction;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.commands.Joint1Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint2Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;


public class ArmResetCmd  extends Command {
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	private int moveStep = 0;
    private int moveTime = 6000;
    private int movePlanStep = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double currentX = 0;
    private double currentY = 0;
    private double [] moveStepX = new double[1000];
    private double [] moveStepY = new double[1000];
    private final int EXE_TIME = 20;
	public ArmResetCmd() {
		currentX = Robot.armPosX;
        currentY = Robot.armPosY;
        targetX = Robot.RESET_X;
        targetY = Robot.RESET_Y;
       
        
        Robot.j1TurnAngle = 0;
        Robot. j2TurnAngle = 0;
        
        Robot.j1EncoderPos = 0;
        Robot.j2EncoderPos = 0;
        Robot.j3EncoderPos = 0;
        //go to that encoder position
        Robot.joint1Cmd = new Joint1Cmd(j1Encoder.getRaw(), Robot.j1EncoderPos, moveTime);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), Robot.j2EncoderPos, moveTime);
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), Robot.j3EncoderPos, moveTime);
        
        Robot.joint1Cmd.start();
        Robot.joint2Cmd.start();
        Robot.joint3Cmd.start();

        Robot.armPosX = Robot.RESET_X;
        Robot.armPosY = Robot.RESET_X;
		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
	
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (Robot.joint1Cmd.isFinished() && Robot.joint2Cmd.isFinished() && Robot.joint3Cmd.isFinished()) {
			//Robot.joint3Subsystem.disable();  //for arm maybe we could not disable the PID to keep arm at position
			System.out.println("Reset Job done");
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
