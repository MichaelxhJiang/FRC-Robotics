package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class SwitchDropAutoCmd extends Command {
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;
	private final static int EXE_TIME = 500;
	private static int step;

	public SwitchDropAutoCmd() {
       
        Robot.j3EncoderPos = -150;	//positive rotates upwards, negative rotates downwards
        //go to that encoder position
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), Robot.j3EncoderPos, EXE_TIME);
        
        Robot.joint3Cmd.start();
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		step = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (step == 0) {
			if (Robot.joint3Cmd.isFinished()) {
				Robot.joint3Cmd.cancel();
				
				Robot.j3EncoderPos = 0;
				Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), Robot.j3EncoderPos, EXE_TIME);
				
				Robot.joint3Cmd.start();
				step = 1;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (Robot.joint3Cmd.isFinished() && step == 1) {
			//Robot.joint3Subsystem.disable();  //for arm maybe we could not disable the PID to keep arm at position
			System.out.println("Switch Job done");
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
