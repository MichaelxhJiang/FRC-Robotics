package org.usfirst.frc.team7176.robot.commands;

import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Command;

public class SwitchDropAutoCmd extends Command {
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;
	private final static int RUN_TIME = 300;
	private static int step;

	public SwitchDropAutoCmd() {
        //go to that encoder position
        Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), j3Encoder.getRaw() - (int)(60.0/360 * Robot.SHOVEL_CIRCLE_CNT), RUN_TIME);
        Robot.joint2Cmd = new Joint2Cmd(j2Encoder.getRaw(), j2Encoder.getRaw() - (int)(10.0/360 * Robot.CIRCLE_CNT_J2), RUN_TIME);
       
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		System.out.println("SWITCH DROP CMD");
		step = 0;
		Robot.joint2Cmd.start();
	    Robot.joint3Cmd.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (step == 0) {
			if (Robot.joint3Cmd.isFinished() && Robot.joint2Cmd.isFinished()) {
				System.out.println("STEP 1");
				Robot.joint3Cmd.cancel();
				
				Robot.armResetCmd = new ArmResetCmd();
				
				Robot.armResetCmd.start();
				step = 1;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (step == 1 && Robot.armResetCmd.isFinished()) {
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
