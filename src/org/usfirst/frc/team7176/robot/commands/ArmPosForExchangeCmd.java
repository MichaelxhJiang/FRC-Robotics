package org.usfirst.frc.team7176.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.commands.Joint1Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint2Cmd;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;


public class ArmPosForExchangeCmd  extends Command {
	private static final int exeTime = 1000;	//1 second(1000ms) to get reset positiom
	private Encoder j1Encoder  = RobotMap.jointEncoder1;
	private Encoder j2Encoder  = RobotMap.jointEncoder2;
	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	public ArmPosForExchangeCmd() {
		
		
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
			System.out.println("Scale Pos Job done");
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

