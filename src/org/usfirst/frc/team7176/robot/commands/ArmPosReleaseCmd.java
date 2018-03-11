package org.usfirst.frc.team7176.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;


public class ArmPosReleaseCmd  extends Command {
	private static final int exeTime = 1000;	//1 second(1000ms) to get reset positiom

	private Encoder j3Encoder  = RobotMap.jointEncoder3;

	public ArmPosReleaseCmd() {
		Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(-50.0/360 * Robot.SHOVEL_CIRCLE_CNT), exeTime);
		
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Robot.joint3Cmd.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
			
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (Robot.joint3Cmd.isFinished()) {
			//Robot.joint3Subsystem.disable();  //for arm maybe we could not disable the PID to keep arm at position
			System.out.println("Releae Job done");
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

