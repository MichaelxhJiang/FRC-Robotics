package org.usfirst.frc.team7176.robot.commands;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Encoder;
import org.usfirst.frc.team7176.robot.Robot;
import org.usfirst.frc.team7176.robot.RobotMap;
import org.usfirst.frc.team7176.robot.Robot.ArmPosition;
import org.usfirst.frc.team7176.robot.commands.Joint3Cmd;


public class ArmThrowCubeCmd  extends Command {
	private static final int RUN_TIME1 = 200;	//300ms down
	private static final int RUN_TIME2 = 100;	//100ms up
	private Encoder j3Encoder  = RobotMap.jointEncoder3;
	private int step = 0;
	private static final double SWITCH_ANGLE1 = 30.0;
	private static final double SWITCH_ANGLE2 = 120.0;
	private static final double SCALE_ANGLE1 = - 65.0;
	private static final double SCALE_ANGLE2 = 30.0;
	private double turnAngle1 = 0;
	private double turnAngle2 = 0;
	private int timer20msCnt = 0;
	public ArmThrowCubeCmd() {
	
	}
	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (Robot.armPosition == ArmPosition.SCALE) {
			turnAngle1 = SCALE_ANGLE1;
			turnAngle2 = SCALE_ANGLE2;
		}else {
			turnAngle1 = SWITCH_ANGLE1;
			turnAngle2 = SWITCH_ANGLE2;
		}
		Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(turnAngle1/360 * Robot.SHOVEL_CIRCLE_CNT), RUN_TIME1);
		Robot.joint3Cmd.start();
		step = 0;
		timer20msCnt = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		timer20msCnt ++;
		if ((step == 0) && timer20msCnt > (RUN_TIME1 /20)) {
			step = 1;
			Robot.joint3Cmd = new Joint3Cmd(j3Encoder.getRaw(), (int)(turnAngle2/360 * Robot.SHOVEL_CIRCLE_CNT), RUN_TIME2);	
			Robot.joint3Cmd.start();
		}
			
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if ((step == 1) && (Robot.joint3Cmd.isFinished())) {
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

