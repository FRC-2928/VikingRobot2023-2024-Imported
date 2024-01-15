package frc.robot.commands.DrivetrainCommands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnTime extends Command {
	private final double duration;
	private final double rotationalSpeed;
	private final Drivetrain drivetrain;

	private long startTime;

	public TurnTime(double speed, double time, Drivetrain drive) {
		this.rotationalSpeed = speed;
		this.duration = time * 1000;
		this.drivetrain = drive;
		this.addRequirements(drive);
	}

	@Override
	public void initialize() {
		this.startTime = System.currentTimeMillis();
		this.drivetrain.halt();
	}

	@Override
	public void execute() {
		this.drivetrain.diffDrive.arcadeDrive(0, this.rotationalSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() - this.startTime) >= this.duration;
	}
}
