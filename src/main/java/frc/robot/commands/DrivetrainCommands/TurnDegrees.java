package frc.robot.commands.DrivetrainCommands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnDegrees extends Command {
	private final Drivetrain drivetrain;
	private final double degrees;
	private final double speed;

	public TurnDegrees(double speed, double degrees, Drivetrain drive) {
		this.degrees = degrees;
		this.speed = speed;
		this.drivetrain = drive;
		this.addRequirements(drive);
	}

	@Override
	public void initialize() {
		this.drivetrain.halt();
		this.drivetrain.resetEncoders();
	}

	@Override
	public void execute() {
		this.drivetrain.diffDrive.arcadeDrive(0, this.speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.drivetrain.halt();
	}

	@Override
	public boolean isFinished() {
		/* Need to convert distance travelled to degrees. Sweetpants has a wheel
			placement distance of 0.7 meters = 700 mm - width of the wheel (20 mm) = 720 mm
			We then take into consideration the width of the tires.
		*/
		double metersPerDegree = Math.PI * DrivetrainConstants.trackWidthMeters / 360;
		// Compare distance travelled from start to distance based on degree turn
		return this.getAverageTurningDistance() >= (metersPerDegree * this.degrees);
	}

	private double getAverageTurningDistance() {
		double leftDistance = Math.abs(this.drivetrain.getLeftDistanceMeters());
		double rightDistance = Math.abs(this.drivetrain.getRightDistanceMeters());
		return (leftDistance + rightDistance) / 2.0;
	}
}
