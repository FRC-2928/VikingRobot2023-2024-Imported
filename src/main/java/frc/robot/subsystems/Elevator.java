package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
// import com.ctre.phoenix.motorcontrol.*;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
	// Fwd: Down
	// Rev: Up
	public final TalonFX motor = new TalonFX(Constants.CANBusIDs.ElevatorTalon);

	// True: Unlocked
	// False: Locked
	public final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.elevatorLock);

	// ------------ Initialization -----------------------------

	public Elevator() {
		this.configureMotors();
		this.lock(true);
	}

	public void configureMotors() {
		// Configure the motors
		for(TalonFX fx : new TalonFX[] { this.motor }) {
			// Reset settings for safety
			fx.getConfigurator().apply(new TalonFXConfiguration());     

		}

		// Home limit switch. Stop motor if this switch is triggered.
		this.motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
		

		// Top limit switch. Read as a digital input
		this.motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		this.motor.configForwardSoftLimitThreshold(ElevatorConstants.bottomSoftLimit);
		this.motor.configReverseSoftLimitThreshold(ElevatorConstants.topSoftLimit);

		this.motor.configForwardSoftLimitEnable(true);
		this.motor.configReverseSoftLimitEnable(true);

		this.motor.overrideSoftLimitsEnable(true);

		Telemetry.track("Elevator Position", this::getPosition, false);
		Telemetry.track("Elevator Home Limit", () -> this.limitHomeClosed(), false);
		Telemetry.track("Elevator Top Limit", () -> this.limitTopClosed(), false);
	}

	// --------------- Control Input ---------------------

	public void halt() {
		this.lock(true);
	}

	public void control(double power) {
		if(this.pastTopLimit()) power = Math.max(power, 0.0);
		if(this.pastBottomLimit()) power = Math.min(power, 0.0);

		power = MathUtil.applyDeadband(power, 0.25);

		this.lock(power == 0);

		if(power != 0) this.setPower(power - 0.05);
	}

	public void setPower(double power) {
		this.motor.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.2, 0.2));
	}

	public void lock(boolean shouldLock) {
		this.lockingPiston.set(!shouldLock);
		this.motor.set(ControlMode.PercentOutput, 0.0); // just to be safe
	}

	// ------------- System State -------------------

	public boolean limitTopClosed() {
		return this.motor.getSensorCollection().isRevLimitSwitchClosed() == 1;
	}

	public boolean limitHomeClosed() {
		return this.motor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
	}

	public double getPosition() {
		return this.motor.getSelectedSensorPosition();
	}

	public void overrideEncoderPosition(double ticks) {
		this.motor.setSelectedSensorPosition(ticks);
	}

	private boolean pastTopLimit() {
		return this.getPosition() <= ElevatorConstants.topSoftLimit;
	}

	private boolean pastBottomLimit() {
		return this.getPosition() >= ElevatorConstants.bottomSoftLimit;
	}

	// ------------- Process State -------------------

	@Override
	public void periodic() {
		if(this.limitTopClosed() || this.pastBottomLimit() || this.pastTopLimit()) this.halt();

		MechanismLigament2d mech = Robot.instance.robotContainer.mechElevatorExtension;
		mech.setLength(1);
	}
}
