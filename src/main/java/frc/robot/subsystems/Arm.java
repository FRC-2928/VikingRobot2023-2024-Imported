package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
// import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

// import com.ctre.phoenix.motorcontrol.*;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANBusIDs;

public class Arm extends SubsystemBase {
	public final TalonFX motorLead = new TalonFX(Constants.CANBusIDs.ArmTalonLeader);
	public final TalonFX motorFollower = new TalonFX(Constants.CANBusIDs.ArmTalonFollower);

	public final CANcoder encoder = new CANcoder(Constants.CANBusIDs.ArmEncoder);

	// True: Unlocked
	// False: Locked
	private final Solenoid lockingPiston = new Solenoid(PneumaticsModuleType.REVPH, Constants.PneumaticIDs.armLock);

	public Arm() {
		for(final TalonFX fx : new TalonFX[] { this.motorLead, this.motorFollower}) {
      		fx.getConfigurator().apply(new TalonFXConfiguration());     
		}
			//NEW
			var configuration = new TalonFXConfiguration();
			configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

			configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;


			this.motorLead.getConfigurator().apply(configuration);
			this.motorFollower.getConfigurator().apply(configuration);

			this.motorFollower.setControl(new Follower(motorLead.getDeviceID(), false));
			// end NEW  

			//OLD
			// Reset settings for safety
			// fx.configFactoryDefault();

			// Sets voltage compensation to 10, used for percent output
			// SKIP fx.configVoltageCompSaturation(10);
			// SKIP fx.enableVoltageCompensation(true);

			// Setting just in case
			// SKIP fx.configNominalOutputForward(0);
			// SKIP fx.configNominalOutputReverse(0);
			// SKIP fx.configPeakOutputForward(1);
			// SKIP fx.configPeakOutputReverse(-1);

			// SKIP fx.configOpenloopRamp(0.1);

			// Setting deadband(area required to start moving the motor) to 1%
			// SKIP fx.configNeutralDeadband(0.01);

			// Set to brake mode, will brake the motor when no power is sent
			// DONE fx.setNeutralMode(NeutralMode.Brake);

			/**
			 * Setting input side current limit (amps)
			 * 45 continious, 80 peak, 30 millieseconds allowed at peak
			 * 40 amp breaker can support above 40 amps for a little bit
			 * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds
			 * should be fine
			 */
			// SKIP fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

			// Either using the integrated Falcon sensor or an external one, will change if
			// needed
			// DONE fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

			// SKIP fx.configRemoteFeedbackFilter(CANBusIDs.ArmEncoder, RemoteSensorSource.CANCoder, 0, 0);
			// TODO ?? fx.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
		//  END OLD   }

		// DONE this.motorFollower.setInverted(InvertType.FollowMaster);
		// DONE this.motorFollower.follow(this.motorLead);

		// TODO this.motorLead.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

		Telemetry.track("Arm Position", this::getPosition, false);
		// TODO Telemetry.track("Arm Limit", () -> this.motorLead.getSensorCollection().isRevLimitSwitchClosed() == 1, false);
	}

	public void halt() {
		this.lock(true);
	}

	public void control(double power) {
		if(this.pastTopLimit()) power = Math.max(power, 0.0);
		if(this.pastBottomLimit()) power = Math.min(power, 0.0);

		final double deadbandPower = MathUtil.applyDeadband(power, 0.25);

		this.lock(deadbandPower == 0);
		this.setPower(deadbandPower);
	}

	public void setPower(final double power) {
		// this.motorLead.set(ControlMode.PercentOutput, MathUtil.clamp(power, -0.5, 0.5));
		var voltsRequest = new DutyCycleOut(power / 12);
		this.motorLead.setControl(voltsRequest);
	}

	public void lock(final boolean shouldLock) {
		this.lockingPiston.set(!shouldLock);
		// this.motorLead.set(ControlMode.PercentOutput, 0.0); // just to be safe
		var voltsRequest = new DutyCycleOut(0);
		this.motorLead.setControl(voltsRequest);
	}

	public double getPosition() {
		return this.motorLead.getPosition().getValue();
		// 	return this.encoder.getAbsolutePosition();
	}

	private boolean pastTopLimit() {
		return this.getPosition() <= ArmConstants.homeAngleLimit;
	}

	private boolean pastBottomLimit() {
		return this.getPosition() >= ArmConstants.maxAngleLimit;
	}

	// Returns whether the arm is far enough out to limit speed - beyond pickup position
	public boolean armIsOut() {
		return (this.getPosition() > ArmConstants.lowPositionCone + 10);
	}

	@Override
	public void periodic() {
		if(this.pastBottomLimit() || this.pastTopLimit()) this.halt();

		// TODO if(this.motorLead.getSensorCollection().isRevLimitSwitchClosed() == 1) {
        //     //System.out.println(this.encoder.getAbsolutePosition());
		// 	this.encoder.configMagnetOffset(this.encoder.configGetMagnetOffset() - 
		// 	(this.encoder.getAbsolutePosition() - ArmConstants.armLimitSwitchOffset) );
		// }
	}
}
