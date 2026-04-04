package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HailMaryIntakeSubsystem extends SubsystemBase {

    private static final int kLeftMotorId = 54;
    private static final int kRightMotorId = 55;

    private static final double kIntakeSpeed = 1.0; // 0.0 – 1.0

    private final TalonFX leftMotor = new TalonFX(kLeftMotorId);
    private final TalonFX rightMotor = new TalonFX(kRightMotorId);

    private final DutyCycleOut driveRequest = new DutyCycleOut(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public HailMaryIntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(config);

        // Right side inverted so both sides pull inward
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);
    }

    public void runIntake() {
        leftMotor.setControl(driveRequest.withOutput(kIntakeSpeed));
        rightMotor.setControl(driveRequest.withOutput(kIntakeSpeed));
    }

    public void reverseIntake() {
        leftMotor.setControl(driveRequest.withOutput(-kIntakeSpeed));
        rightMotor.setControl(driveRequest.withOutput(-kIntakeSpeed));
    }

    public void stopIntake() {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
    }

    /** Command: run intake while active, stop on end. */
    public Command runIntakeCommand() {
        return startEnd(this::runIntake, this::stopIntake).withName("HailMaryRunIntake");
    }

    /** Instant command: start intake forward (for use in autos). */
    public Command startIntakeCommand() {
        return runOnce(this::runIntake).withName("HailMaryStartIntake");
    }

    /** Instant command: stop intake (for use in autos). */
    public Command stopIntakeCommand() {
        return runOnce(this::stopIntake).withName("HailMaryStopIntake");
    }

    /** Command: reverse intake while active, stop on end. */
    public Command reverseIntakeCommand() {
        return startEnd(this::reverseIntake, this::stopIntake).withName("HailMaryReverseIntake");
    }
}