package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private static final int kLeftMotorId  = 43;
    private static final int kRightMotorId = 44;

    // Target velocity in rotations per second (~3000 RPM) — adjustable via D-pad
    private double targetVelocityRPS = 52.0;

    // PID / feedforward gains (tune with SysId or on-robot testing)
    private static final double kP = 0.1;   // V per RPS of error
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kS = 0.05;  // V to overcome static friction
    private static final double kV = 0.12;  // V per RPS (≈ 12 V / 100 RPS free speed)

    private final TalonFX leftMotor  = new TalonFX(kLeftMotorId);
    private final TalonFX rightMotor = new TalonFX(kRightMotorId);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut neutralRequest = new NeutralOut();

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

        // Left motor spins counter-clockwise (from front of robot)
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(config);

        // Right motor spins clockwise to eject in the same direction
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);
    }

    /** Adjust the target velocity by {@code deltaRPS} rotations per second. */
    public void adjustTargetRPS(double deltaRPS) {
        targetVelocityRPS += deltaRPS;
    }

    /** Spin both wheels at the target velocity. */
    public void runShooter() {
        leftMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
        rightMotor.setControl(velocityRequest.withVelocity(targetVelocityRPS));
    }

    /** Coast both wheels to a stop. */
    public void stopShooter() {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
    }

    /** Returns true when both wheels are within 2 RPS of target. */
    public boolean atSpeed() {
        double leftVel  = leftMotor.getVelocity().getValueAsDouble();
        double rightVel = rightMotor.getVelocity().getValueAsDouble();
        return Math.abs(leftVel - targetVelocityRPS) < 2.0
            && Math.abs(rightVel - targetVelocityRPS) < 2.0;
    }

    /** Command: run shooter while active, stop on end. Bind to right trigger. */
    public Command runShooterCommand() {
        return startEnd(this::runShooter, this::stopShooter).withName("RunShooter");
    }

    /** Instant command: start shooter (for use in autos). */
    public Command startShooterCommand() {
        return runOnce(this::runShooter).withName("StartShooter");
    }

    /** Instant command: stop shooter (for use in autos). */
    public Command stopShooterCommand() {
        return runOnce(this::stopShooter).withName("StopShooter");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/LeftVelocityRPS",  leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/RightVelocityRPS", rightMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TargetRPS", targetVelocityRPS);
        SmartDashboard.putBoolean("Shooter/AtSpeed", atSpeed());
    }
}