package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private static final int kLeftTopMotorId = 50;
    private static final int kLeftBottomMotorId = 51;
    private static final int kRightTopMotorId = 52;
    private static final int kRightBottomMotorId = 53;

    private static final double kIntakeSpeed = 1.0; // 0.0 – 1.0

    private final SparkFlex leftTopMotor = new SparkFlex(kLeftTopMotorId, MotorType.kBrushless);
    private final SparkFlex leftBottomMotor = new SparkFlex(kLeftBottomMotorId, MotorType.kBrushless);
    private final SparkMax rightTopMotor = new SparkMax(kRightTopMotorId, MotorType.kBrushless);
    private final SparkFlex rightBottomMotor = new SparkFlex(kRightBottomMotorId, MotorType.kBrushless);

    public IntakeSubsystem() {
        SparkFlexConfig leftConfig = new SparkFlexConfig();
        leftConfig.smartCurrentLimit(30);
        leftConfig.idleMode(IdleMode.kCoast);

        leftTopMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftBottomMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right side — inverted relative to left so both sides pull inward
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.smartCurrentLimit(30);
        rightConfig.idleMode(IdleMode.kCoast);
        rightConfig.inverted(true);
        rightTopMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightBottomMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake() {
        leftTopMotor.set(kIntakeSpeed);
        leftBottomMotor.set(kIntakeSpeed);
        rightTopMotor.set(kIntakeSpeed);
        rightBottomMotor.set(kIntakeSpeed);
    }

    public void reverseIntake() {
        leftTopMotor.set(-kIntakeSpeed);
        leftBottomMotor.set(-kIntakeSpeed);
        rightTopMotor.set(-kIntakeSpeed);
        rightBottomMotor.set(-kIntakeSpeed);
    }

    public void stopIntake() {
        leftTopMotor.set(0);
        leftBottomMotor.set(0);
        rightTopMotor.set(0);
        rightBottomMotor.set(0);
    }

    /**
     * Command: run intake while active, stop on end. Bind to cross (X) button.
     */
    public Command runIntakeCommand() {
        return startEnd(this::runIntake, this::stopIntake).withName("RunIntake");
    }

    /** Instant command: start intake forward (for use in autos). */
    public Command startIntakeCommand() {
        return runOnce(this::runIntake).withName("StartIntake");
    }

    /** Instant command: stop intake (for use in autos). */
    public Command stopIntakeCommand() {
        return runOnce(this::stopIntake).withName("StopIntake");
    }

    /**
     * Command: reverse intake while active, stop on end. Bind to circle button.
     */
    public Command reverseIntakeCommand() {
        return startEnd(this::reverseIntake, this::stopIntake).withName("ReverseIntake");
    }
}