package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

    private static final int kFeederMotorId = 40;

    private static final double kFeederSpeed = 0.7; // 0.0 – 1.0

    private final SparkMax feederMotor = new SparkMax(kFeederMotorId, MotorType.kBrushless);

    public FeederSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runFeeder() {
        feederMotor.set(kFeederSpeed);
    }

    public void stopFeeder() {
        feederMotor.set(0);
    }

    /**
     * Command: run feeder while active, stop on end. Bind to square button.
     */
    public Command runFeederCommand() {
        return startEnd(this::runFeeder, this::stopFeeder).withName("RunFeeder");
    }
}