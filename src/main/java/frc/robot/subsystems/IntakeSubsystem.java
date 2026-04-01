package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

//    private static final int kLeftLeaderId = 50;
//    private static final int kLeftFollowerId = 51;
    private static final int kRightLeaderId = 52;
    private static final int kRightFollowerId = 53;

    private static final double kIntakeSpeed = 1.0; // 0.0 – 1.0

//    private final SparkFlex leftLeader = new SparkFlex(kLeftLeaderId, MotorType.kBrushless);
//    private final SparkFlex leftFollower = new SparkFlex(kLeftFollowerId, MotorType.kBrushless);
    private final SparkFlex rightLeader = new SparkFlex(kRightLeaderId, MotorType.kBrushless);
    private final SparkFlex rightFollower = new SparkFlex(kRightFollowerId, MotorType.kBrushless);

    public IntakeSubsystem() {
//        SparkFlexConfig leaderConfig = new SparkFlexConfig();
//        leaderConfig.smartCurrentLimit(60);
//
//         Left side
//        leftLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//
//        SparkFlexConfig leftFollowerConfig = new SparkFlexConfig();
//        leftFollowerConfig.follow(leftLeader, false); // same direction as left leader
//        leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Right side — inverted relative to left so both sides pull inward
        SparkFlexConfig rightLeaderConfig = new SparkFlexConfig();
        rightLeaderConfig.smartCurrentLimit(60);
        rightLeaderConfig.inverted(true);
        rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig rightFollowerConfig = new SparkFlexConfig();
        rightFollowerConfig.follow(rightLeader, false); // same direction as right leader
        rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake() {
//        leftLeader.set(kIntakeSpeed);
        rightLeader.set(kIntakeSpeed);
    }

    public void reverseIntake() {
//        leftLeader.set(-kIntakeSpeed);
        rightLeader.set(-kIntakeSpeed);
    }

    public void stopIntake() {
//        leftLeader.set(0);
        rightLeader.set(0);
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