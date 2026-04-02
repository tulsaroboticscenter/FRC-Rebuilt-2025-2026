// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private static final double kFullSpeedRumbleStrength = 0.5;
    private static final double kFullSpeedRumblePeriodSeconds = 1.0;
    private static final double kFullSpeedRumblePulseSeconds = 0.2;
    private static final double kTriggerPressedThreshold = 0.5;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(1.50).in(RadiansPerSecond); // 1.5 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private static final String kLimelightName = "limelight";

    // Field-centric request that locks heading toward a Limelight target (right bumper)
    private double aimKp = 0.04;
    private double angle = 6.0;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final BetterCommandPS5Controller driver = new BetterCommandPS5Controller(0);
    private final CommandXboxController manipulator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TestSubsystem testSubsystem = new TestSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final FeederSubsystem feeder = new FeederSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        testSubsystem.markInitialized();
        SmartDashboard.putNumber("AimAtTarget/kP", aimKp);
        SmartDashboard.putNumber("AimAtTarget/AngleTolerance", angle);
        configureBindings();

        NamedCommands.registerCommand("StartShooter", shooter.startShooterAtRPSCommand(75.0));
        NamedCommands.registerCommand("StopShooter",  shooter.stopShooterCommand());
        NamedCommands.registerCommand("StartFeeder",  feeder.startFeederCommand());
        NamedCommands.registerCommand("StopFeeder",   feeder.stopFeederCommand());
        NamedCommands.registerCommand("StartIntake",  intake.startIntakeCommand());
        NamedCommands.registerCommand("StopIntake",   intake.stopIntakeCommand());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.triangle().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Driver: feeder in/out on triggers.
        driver.R2(kTriggerPressedThreshold).whileTrue(feeder.runFeederCommand());
        driver.L2(kTriggerPressedThreshold).whileTrue(feeder.reverseFeederCommand());

        // R1: angle-only Limelight tracking — driver keeps full translational control
        driver.R1().whileTrue(
                drivetrain.applyRequest(() -> {
                    double tx = LimelightHelpers.getTX(kLimelightName);
                    boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
                    double steeringAdjust = hasTarget && Math.abs(tx) > angle
                            ? -(tx * aimKp) * MaxAngularRate
                            : -driver.getRightX() * MaxAngularRate;
                    return drive
                            .withVelocityX(-driver.getLeftY() * MaxSpeed)
                            .withVelocityY(-driver.getLeftX() * MaxSpeed)
                            .withRotationalRate(steeringAdjust);
                })
        );

        // Manipulator: shooter and intake controls.
        manipulator.rightTrigger(kTriggerPressedThreshold).whileTrue(shooter.runShooterCommand());
        manipulator.leftBumper().whileTrue(shooter.reverseShooterCommand());
        manipulator.leftTrigger(kTriggerPressedThreshold).whileTrue(intake.runIntakeCommand());
        manipulator.rightBumper().whileTrue(intake.reverseIntakeCommand());
        manipulator.povRight().onTrue(shooter.runOnce(() -> shooter.adjustTargetRPS(1)));
        manipulator.povLeft().onTrue(shooter.runOnce(() -> shooter.adjustTargetRPS(-1)));
        manipulator.povDown().onTrue(shooter.runOnce(shooter::toggleSpeedMode));

        // Feed Limelight MegaTag2 pose estimates into the drivetrain's pose estimator
//        drivetrain.run(() -> drivetrain.updateVisionPose(kLimelightName));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void stopAllMechanisms() {
        shooter.stopShooter();
        intake.stopIntake();
        feeder.stopFeeder();
        manipulator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }

    public void logDriverInputs() {
        SmartDashboard.putBoolean("Driver/ControllerConnected", driver.getHID().isConnected());
        SmartDashboard.putBoolean("Manipulator/ControllerConnected", manipulator.getHID().isConnected());

        boolean shouldRumble = DriverStation.isTeleopEnabled() && shooter.isFullSpeedMode();
        double cyclePosition = Timer.getFPGATimestamp() % kFullSpeedRumblePeriodSeconds;
        double rumbleValue = shouldRumble && cyclePosition < kFullSpeedRumblePulseSeconds
                ? kFullSpeedRumbleStrength
                : 0.0;
        manipulator.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
    }
}
