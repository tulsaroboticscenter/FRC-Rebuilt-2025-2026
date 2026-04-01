// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TestSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
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

    //    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

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

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.triangle().whileTrue(drivetrain.applyRequest(() -> brake));
//        joystick.b().whileTrue(drivetrain.applyRequest(() ->
//            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
//        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
//        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
//        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
//        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
//        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // D-pad up/down: adjust shooter target RPS by 1
        joystick.povUp().onTrue(shooter.runOnce(() -> shooter.adjustTargetRPS(1)));
        joystick.povDown().onTrue(shooter.runOnce(() -> shooter.adjustTargetRPS(-1)));

        // Reset the field-centric heading on L1 press.
        joystick.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // R1: angle-only Limelight tracking — driver keeps full translational control
        joystick.R1().whileTrue(
                drivetrain.applyRequest(() -> {
                    double tx = LimelightHelpers.getTX(kLimelightName);
                    boolean hasTarget = LimelightHelpers.getTV(kLimelightName);
                    double steeringAdjust = hasTarget && Math.abs(tx) > angle
                            ? -(tx * aimKp) * MaxAngularRate
                            : -joystick.getRightX() * MaxAngularRate;
                    return drive
                            .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                            .withRotationalRate(steeringAdjust);
                })
        );

        // Right trigger: run shooter while held
        joystick.R2().whileTrue(shooter.runShooterCommand());

        // Cross (X): toggle intake on/off
        joystick.cross().toggleOnTrue(intake.runIntakeCommand());

        // Circle: reverse intake while held
        joystick.circle().whileTrue(intake.reverseIntakeCommand());

        // Square: run feeder while held
        joystick.square().whileTrue(feeder.runFeederCommand());

        // Feed Limelight MegaTag2 pose estimates into the drivetrain's pose estimator
//        drivetrain.run(() -> drivetrain.updateVisionPose(kLimelightName));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
