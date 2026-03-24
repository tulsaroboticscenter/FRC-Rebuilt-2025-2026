// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TestSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MaxSpeed * 0.9);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MaxSpeed * 0.9);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private static final String kLimelightName = "limelight";

    // Field-centric request that locks heading toward a Limelight target (right bumper)
    private double aimKp = 0.04;
    private double angle = 6.0;
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TestSubsystem testSubsystem = new TestSubsystem();

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
                        drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                                .withVelocityY(yLimiter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

//        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
//        joystick.b().whileTrue(drivetrain.applyRequest(() ->
//            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
//        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // D-pad up/down: adjust aimAtTarget kP by 0.5
        joystick.povUp().onTrue(drivetrain.runOnce(() -> {
            aimKp += 0.01;
            SmartDashboard.putNumber("AimAtTarget/kP", aimKp);
        }));
        joystick.povDown().onTrue(drivetrain.runOnce(() -> {
            aimKp = Math.max(0, aimKp - 0.01);
            SmartDashboard.putNumber("AimAtTarget/kP", aimKp);
        }));

        // Triggers: adjust angle tolerance up/down by 0.5 degrees
        joystick.rightTrigger(0.5).onTrue(drivetrain.runOnce(() -> {
            angle += 0.5;
            SmartDashboard.putNumber("AimAtTarget/AngleTolerance", angle);
        }));
        joystick.leftTrigger(0.5).onTrue(drivetrain.runOnce(() -> {
            angle = Math.max(0, angle - 0.5);
            SmartDashboard.putNumber("AimAtTarget/AngleTolerance", angle);
        }));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Right bumper: angle-only Limelight tracking — driver keeps full translational control
        joystick.rightBumper().whileTrue(
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

        // Feed Limelight MegaTag2 pose estimates into the drivetrain's pose estimator
//        drivetrain.run(() -> drivetrain.updateVisionPose(kLimelightName));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
