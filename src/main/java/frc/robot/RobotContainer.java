// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/maxSwerve"));
    private final IntakeSubsystem intake = new IntakeSubsystem();

    // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        //Set the default auto (do nothing)
        autoChooser.setDefaultOption("Do Nothing", Commands.none());

        //Add a simple auto option to have the robot drive forward for 1 second then stop
        autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

        //Put the autoChooser on the SmartDashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // Right trigger runs intake and indexer forward
        driverXbox.rightTrigger(0.1).whileTrue(
            Commands.run(() -> intake.run(IntakeConstants.kMaxOutput), intake)
        ).onFalse(
            Commands.runOnce(() -> intake.stop(), intake)
        );

        // Left trigger runs intake and indexer in reverse
        driverXbox.leftTrigger(0.1).whileTrue(
            Commands.run(() -> intake.run(-IntakeConstants.kMaxOutput), intake)
        ).onFalse(
            Commands.runOnce(() -> intake.stop(), intake)
        );

        // Left bumper toggles indexer direction with vibration feedback
        driverXbox.leftBumper().onTrue(
            Commands.runOnce(() -> {
                intake.toggleIndexerDirection();
                // Vibrate left side for forward (not inverted), right side for inverted
                if (intake.isIndexerInverted()) {
                    driverXbox.getHID().setRumble(RumbleType.kRightRumble, 1.0);
                } else {
                    driverXbox.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
                }
            }).andThen(
                Commands.waitSeconds(0.2)
            ).andThen(
                Commands.runOnce(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.0))
            )
        );

        // Start button resets gyro so current heading becomes forward.
        driverXbox.start().onTrue(
            Commands.runOnce(drivebase::zeroGyroWithAlliance, drivebase)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
        return autoChooser.getSelected();
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
