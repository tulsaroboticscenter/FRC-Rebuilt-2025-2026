// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ExampleSubsystem

/**
 * An example command written in Kotlin that uses an example subsystem.
 */
class ExampleKotlinCommand(private val subsystem: ExampleSubsystem) : Command() {

    init {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
        println("ExampleKotlinCommand initialized!")
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        // Command logic goes here
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
        if (interrupted) {
            println("ExampleKotlinCommand was interrupted")
        } else {
            println("ExampleKotlinCommand ended normally")
        }
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
