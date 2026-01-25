// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.ExampleSubsystem

/**
 * A simple "Hello World" teleop command that prints messages every 3 seconds.
 * This command runs continuously during teleop to demonstrate teleoperated functionality.
 */
class HelloWorldTeleop(private val subsystem: ExampleSubsystem) : Command() {

    private val timer = Timer()
    private var lastPrintTime = 0.0
    private var messageCount = 0

    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        timer.restart()
        lastPrintTime = 0.0
        messageCount = 0
        println("========================================")
        println("HelloWorldTeleop STARTED!")
        println("This is a Kotlin teleop command running on the RoboRIO")
        println("Running continuously - disable robot to stop")
        println("========================================")
    }

    override fun execute() {
        val currentTime = timer.get()

        // Print a message every 3 seconds
        if (currentTime - lastPrintTime >= 3.0) {
            messageCount++
            println("Teleop Message #$messageCount - Time: ${String.format("%.1f", currentTime)}s - Robot is alive and running Kotlin!")
            lastPrintTime = currentTime
        }
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        println("========================================")
        if (interrupted) {
            println("HelloWorldTeleop was INTERRUPTED after ${String.format("%.1f", timer.get())}s")
        } else {
            println("HelloWorldTeleop ENDED after ${String.format("%.1f", timer.get())}s")
        }
        println("Total messages sent: $messageCount")
        println("========================================")
    }

    override fun isFinished(): Boolean {
        // Never finish - runs until robot is disabled
        return false
    }
}
