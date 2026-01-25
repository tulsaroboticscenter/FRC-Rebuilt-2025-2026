// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command

/**
 * A simple "Hello World" autonomous command that prints messages every second.
 * This command runs for 5 seconds to demonstrate autonomous functionality.
 */
class HelloWorldAuto : Command() {

    private val timer = Timer()
    private var lastPrintTime = 0.0
    private var messageCount = 0

    init {
        // This command doesn't require any subsystems
    }

    override fun initialize() {
        timer.restart()
        lastPrintTime = 0.0
        messageCount = 0
        println("========================================")
        println("HelloWorldAuto STARTED!")
        println("This is a Kotlin autonomous command running on the RoboRIO")
        println("========================================")
    }

    override fun execute() {
        val currentTime = timer.get()

        // Print a message every second
        if (currentTime - lastPrintTime >= 1.0) {
            messageCount++
            println("Auto Message #$messageCount - Time: ${String.format("%.1f", currentTime)}s - Hello from Kotlin!")
            lastPrintTime = currentTime
        }
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        println("========================================")
        if (interrupted) {
            println("HelloWorldAuto was INTERRUPTED after ${String.format("%.1f", timer.get())}s")
        } else {
            println("HelloWorldAuto COMPLETED after ${String.format("%.1f", timer.get())}s")
        }
        println("Total messages sent: $messageCount")
        println("========================================")
    }

    override fun isFinished(): Boolean {
        // Run for 5 seconds
        return timer.hasElapsed(5.0)
    }
}
