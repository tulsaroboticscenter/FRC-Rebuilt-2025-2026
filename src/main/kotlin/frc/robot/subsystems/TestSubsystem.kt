package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

class TestSubsystem : SubsystemBase() {
    private var initialized = false

    fun markInitialized() {
        initialized = true
    }

    fun isInitialized(): Boolean {
        return initialized
    }
}
