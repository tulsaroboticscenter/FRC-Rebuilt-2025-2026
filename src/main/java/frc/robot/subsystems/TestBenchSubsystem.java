package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NewMotorConstants;

public class TestBenchSubsystem extends SubsystemBase {

    private final SparkMax m_motor = new SparkMax(NewMotorConstants.kCanId, MotorType.kBrushless);

    private boolean m_running = false;

    public void toggle() {
        m_running = !m_running;
    }

    public boolean isRunning() {
        return m_running;
    }

    @Override
    public void periodic() {
        m_motor.set(m_running ? NewMotorConstants.kSpeed : 0.0);
    }
}