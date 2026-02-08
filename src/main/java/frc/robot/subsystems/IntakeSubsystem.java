// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorCanId, MotorType.kBrushless);
  private final SlewRateLimiter m_outputLimiter =
      new SlewRateLimiter(IntakeConstants.kOutputRampRatePerSecond);

  private boolean m_indexerInverted = false;
  private double m_targetOutput = 0.0;

  public IntakeSubsystem() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
        .inverted(IntakeConstants.kIntakeMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig
        .inverted(IntakeConstants.kIndexerMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void run(double speed) {
    m_targetOutput = MathUtil.clamp(speed, -1.0, 1.0);
  }

  public void stop() {
    m_targetOutput = 0.0;
  }

  public void toggleIndexerDirection() {
    m_indexerInverted = !m_indexerInverted;
  }

  public boolean isIndexerInverted() {
    return m_indexerInverted;
  }

  @Override
  public void periodic() {
    double rampedOutput = m_outputLimiter.calculate(m_targetOutput);
    m_intakeMotor.set(rampedOutput);
    m_indexerMotor.set(m_indexerInverted ? -rampedOutput : rampedOutput);
  }
}
