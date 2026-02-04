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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);
  private final SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorCanId, MotorType.kBrushless);

  private boolean m_indexerInverted = false;

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
    double clampedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    m_intakeMotor.set(clampedSpeed);
    m_indexerMotor.set(m_indexerInverted ? -clampedSpeed : clampedSpeed);
  }

  public void stop() {
    m_intakeMotor.stopMotor();
    m_indexerMotor.stopMotor();
  }

  public void toggleIndexerDirection() {
    m_indexerInverted = !m_indexerInverted;
  }

  public boolean isIndexerInverted() {
    return m_indexerInverted;
  }
}
