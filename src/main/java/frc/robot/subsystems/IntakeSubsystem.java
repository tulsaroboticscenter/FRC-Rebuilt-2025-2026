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
  private final SparkMax m_motor = new SparkMax(IntakeConstants.kMotorCanId, MotorType.kBrushless);

  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(IntakeConstants.kMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void run(double speed) {
    m_motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
