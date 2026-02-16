// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.spec.PBEKeySpec;

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

  // Motor Definition
  private final SparkMax m_launchMotor = new SparkMax(IntakeConstants.kLaunchMotorCanId, MotorType.kBrushless);
  private final SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorCanId, MotorType.kBrushless);
  private final SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

  private final SlewRateLimiter m_outputLimiter =
      new SlewRateLimiter(IntakeConstants.kOutputRampRatePerSecond);

  private boolean m_indexerInverted = false;
  private double m_launchTargetOutput = 0.0;
  private double m_intakeTargetOutput = 0.0;

  private boolean isFlywheelRunning = false;

  public IntakeSubsystem() {

    // Launcher Configuration
    SparkMaxConfig launchConfig = new SparkMaxConfig();
    launchConfig
        .inverted(IntakeConstants.kLaunchMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_launchMotor.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Indexer Configuration
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    indexerConfig
        .inverted(IntakeConstants.kIndexerMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Intake Configuration
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    launchConfig
        .inverted(IntakeConstants.kIntakeMotorInverted)
        .smartCurrentLimit(IntakeConstants.kCurrentLimitAmps)
        .idleMode(IdleMode.kCoast);
    m_intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runLauncher(double speed) 
  {
    m_launchTargetOutput = MathUtil.clamp(speed, -1.0, 1.0);
  }

  public void runIntake(double speed)
  {
    m_intakeTargetOutput = MathUtil.clamp(speed, -1.0, 1.0);
  }

  public void stopIntake()
  {
    m_intakeTargetOutput = 0;
  }

  public void stopLaunch() {
    m_launchTargetOutput = 0.0;
  }

  public void toggleFlywheel()
  {
    isFlywheelRunning = !isFlywheelRunning;
  }

  public void toggleIndexerDirection() {
    m_indexerInverted = !m_indexerInverted;
  }

  public boolean isIndexerInverted() {
    return m_indexerInverted;
  }

  @Override
  public void periodic() {
    double rampedLaunchOutput = m_outputLimiter.calculate(m_launchTargetOutput);
    double rampedIntakeOutput = m_outputLimiter.calculate(m_intakeTargetOutput);
    m_launchMotor.set(isFlywheelRunning ? 0.65 : 0);
    m_indexerMotor.set(isFlywheelRunning ? -rampedIntakeOutput : rampedIntakeOutput);
    m_intakeMotor.set(-MathUtil.clamp(rampedLaunchOutput + rampedIntakeOutput, 0, 1));
  }
}
