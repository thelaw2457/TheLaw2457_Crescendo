// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkFlex m_bottomShooter = new CANSparkFlex(18, MotorType.kBrushless);
  private final CANSparkFlex m_topShooter = new CANSparkFlex(17, MotorType.kBrushless);
  public ShooterSubsystem() {
    m_bottomShooter.setIdleMode(IdleMode.kCoast);
    m_bottomShooter.setInverted(false);

    m_topShooter.setIdleMode(IdleMode.kCoast);
    m_topShooter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
      SmartDashboard.putNumber("Speed", m_bottomShooter.get());
      SmartDashboard.putNumber("Speed", m_topShooter.get());
    }
  }

  public void set(double intakeSpeed) {
    m_bottomShooter.set(intakeSpeed);
    m_topShooter.set(intakeSpeed);
  }

  public void stop() {
    m_bottomShooter.set(0.0);
    m_topShooter.set(0);
  }
}
