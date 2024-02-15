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
  private final CANSparkFlex m_shooterDrive = new CANSparkFlex(15, MotorType.kBrushless);
  public ShooterSubsystem() {
    m_shooterDrive.setIdleMode(IdleMode.kBrake);
    m_shooterDrive.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
      SmartDashboard.putNumber("Speed", m_shooterDrive.get());
    }
  }

  public void set(double intakeSpeed) {
    m_shooterDrive.set(intakeSpeed);
  }

  public void stop() {
    m_shooterDrive.set(0.0);
  }
}
