// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterLiftSubsystem extends SubsystemBase {
  /** Creates a new ShooterLiftSubsystem. */
  private final CANSparkMax m_shooterLift = new CANSparkMax(15, MotorType.kBrushless);
  private final AnalogPotentiometer sLiftPot = new AnalogPotentiometer(0, 27, 2.375);
  public ShooterLiftSubsystem() {
    m_shooterLift.setIdleMode(IdleMode.kBrake);
    m_shooterLift.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
    SmartDashboard.putNumber("Shooter Lift Speed", m_shooterLift.get());
    SmartDashboard.putNumber("S.Lift Potentiometer", sLiftPot.get());
    }
  }

  public void set(double sLiftSpeed) {
    m_shooterLift.set(sLiftSpeed);
  }

  public void stop() {
    m_shooterLift.set(0);
  }

  public double getPosition(){
  return sLiftPot.get();
  }

}
