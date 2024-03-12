// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  /** Creates a new LiftSubsystem. */

private final CANSparkMax m_lift = new CANSparkMax(72, MotorType.kBrushless);

  public LiftSubsystem() {
    m_lift.setIdleMode(IdleMode.kBrake);
    m_lift.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
      SmartDashboard.putNumber("Lift Speed", m_lift.get());
    }
  }

  public void set(double liftSpeed) {
    m_lift.set(liftSpeed);
  }

  public void stop() {
    m_lift.set(0);
  }

}
