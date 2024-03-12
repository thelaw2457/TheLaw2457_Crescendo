// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax m_pivot = new CANSparkMax(16, MotorType.kBrushless);
  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    m_pivot.setIdleMode(IdleMode.kBrake);
    m_pivot.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(Constants.SpeedConstants.isTunable) {
      SmartDashboard.putNumber("Pivot Speed", m_pivot.get());
      SmartDashboard.putNumber("absEncoder Pos.", absEncoder.getAbsolutePosition());
    }
  }

public void set(double pivotSpeed) {
  m_pivot.set(pivotSpeed);
}

public void stop() {
  m_pivot.set(0);
}

}
