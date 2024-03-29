// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_intakeDrive = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_Conveyor = new CANSparkMax(8, MotorType.kBrushless);
  public final DigitalInput distanceSensor = new DigitalInput(4);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // m_intakeDrive.restoreFactoryDefaults();
    m_intakeDrive.setIdleMode(IdleMode.kCoast);
    m_intakeDrive.setInverted(false);

    m_Conveyor.setIdleMode(IdleMode.kBrake);
    m_Conveyor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
      SmartDashboard.putNumber("Intake Speed", m_intakeDrive.get());
      SmartDashboard.putBoolean("distanceSensor", distanceSensor.get());
    }
  }

  public void set(double intakeSpeed) {
    m_intakeDrive.set(intakeSpeed);
    m_Conveyor.set(-intakeSpeed);
  }

  public void stop() {
    m_intakeDrive.set(0.0);
    m_Conveyor.set(0);
  }
}

