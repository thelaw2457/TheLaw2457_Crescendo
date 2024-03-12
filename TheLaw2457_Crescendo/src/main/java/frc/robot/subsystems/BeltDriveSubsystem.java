// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeltDriveSubsystem extends SubsystemBase {

  //private final CANSparkMax m_Conveyor = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkFlex m_Roller = new CANSparkFlex(14, MotorType.kBrushless);
  /** Creates a new BeltDriveSubsystem. */
  public BeltDriveSubsystem() {
//m_Conveyor.setIdleMode(IdleMode.kCoast);
//m_Conveyor.setInverted(false);

m_Roller.setIdleMode(IdleMode.kCoast);
m_Roller.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Constants.SpeedConstants.isTunable) {
      //SmartDashboard.putNumber("Speed", m_Conveyor.get());
      SmartDashboard.putNumber("Roller Speed", m_Roller.get());
    }
  }

  public void set(double conveyorSpeed) {
    //m_Conveyor.set(conveyorSpeed);
    m_Roller.set(conveyorSpeed);
  }

  public void stop() {
    m_Roller.set(0);
  }
}
