// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class Slurp extends Command {

  private IntakeSubsystem INTAKE_SUBSYSTEM;
  private double intakeSpeed;

  /** Creates a new CubeSlurp. */
  public Slurp(IntakeSubsystem intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.INTAKE_SUBSYSTEM = intake;
    this.intakeSpeed = speed;

    addRequirements(INTAKE_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (INTAKE_SUBSYSTEM.distanceSensor.get()) {
      INTAKE_SUBSYSTEM.set(intakeSpeed);
    } else {
      INTAKE_SUBSYSTEM.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    INTAKE_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

