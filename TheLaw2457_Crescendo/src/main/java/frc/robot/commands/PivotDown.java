// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotDown extends Command {
  /** Creates a new PivotDown. */

  private PivotSubsystem PIVOT_SUBSYSTEM;
  private double pivotSpeed;
  

  public PivotDown(PivotSubsystem pivot, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.PIVOT_SUBSYSTEM = pivot;
    this.pivotSpeed = speed;

    addRequirements(PIVOT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (PIVOT_SUBSYSTEM.getPosition() <= .17) {
      PIVOT_SUBSYSTEM.set(0);
    } else
        PIVOT_SUBSYSTEM.set(pivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PIVOT_SUBSYSTEM.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
