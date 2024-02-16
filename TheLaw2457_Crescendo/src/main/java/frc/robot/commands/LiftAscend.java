// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.Constants.SpeedConstants;

public class LiftAscend extends Command {

  private LiftSubsystem LIFT_SUBSYSTEM;
  private double liftSpeed;

  /** Creates a new LiftAscend. */
  public LiftAscend(LiftSubsystem lift, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.LIFT_SUBSYSTEM = lift;
    this.liftSpeed = speed;

    addRequirements(LIFT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
