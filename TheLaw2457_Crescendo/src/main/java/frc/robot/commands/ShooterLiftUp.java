// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLiftSubsystem;

public class ShooterLiftUp extends Command {
  /** Creates a new ShooterLiftUp. */

private ShooterLiftSubsystem SLIFT_SUBSYSTEM;
private double sLiftSpeed;

  public ShooterLiftUp(ShooterLiftSubsystem sLift, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.SLIFT_SUBSYSTEM = sLift;
    this.sLiftSpeed = speed;

    addRequirements(SLIFT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SLIFT_SUBSYSTEM.set(sLiftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SLIFT_SUBSYSTEM.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
