// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BeltDriveSubsystem;

public class BeltForward extends Command {

  private BeltDriveSubsystem BELT_DRIVE;
  private double beltSpeed;

  /** Creates a new BeltForward. */
  public BeltForward(BeltDriveSubsystem beltdrive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.BELT_DRIVE = beltdrive;
    this.beltSpeed = speed;

    addRequirements(BELT_DRIVE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BELT_DRIVE.set(beltSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BELT_DRIVE.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
