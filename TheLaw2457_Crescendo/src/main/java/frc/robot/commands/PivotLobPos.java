// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLobPos extends Command {
  /** Creates a new PivotLobPos. */

  private PivotSubsystem PIVOT_SUBSYSTEM;
  private PIDController pidController;

  public PivotLobPos(PivotSubsystem pivot, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.PIVOT_SUBSYSTEM = pivot;
    this.pidController = new PIDController(PivotConstants.PIVOT_KP, PivotConstants.PIVOT_KI, PivotConstants.PIVOT_KD);
    pidController.setSetpoint(setpoint);

    addRequirements(PIVOT_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pidController.calculate(PIVOT_SUBSYSTEM.getPosition());
    PIVOT_SUBSYSTEM.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PIVOT_SUBSYSTEM.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
