// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.BeltForward;
import frc.robot.commands.BeltReverse;
import frc.robot.commands.Drool;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LiftAscend;
import frc.robot.commands.LiftDescend;
import frc.robot.commands.ShooterLiftDown;
import frc.robot.commands.ShooterLiftUp;
import frc.robot.commands.ShooterStop;
import frc.robot.commands.PivotUp;
import frc.robot.commands.RollerStop;
import frc.robot.commands.Slurp;
import frc.robot.commands.Spew;
import frc.robot.commands.Spit;
import frc.robot.commands.PivotUp;
import frc.robot.commands.PivotDown;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BeltDriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterLiftSubsystem;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final BeltDriveSubsystem m_beltDriveSubsystem = new BeltDriveSubsystem();
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem();
  private final ShooterLiftSubsystem m_sLiftSubsystem = new ShooterLiftSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverControl = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureSmartDashboard();
    configureNamedAutoCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public void configureNamedAutoCommands() {
    NamedCommands.registerCommand("Shoot",
        new InstantCommand(() -> m_shooterSubsystem.set(Constants.SpeedConstants.SPEW_SPEED))
            .andThen(new WaitCommand(1.0)));
    NamedCommands.registerCommand("Stop Shooter",
        new ShooterStop(m_shooterSubsystem, Constants.SpeedConstants.SHOOTER_STOP));
    NamedCommands.registerCommand("Roller",
        new InstantCommand(() -> m_beltDriveSubsystem.set(Constants.SpeedConstants.BELT_FORWARD))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> m_beltDriveSubsystem.set(Constants.SpeedConstants.ROLLER_STOP))));
    NamedCommands.registerCommand("End Roller",
        new RollerStop(m_beltDriveSubsystem, Constants.SpeedConstants.ROLLER_STOP));
    NamedCommands.registerCommand("Start Intake",
        new IntakeForward(m_intakeSubsystem, Constants.SpeedConstants.INTAKE_FORWARD));
    NamedCommands.registerCommand("End Intake",
        new IntakeStop(m_intakeSubsystem, Constants.SpeedConstants.INTAKE_STOP));
  }

  public void configureSmartDashboard() {

  }

  public void printWheelAngles() {
    swerveSubsystem.printAngles();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.drive(
        () -> -driverControl.getY(),
        () -> -driverControl.getX(),
        () -> -driverControl.getZ(),
        true,
        false,
        0));

    // Field Orientation
    new JoystickButton(driverControl, 7).onTrue(swerveSubsystem.zeroGyroCommand());

    // Intake
    new JoystickButton(driverControl, 12)
        .whileTrue(new IntakeForward(m_intakeSubsystem, SpeedConstants.INTAKE_FORWARD));
    // new JoystickButton(driverControl, 11).whileTrue(new
    // IntakeForward(m_intakeSubsystem, SpeedConstants.INTAKE_FORWARD));
    new JoystickButton(driverControl, 9).whileTrue(new IntakeReverse(m_intakeSubsystem, SpeedConstants.INTAKE_REVERSE));
    // new JoystickButton(driverControl, 10).whileTrue(new
    // IntakeReverse(m_intakeSubsystem, SpeedConstants.INTAKE_REVERSE));

    // new JoystickButton(xboxController, 2).whileTrue(new
    // IntakeForward(m_intakeSubsystem, SpeedConstants.INTAKE_FORWARD));
    // new JoystickButton(xboxController, 3).whileTrue(new
    // IntakeReverse(m_intakeSubsystem, SpeedConstants.INTAKE_REVERSE));

    // Shooter Lift
    new JoystickButton(xboxController, 4).whileTrue(new ShooterLiftUp(m_sLiftSubsystem, SpeedConstants.SLIFT_UP));
    new JoystickButton(xboxController, 1).whileTrue(new ShooterLiftDown(m_sLiftSubsystem, SpeedConstants.SLIFT_DOWN));

    // Lift
    new JoystickButton(xboxController, 2).whileTrue(new LiftAscend(m_liftSubsystem, SpeedConstants.LIFT_UP));
    new JoystickButton(xboxController, 3).whileTrue(new LiftDescend(m_liftSubsystem, SpeedConstants.LIFT_DOWN));

    // Shooter
    // new JoystickButton(xboxController, 5).whileTrue(new Slurp(m_shooterSubsystem,
    // SpeedConstants.SLURP_SPEED));
    // new JoystickButton(xboxController, 6).whileTrue(new Drool(m_shooterSubsystem,
    // SpeedConstants.DROOL_SPEED));
    new JoystickButton(xboxController, 5).whileTrue(new Spit(m_shooterSubsystem, SpeedConstants.SPIT_SPEED));
    new JoystickButton(xboxController, 6).whileTrue(new Spew(m_shooterSubsystem, SpeedConstants.SPEW_SPEED));
    new JoystickButton(driverControl, 4).whileTrue(new Drool(m_shooterSubsystem, SpeedConstants.DROOL_SPEED));

    // Shooter Pivot
    new JoystickButton(xboxController, 8).whileTrue(new PivotUp(m_pivotSubsystem, SpeedConstants.PIVOT_UP));
    new JoystickButton(xboxController, 7).whileTrue(new PivotDown(m_pivotSubsystem, SpeedConstants.PIVOT_DOWN));

    // BeltDrive
    new JoystickButton(driverControl, 1).whileTrue(new BeltForward(m_beltDriveSubsystem, SpeedConstants.BELT_FORWARD));
    new JoystickButton(driverControl, 2).whileTrue(new BeltReverse(m_beltDriveSubsystem, SpeedConstants.BELT_REVERSE));
    new JoystickButton(driverControl, 3).whileTrue(new BeltForward(m_beltDriveSubsystem, SpeedConstants.FASTER_BELT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
