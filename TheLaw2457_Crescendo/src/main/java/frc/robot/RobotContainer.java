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
import frc.robot.commands.LiftAscend;
import frc.robot.commands.LiftDescend;
import frc.robot.commands.PivotUp;
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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverControl = new Joystick(0);
  private final XboxController xboxController = new XboxController(1);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    configureSmartDashboard();
  }

  public void configureSmartDashboard() {
    SmartDashboard.putNumber("Angle", 5);
  }

  public void printWheelAngles() {
    swerveSubsystem.printAngles();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.drive(
      () -> -driverControl.getY(),
      () -> -driverControl.getX(),
      () -> -driverControl.getZ(),
      true,
      false,
      0
    ));

    //Field Orientation
    new JoystickButton(driverControl, 7).onTrue(swerveSubsystem.zeroGyroCommand());

    // Intake
      new JoystickButton(driverControl, 12).whileTrue(new IntakeForward(m_intakeSubsystem, SpeedConstants.INTAKE_FORWARD));
      new JoystickButton(driverControl, 9).whileTrue(new IntakeReverse(m_intakeSubsystem, SpeedConstants.INTAKE_REVERSE));

      // new JoystickButton(xboxController, 2).whileTrue(new IntakeForward(m_intakeSubsystem, SpeedConstants.INTAKE_FORWARD));
      // new JoystickButton(xboxController, 3).whileTrue(new IntakeReverse(m_intakeSubsystem, SpeedConstants.INTAKE_REVERSE));

    // Lift
      // new JoystickButton(xboxController, 2).whileTrue(new LiftAscend(m_liftSubsystem, SpeedConstants.LIFT_UP));
      // new JoystickButton(xboxController, 3).whileTrue(new LiftDescend(m_liftSubsystem, SpeedConstants.LIFT_DOWN));

    // Shooter
      //new JoystickButton(xboxController, 5).whileTrue(new Slurp(m_shooterSubsystem, SpeedConstants.SLURP_SPEED));
      //new JoystickButton(xboxController, 6).whileTrue(new Drool(m_shooterSubsystem, SpeedConstants.DROOL_SPEED));
      new JoystickButton(xboxController, 5).whileTrue(new Spit(m_shooterSubsystem, SpeedConstants.SPIT_SPEED));
      new JoystickButton(xboxController,6).whileTrue(new Spew(m_shooterSubsystem, SpeedConstants.SPEW_SPEED));

      //Shooter Pivot
      new JoystickButton(xboxController, 8).whileTrue(new PivotUp(m_pivotSubsystem, SpeedConstants.PIVOT_UP));
      new JoystickButton(xboxController, 7).whileTrue(new PivotDown(m_pivotSubsystem, SpeedConstants.PIVOT_DOWN));


    //BeltDrive
      new JoystickButton(xboxController, 4).whileTrue(new BeltForward(m_beltDriveSubsystem, SpeedConstants.BELT_FORWARD));
      new JoystickButton(xboxController, 1).whileTrue(new BeltReverse(m_beltDriveSubsystem, SpeedConstants.BELT_REVERSE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
