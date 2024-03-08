// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule[] modules;

  private final SwerveDriveOdometry swerveOdometry;

  private final AHRS gyro;

  private final Timer timer;

  public SwerveSubsystem() {
    gyro = new AHRS();
    zeroGyro();

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw(), getPositions());

    timer = new Timer();
  }

  public void printAngles() {
    System.out.println("Mod 0: " + modules[0].getCanCoder());
    System.out.println("Mod 1: " + modules[1].getCanCoder());
    System.out.println("Mod 2: " + modules[2].getCanCoder());
    System.out.println("Mod 3: " + modules[3].getCanCoder());
  }


  /** 
   * This is called a command factory method, and these methods help reduce the
   * number of files in the command folder, increasing readability and reducing
   * boilerplate. 
   * 
   * Double suppliers are just any function that returns a double.
   */
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative, boolean isOpenLoop, double seconds) {
    return run(() -> {

      // Grabbing input from suppliers.
      double forwardBack = forwardBackAxis.getAsDouble()*8;
      double leftRight = leftRightAxis.getAsDouble()*8;
      double rotation = rotationAxis.getAsDouble()*10;

      // Adding deadzone.
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
        : new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    }).withName("SwerveDriveCommand");
  }

  public void drive(final double forwardBack, final double leftRight, final double rotation) {
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw()));
    setModuleStates(states, true);
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public Command autoDrive(double forwardBackAxis, double leftRightAxis, double rotationAxis, boolean isFieldRelative, boolean isOpenLoop, double seconds) {
    if(seconds != 0){
      if(timer.get() < seconds){
        return run(() -> {
          // Grabbing input from suppliers.
          double forwardBack = forwardBackAxis;
          double leftRight = leftRightAxis;
          double rotation = rotationAxis;

          // Adding deadzone.
          forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
          leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
          rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

          // Get desired module states.
          ChassisSpeeds chassisSpeeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
            : new ChassisSpeeds(forwardBack, leftRight, rotation);

          SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

          setModuleStates(states, isOpenLoop);
        }).withName("SwerveDriveCommand");
      }
    }
    return run(() -> {

    // Grabbing input from suppliers.
    double forwardBack = forwardBackAxis;
    double leftRight = leftRightAxis;
    double rotation = rotationAxis;

    // Adding deadzone.
    forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
    leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
    rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

    // Get desired module states.
    ChassisSpeeds chassisSpeeds = isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
      : new ChassisSpeeds(forwardBack, leftRight, rotation);

    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(states, isOpenLoop);
    }).withName("SwerveDriveCommand");
}

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
    }
  }

  /** To be used by auto. Use the drive method during teleop. */
  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }

    return currentStates;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getPosition();
      SmartDashboard.putNumber("mod " + i + " distance", modules[i].getPosition().distanceMeters);
      SmartDashboard.putNumber("mod " + i + " angle", modules[i].getPosition().angle.getDegrees());
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    SmartDashboard.putNumber("gyroAngle", getYaw().getDegrees());
    SmartDashboard.putNumber("gyroPitch", getPitch());
  }

  public void setXStance(boolean enabled) {
    if (enabled) {
        
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    for (SwerveModule module : modules) {
      builder.addStringProperty(
        String.format("Module %d", module.moduleNumber),
        () -> {
          SwerveModuleState state = module.getState();
          return String.format("%6.2fm/s %6.3fdeg", state.speedMetersPerSecond, state.angle.getDegrees());
        },
        null);

        builder.addDoubleProperty(
          String.format("Cancoder %d", module.moduleNumber),
          () -> module.getCanCoder(),
          null);

          
        builder.addDoubleProperty(
          String.format("Angle %d", module.moduleNumber),
          () -> module.getAngle().getDegrees(),
          null);
    }
  }
}
