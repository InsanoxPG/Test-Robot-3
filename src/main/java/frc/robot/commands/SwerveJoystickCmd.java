// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCmd extends Command {

  private final Swerve swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  public SwerveJoystickCmd(Swerve swerveSubsystem,
                           Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
                           Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;
    this.xLimiter = new SlewRateLimiter(Constants.SwerveConstants.Limits.maxAccelerationMetersPerSecond);
    this.yLimiter = new SlewRateLimiter(Constants.SwerveConstants.Limits.maxAccelerationMetersPerSecond);
    this.turningLimiter = new SlewRateLimiter(Constants.SwerveConstants.Limits.maxAccelerationMetersPerSecond);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get joystick inputs
    double xSpd = xSpdFunction.get();
    double ySpd = ySpdFunction.get();
    double turningSpd = turningSpdFunction.get();
    // Apply deadbands
    xSpd = Math.abs(xSpd) > Constants.SwerveConstants.Limits.deadband ? xSpd : 0;
    ySpd = Math.abs(ySpd) > Constants.SwerveConstants.Limits.deadband ? ySpd : 0;
    turningSpd = Math.abs(turningSpd) > Constants.SwerveConstants.Limits.deadband ? turningSpd : 0;
    // Use acceleration limit to smooth out driving
    xSpd = xLimiter.calculate(xSpd) * Constants.SwerveConstants.Limits.Teleop.maxDriveMotorSpeedMetersPerSecond;
    ySpd = yLimiter.calculate(ySpd) * Constants.SwerveConstants.Limits.Teleop.maxDriveMotorSpeedMetersPerSecond;
    turningSpd = turningLimiter.calculate(turningSpd) * Constants.SwerveConstants.Limits.Teleop.maxTurnMotorSpeedRadiansPerSecond;
    // field oriented vs. robot oriented
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      // Field relative
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, turningSpd, swerveSubsystem.getRotation2d());
    } else {
      // Robot relative
      chassisSpeeds = new ChassisSpeeds(xSpd, ySpd, turningSpd);
    }
    // Chassis speeds -> module states
    SwerveModuleState[] moduleStates = Constants.SwerveConstants.Dimensions.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    // Set module states to each module
    swerveSubsystem.setModules(moduleStates);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
