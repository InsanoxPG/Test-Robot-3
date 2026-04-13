// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CustomSwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.IDs.FLdriveMotorID,
    SwerveConstants.IDs.FLturnMotorID, 
    SwerveConstants.IDs.FLencoderID, 
    SwerveConstants.Configs.FLencoderConfig, 
    SwerveConstants.Configs.FLdriveMotorConfig, 
    SwerveConstants.Configs.FLturnMotorConfig,
    0, 0, 0, 0);

  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.IDs.FRdriveMotorID,
    SwerveConstants.IDs.FRturnMotorID, 
    SwerveConstants.IDs.FRencoderID, 
    SwerveConstants.Configs.FRencoderConfig, 
    SwerveConstants.Configs.FRdriveMotorConfig, 
    SwerveConstants.Configs.FRturnMotorConfig,
    0, 0, 0, 0);

  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.IDs.BLdriveMotorID,
    SwerveConstants.IDs.BLturnMotorID, 
    SwerveConstants.IDs.BLencoderID, 
    SwerveConstants.Configs.BLencoderConfig, 
    SwerveConstants.Configs.BLdriveMotorConfig, 
    SwerveConstants.Configs.BLturnMotorConfig,
    0, 0, 0, 0);
  
  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.IDs.BRdriveMotorID,
    SwerveConstants.IDs.BRturnMotorID, 
    SwerveConstants.IDs.BRencoderID, 
    SwerveConstants.Configs.BRencoderConfig, 
    SwerveConstants.Configs.BRdriveMotorConfig, 
    SwerveConstants.Configs.BRturnMotorConfig,
    0, 0, 0, 0);
  
  private Pigeon2 gyro = new Pigeon2(SwerveConstants.IDs.gyroID);
  
  public Swerve() {
    // reset gyro in another thread after 1 second of delay
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.reset();
      } catch (Exception e) {
      }
    }).start();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getRotation2d().getDegrees(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());  
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModules(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, CustomSwerveModuleConstants.maxDriveMotorSpeed);
    frontLeft.setState(desiredStates[0]);
    frontRight.setState(desiredStates[1]);
    backLeft.setState(desiredStates[2]);
    backRight.setState(desiredStates[3]);
  }

  @Override
  public void periodic() {
    DogLog.log("Robot heading(degrees)", getHeading());
  }
}
