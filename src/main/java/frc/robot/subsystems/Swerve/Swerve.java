// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Swerve extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(
    SwerveConstants.FLdriveMotorID,
    SwerveConstants.FLturnMotorID, 
    SwerveConstants.FLencoderID, 
    SwerveConstants.FLencoderConfig, 
    SwerveConstants.FLdriveMotorConfig, 
    SwerveConstants.FLturnMotorConfig);

  private final SwerveModule frontRight = new SwerveModule(
    SwerveConstants.FRdriveMotorID,
    SwerveConstants.FRturnMotorID, 
    SwerveConstants.FRencoderID, 
    SwerveConstants.FRencoderConfig, 
    SwerveConstants.FRdriveMotorConfig, 
    SwerveConstants.FRturnMotorConfig);

  private final SwerveModule backLeft = new SwerveModule(
    SwerveConstants.BLdriveMotorID,
    SwerveConstants.BLturnMotorID, 
    SwerveConstants.BLencoderID, 
    SwerveConstants.BLencoderConfig, 
    SwerveConstants.BLdriveMotorConfig, 
    SwerveConstants.BLturnMotorConfig);
  
  private final SwerveModule backRight = new SwerveModule(
    SwerveConstants.BRdriveMotorID,
    SwerveConstants.BRturnMotorID, 
    SwerveConstants.BRencoderID, 
    SwerveConstants.BRencoderConfig, 
    SwerveConstants.BRdriveMotorConfig, 
    SwerveConstants.BRturnMotorConfig);
  
  private Pigeon2 gyro = new Pigeon2(SwerveConstants.gyroID);
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
