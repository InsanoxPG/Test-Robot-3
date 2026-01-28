// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CustomSwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
  // Swerve module motors
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  // encoder (absolute + relative)
  private final CANcoder encoder;
  private final double encoderOffsetRot;
  private final boolean encoderReversed;
  // config for motors
  private final TalonFXConfiguration driveMotorConfig;
  private final TalonFXConfiguration turnMotorConfig;
  // pid
  private final PIDController turningPidController;

  public SwerveModule(int driveMotorID, int turnMotorID, InvertedValue driveMotorReversed, InvertedValue turnMotorReversed,
                      int encoderID, double encoderOffsetRot, boolean encoderReversed, TalonFXConfiguration driveMotorConfig, TalonFXConfiguration turnMotorConfig) {
    // initialize motors                
    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    // configurate motors
    this.driveMotorConfig = driveMotorConfig;
    this.turnMotorConfig = turnMotorConfig;
    driveMotorConfig.MotorOutput.withInverted(driveMotorReversed);
    turnMotorConfig.MotorOutput.withInverted(turnMotorReversed);
    // apply configs
    driveMotor.getConfigurator().apply(driveMotorConfig);
    turnMotor.getConfigurator().apply(turnMotorConfig);
    // initialize relative encoders                        
    encoder = new CANcoder(encoderID);
    this.encoderOffsetRot = encoderOffsetRot;
    this.encoderReversed = encoderReversed;
    // pid
    turningPidController = new PIDController(CustomSwerveModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition() {
    return encoder.getPosition().getValueAsDouble(); // in rotations
  }

  public double getTurningPosition() {
    return encoder.getPosition().getValueAsDouble(); // in rotations
  }

  public double getDriveVelocity() {
    return encoder.getVelocity().getValueAsDouble(); // in rotations/sec
  }

  public double getTurningVelocity() {
    return encoder.getVelocity().getValueAsDouble(); // in rot/sec
  }

  public double getAbsoluteEncoderRot() {
    double pos = encoder.getAbsolutePosition().getValueAsDouble(); // in rot
    pos -= encoderOffsetRot;
    pos *= (encoderReversed ? -1.0 : 1.0); // multiply by -1 if reversed
    return pos;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRotations(getTurningPosition()));
  }

  public void setState(SwerveModuleState state) {
    state.optimize(Rotation2d.fromRotations(getTurningPosition()));
    driveMotor.set(state.speedMetersPerSecond / CustomSwerveModuleConstants.maxMotorSpeed); // why does this work? if state.speed is 100 meter/sec and maxmotorspeed is 5 m/s, then the result will still be 20 m/s, over the limit
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
