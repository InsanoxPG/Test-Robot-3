// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import dev.doglog.DogLog;
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
  private final CANcoderConfiguration encoderConfig;
  // config for motors
  private final TalonFXConfiguration driveMotorConfig;
  private final TalonFXConfiguration turnMotorConfig;
  // pid
  private final PIDController turningPidController;

  public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, CANcoderConfiguration encoderConfig,
                      TalonFXConfiguration driveMotorConfig, TalonFXConfiguration turnMotorConfig) {
    // initialize motors                
    driveMotor = new TalonFX(driveMotorID);
    turnMotor = new TalonFX(turnMotorID);
    // configurate motors
    this.driveMotorConfig = driveMotorConfig;
    this.turnMotorConfig = turnMotorConfig;
    // apply configs
    driveMotor.getConfigurator().apply(driveMotorConfig);
    turnMotor.getConfigurator().apply(turnMotorConfig);
    // initialize encoder                        
    encoder = new CANcoder(encoderID);
    this.encoderConfig = encoderConfig;
    encoder.getConfigurator().apply(encoderConfig);
    // pid
    turningPidController = new PIDController(CustomSwerveModuleConstants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double getDrivePosition() {
    return toRadians(encoder.getPosition().getValueAsDouble()); // convert to radians
  }

  public double getTurningPosition() {
    return toRadians(encoder.getPosition().getValueAsDouble()); // convert to radians
  }

  public double getDriveVelocity() {
    return toRadians(encoder.getVelocity().getValueAsDouble()); // in radians/sec
  }

  public double getTurningVelocity() {
    return toRadians(encoder.getVelocity().getValueAsDouble()); // in rads/sec
  }

  public double getAbsoluteEncoderRad() {
    double pos = toRadians(encoder.getAbsolutePosition().getValueAsDouble()); // in radians
    pos -= toRadians(encoderConfig.MagnetSensor.MagnetOffset);
    if (encoderConfig.MagnetSensor.SensorDirection == SensorDirectionValue.Clockwise_Positive) { // multiply by -1 if reversed
        pos *= 1.0;
    } else if (encoderConfig.MagnetSensor.SensorDirection == SensorDirectionValue.CounterClockwise_Positive) {
        pos *= -1.0;
    }
    return pos;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getTurningPosition()));
  }

  public void setState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
        stop();
        return;
    }
    state.optimize(Rotation2d.fromRadians(getTurningPosition()));
    driveMotor.set(state.speedMetersPerSecond / CustomSwerveModuleConstants.maxMotorSpeed); 
    turnMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    DogLog.log("Swerve Module State as a string", state.toString());
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }

  private double toRadians(double number) {
    return number * 2 * Math.PI;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
