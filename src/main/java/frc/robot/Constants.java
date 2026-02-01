// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class CustomSwerveModuleConstants {
    public static final double kPTurning = 0.1;
    public static final double maxMotorSpeed = 1; // meters/sec
  }

  public static class SwerveConstants {
    public static final int FLdriveMotorID = 0;
    public static final int FRdriveMotorID = 1;
    public static final int BLdriveMotorID = 2;
    public static final int BRdriveMotorID = 3;

    public static final int FLturnMotorID = 4;
    public static final int FRturnMotorID = 5;
    public static final int BLturnMotorID = 6;
    public static final int BRturnMotorID = 7;

    public static final int FLencoderID = 8;
    public static final int FRencoderID = 9;
    public static final int BLencoderID = 10;
    public static final int BRencoderID = 11;
    
    public static final TalonFXConfiguration FLdriveMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration FRdriveMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration BLdriveMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration BRdriveMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public static final TalonFXConfiguration FLturnMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration FRturnMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration BLturnMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration BRturnMotorConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public static final CANcoderConfiguration FLencoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0).withSensorDirection(null));
    public static final CANcoderConfiguration FRencoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0).withSensorDirection(null));
    public static final CANcoderConfiguration BLencoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0).withSensorDirection(null));
    public static final CANcoderConfiguration BRencoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0).withSensorDirection(null));

    public static final int gyroID = 12;
  }
}
