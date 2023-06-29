// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    //FL-Front Left, DM-Driving Motor, TM-Turning Motor, DE-Driving Encoder, TE-Turning Encoder
    public static final int FLDMChannel = 12;
    public static final int FLTMChannel = 11;
    public static final int FLTEChannel = 13;

    public static final int FRDMChannel = 3;
    public static final int FRTMChannel = 2;
    public static final int FRTEChannel = 4;

    public static final int BLDMChannel = 9;
    public static final int BLTMChannel = 8;
    public static final int BLTEChannel = 10;
    public static final int BRDMChannel = 6;
    public static final int BRTMChannel = 5;
    public static final int BRTEChannel = 7;
  }
  
  public static class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kModuleMaxAngularVelocity = DriveConstants.kMaxAngularSpeed;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    // drive PID constants
    public static final double PIDp = 0.6;

    public static final double PIDi = 0;
    public static final double PIDd = 0;

    // turning PID constants 
    public static final double ProfiledPIDp = 0.05;
    public static final double ProfiledPIDi = 0;
    public static final double ProfiledPIDd = 0;

    public static final double DriveKs = 0;
    public static final double DriveKv = 0;

    public static final double TurnKs = 0;
    public static final double TurnKv = 0;
  }
}
