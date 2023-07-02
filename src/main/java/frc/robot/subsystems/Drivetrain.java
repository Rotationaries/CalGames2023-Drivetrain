// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase{

  public final Translation2d m_frontLeftLocation = new Translation2d(0.3429, 0.3429); //meters
  public final Translation2d m_frontRightLocation = new Translation2d(0.3429, -0.3429);
  public final Translation2d m_backLeftLocation = new Translation2d(-0.3429, 0.3429);
  public final Translation2d m_backRightLocation = new Translation2d(-0.3429, -0.3429);
  
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.FLDMChannel, DriveConstants.FLTMChannel, 
    DriveConstants.FLTEChannel, DriveConstants.FLTEOffsetDegrees);
  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.FRDMChannel, DriveConstants.FRTMChannel, 
    DriveConstants.FRTEChannel, DriveConstants.FRTEOffsetDegrees);
  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.BLDMChannel, DriveConstants.BLTMChannel, 
    DriveConstants.BLTEChannel, DriveConstants.BLTEOffsetDegrees);
  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.BRDMChannel, DriveConstants.BRTMChannel, 
    DriveConstants.BRTEChannel, DriveConstants.BRTEOffsetDegrees);
  

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          ahrs.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    ahrs.reset();
   // SmartDashboard.putNumber("Velocity Error", m_frontLeft.getVelocityError());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    
    SmartDashboard.putNumber("FL Speed", m_frontLeft.getModuleVelocity());
    SmartDashboard.putNumber("FL D S", m_frontLeft.getDesiredVelocity());
    SmartDashboard.putNumber("FL Rotation", m_frontLeft.getModuleAngle());
    SmartDashboard.putNumber("FL D R U", swerveModuleStates[0].angle.getRadians());
    SmartDashboard.putNumber("FL D R", m_frontLeft.getDesiredAngle());

    SmartDashboard.putNumber("FR Speed", m_frontRight.getModuleVelocity());
    SmartDashboard.putNumber("FR D S", m_frontRight.getDesiredVelocity());
    SmartDashboard.putNumber("FR Rotation", m_frontRight.getModuleAngle());
    SmartDashboard.putNumber("FR D R U", swerveModuleStates[1].angle.getRadians());
    SmartDashboard.putNumber("FR D R", m_frontRight.getDesiredAngle());

    SmartDashboard.putNumber("BL Speed", m_backLeft.getModuleVelocity());
    SmartDashboard.putNumber("BL D S", m_backLeft.getDesiredVelocity());
    SmartDashboard.putNumber("BL Rotation", m_backLeft.getModuleAngle());
    SmartDashboard.putNumber("BL D R U", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("BL D R", m_backLeft.getDesiredAngle());

    SmartDashboard.putNumber("BR Speed", m_backRight.getModuleVelocity());
    SmartDashboard.putNumber("BR D S", m_backRight.getDesiredVelocity());
    SmartDashboard.putNumber("BR Rotation", m_backRight.getModuleAngle());
    SmartDashboard.putNumber("BR D R U", swerveModuleStates[3].angle.getRadians());
    SmartDashboard.putNumber("BR D R", m_backRight.getDesiredAngle());

    // System.out.println(swerveModuleStates[3]);
 // System.out.println("frontleft current Rotation: " + m_frontLeft.getStateRotation(swerveModuleStates[0]));
   // System.out.println("frontleft current state: " + m_frontLeft.getState().toString() );
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        ahrs.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void printEncoder() {
    //System.out.println(m_frontLeft.getVelocityError());
  }

  @Override
  public void periodic(){
    //System.out.println(m_frontLeft.getModuleVelocity());
  }

}

