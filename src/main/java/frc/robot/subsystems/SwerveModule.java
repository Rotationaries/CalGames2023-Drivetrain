// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase{
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANCoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(SwerveConstants.PIDp, SwerveConstants.PIDi, SwerveConstants.PIDd);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveConstants.ProfiledPIDp,
          SwerveConstants.ProfiledPIDi,
          SwerveConstants.ProfiledPIDd,
          new TrapezoidProfile.Constraints(
              SwerveConstants.kModuleMaxAngularVelocity, SwerveConstants.kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.DriveKs, SwerveConstants.DriveKv);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.TurnKs, SwerveConstants.TurnKv);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */

  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = new CANCoder(turningEncoderChannel);

    // Distance per pulse
    m_driveEncoder.setPositionConversionFactor(2 * Math.PI * SwerveConstants.kWheelRadius / SwerveConstants.kEncoderResolution);

    // Radians per pulse
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / SwerveConstants.kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public SwerveModuleState getState() {

    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_moduleAngleRadians));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  public SwerveModulePosition getPosition() {

    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;

    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_moduleAngleRadians));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  
  public void setDesiredState(SwerveModuleState desiredState) {

    double m_moduleAngleRadians = m_turningEncoder.getAbsolutePosition() * 2 * Math.PI / 360;


    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocityConversionFactor(), state.speedMetersPerSecond);
    

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(m_moduleAngleRadians, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
   // m_driveMotor.setVoltage(2);

   // System.out.println(turnOutput + turnFeedforward);
   // m_turningMotor.setVoltage(1);
   // System.out.println("Drive Velocity: " + driveOutput + "Turn Velocity: " + turnOutput);
  }
}
