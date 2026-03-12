// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 2048;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final Encoder m_driveEncoder;
  // private final Encoder m_turningEncoder;
  private final CANcoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController[] pidController = {
    // Front Left
    new ProfiledPIDController(
          0.009,// 0.208,
          0,
          0, // 0.01,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)),
    // Front Right
    new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)),
    // Back Left
    new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)),
    // Back Right
    new ProfiledPIDController(
          0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))
  };

  // Gains are for example purposes only - must be determined for your own robot!
  // private final ProfiledPIDController m_turningPIDController =
  //     new ProfiledPIDController(
  //         0.15,
  //         0,
  //         0,
  //         new TrapezoidProfile.Constraints(
  //             kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)); //kModuleMaxAngularAcceleration

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

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
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderDeviceId) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);


    m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    // m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
    m_turningEncoder = new CANcoder(turningEncoderDeviceId);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    pidController[0].setTolerance(Math.PI/4.0);
    pidController[1].setTolerance(Math.PI/8.0);
    pidController[2].setTolerance(Math.PI/8.0);
    pidController[3].setTolerance(Math.PI/8.0);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    pidController[0].enableContinuousInput(-Math.PI, Math.PI);
    pidController[1].enableContinuousInput(-Math.PI, Math.PI);
    pidController[2].enableContinuousInput(-Math.PI, Math.PI);
    pidController[3].enableContinuousInput(-Math.PI, Math.PI);

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getRate(), new Rotation2d(getEncoderValue() ));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(getEncoderValue()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, int pidControllerKey, String name) {
    var encoderRotation = new Rotation2d(getEncoderValueDegrees() );

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.

    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);


    SmartDashboard.putNumber(name + " encoder", getEncoderValue()); //  encoder value
    SmartDashboard.putNumber(name + " rads", getEncoderValue());    // graph encoder value

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        pidController[pidControllerKey].calculate(
            getEncoderValue(), -1); //SET TO desiredState.angle.getRadians() -PI to PI
            //TEMPORARY HARD CODE FOR TESTING

    final double turnFeedforward =
        m_turnFeedforward.calculate(pidController[pidControllerKey].getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    // m_turningMotor.setVoltage(turnOutput + 0); //0 was turnFeedfoward
    m_turningMotor.set(turnOutput + 0); //0 was turnFeedfoward
    //set or setVoltage?


    
    SmartDashboard.putNumber(name, pidController[pidControllerKey].getPositionError());
    SmartDashboard.putNumber(name + "v", pidController[pidControllerKey].getVelocityError());
    SmartDashboard.putNumber(name + " setpoint", pidController[pidControllerKey].getSetpoint().position);
    // SmartDashboard.putNumber(name + " setpoint", desiredState.angle.getRadians());
    // SmartDashboard.putNumber(name, pidController[pidControllerKey].getSetpoint().velocity);
  }

  private double getEncoderValue(){
    return m_turningEncoder.getAbsolutePosition().getValueAsDouble() * Math.PI * 2; 
  }
  private double getEncoderValueDegrees(){
    return m_turningEncoder.getAbsolutePosition().getValueAsDouble() * 360 + 180; 
  }
}