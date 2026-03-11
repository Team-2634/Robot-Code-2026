// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.AnalogGyro;
import com.studica.frc.AHRS;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 1.5; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 0,  0, 1, 0);
  private final SwerveModule m_frontRight = new SwerveModule(7, 6, 4, 5, 1);
  private final SwerveModule m_backLeft = new SwerveModule(4, 5,   8, 9, 4);
  private final SwerveModule m_backRight = new SwerveModule(3, 2,  12, 13, 2);

  //private final AnalogGyro m_gyro = new AnalogGyro(0);
  AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); 

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          navx.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    navx.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    
    ChassisSpeeds chassisSpeed;
    if(fieldRelative){
      chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d());
    } else {
      chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, rot);
    }

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeed, periodSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], "F_L_Steer");
    m_frontRight.setDesiredState(swerveModuleStates[1], "F_R_Steer");
    m_backLeft.setDesiredState(swerveModuleStates[2], "B_L_Steer");
    m_backRight.setDesiredState(swerveModuleStates[3], "B_R_Steer");

  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public void straightenWheels(double periodSeconds) {
    ChassisSpeeds wheelOrientationReset = new ChassisSpeeds(0,0,0);

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(wheelOrientationReset, periodSeconds));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0], "F_L_Steer");
    m_frontRight.setDesiredState(swerveModuleStates[1], "F_R_Steer");
    m_backLeft.setDesiredState(swerveModuleStates[2], "B_L_Steer");
    m_backRight.setDesiredState(swerveModuleStates[3], "B_R_Steer");
  
  }
}
