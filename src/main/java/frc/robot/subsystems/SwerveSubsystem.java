// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.OI;


//Module contains a Drive Motor, Turn Motor, Absolute Encoder
class SwerveModule {
  
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private SwerveModuleState currentState;
  private Encoder driveEncoder;
  private Encoder turnEncoder;
  
  //Constructor
  public SwerveModule(int driveMotorPort, int turnMotorPort) {
    System.out.println("SwerveModule constructor");
    driveMotor = new TalonFX(driveMotorPort);
    turnMotor = new TalonFX(turnMotorPort);
    currentState = new SwerveModuleState();
    driveEncoder = new Encoder(0, 1);
    turnEncoder = new Encoder(2,3);
  }

  public SwerveModuleState getState() {
    return currentState;
  }

  public void setState(SwerveModuleState desiredState) {
    currentState = desiredState;
  }

  public void setSpeed() {
    DutyCycleOut speedOutput = new DutyCycleOut(currentState.speedMetersPerSecond / Constants.maxSpeed);
    driveMotor.setControl(speedOutput);
  }

  public void setAngle() {
    double angleOutput = (currentState.angle.getDegrees() / 360);
    turnMotor.setPosition(angleOutput);
  }

}

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  
  //List of Swerve Modules
  // double vx, vy, theta;
  static SwerveSubsystem m_Instance = null;
  CommandXboxController controller;
  SwerveModule frontLeftModule = new SwerveModule(Constants.frontLeftDriveMotor, Constants.frontLeftTurnMotor);
  SwerveModule frontRightModule = new SwerveModule(Constants.frontRightDriveMotor, Constants.frontRightTurnMotor);
  SwerveModule backLeftModule = new SwerveModule(Constants.backLeftDriveMotor, Constants.backLeftTurnMotor);
  SwerveModule backRightModule = new SwerveModule(Constants.backRightDriveMotor, Constants.backRightTurnMotor);

  //Robot Width & Length in meters
  double robotWidthMeters = Units.inchesToMeters(Constants.robotWidth);
  double robotLengthMeters = Units.inchesToMeters(Constants.robotLength);
  
  //Position of Swerve Modules relative to center of robot
  Translation2d frontLeftLocation = new Translation2d(robotLengthMeters / 2, robotWidthMeters / 2);
  Translation2d frontRightLocation = new Translation2d(robotLengthMeters / 2, -robotWidthMeters / 2);
  Translation2d backLeftLocation = new Translation2d(-robotLengthMeters / 2, robotWidthMeters / 2);
  Translation2d backRightLocation = new Translation2d(-robotLengthMeters / 2, -robotWidthMeters / 2);

  //Kinematics object: ChassisSpeeds -> SwerveModuleStates
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
  );


  Joystick m_controller;

  //Constructor
  public SwerveSubsystem(Joystick m_driverController) {
    // System.out.println("SwerveSystem Constructor");
    System.out.println(kinematics);
    m_controller = m_driverController;
  }


public void setChassisSpeeds(ChassisSpeeds desiredSpeeds) {
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeeds);

    frontLeftModule.setState(newStates[0]);
    frontRightModule.setState(newStates[1]);
    backLeftModule.setState(newStates[2]);
    backRightModule.setState(newStates[3]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    ChassisSpeeds newSpeeds = new ChassisSpeeds(
      m_controller.getRawAxis(0),
      m_controller.getRawAxis(1),
      m_controller.getRawAxis(2)
    );

    setChassisSpeeds(newSpeeds);

    frontLeftModule.setSpeed();
    frontLeftModule.setAngle();
    frontRightModule.setSpeed();
    frontRightModule.setAngle();
    backLeftModule.setSpeed();
    backLeftModule.setAngle();
    backRightModule.setSpeed();
    backRightModule.setAngle();

    //FrontLeft, FrontRight, BackLeft, BackRight
    double loggingState[] = {
      frontLeftModule.getState().angle.getDegrees(), frontLeftModule.getState().speedMetersPerSecond,
      frontRightModule.getState().angle.getDegrees(), frontRightModule.getState().speedMetersPerSecond,
      backLeftModule.getState().angle.getDegrees(), backLeftModule.getState().speedMetersPerSecond,
      backRightModule.getState().angle.getDegrees(), backRightModule.getState().speedMetersPerSecond
    };
    
    //Sending data to the SmartDashboard
    SmartDashboard.putNumberArray("Swerve Modules States", loggingState);


  }



  
  // public static SwerveSubsystem getInstance() {
  //   if (m_Instance == null) {
  //     synchronized (SwerveSubsystem.class) {
  //       if (m_Instance == null) {
  //         m_Instance = new SwerveSubsystem();
  //       }
  //     }
  //   }
  //   return m_Instance;
  // }

}
