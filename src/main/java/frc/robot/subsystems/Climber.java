// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  
  static Climber m_Instance = null;
  private static TalonSRX climberMotorLeft = new TalonSRX(Constants.ArmConstants.climberMotorLeft);
  private static TalonSRX climberMotorRight = new TalonSRX(Constants.ArmConstants.climberMotorRight);

  /** Creates a new Climber. */
  public Climber() {
    climberMotorLeft.configFactoryDefault();
    climberMotorRight.configFactoryDefault();
    climberMotorRight.setInverted(true);
    climberMotorLeft.setNeutralMode(NeutralMode.Brake);
    climberMotorRight.setNeutralMode(NeutralMode.Brake);
    climberMotorLeft.configNeutralDeadband(.03);
    climberMotorRight.configNeutralDeadband(.03);
    climberMotorLeft.configContinuousCurrentLimit(5, 30000);
    climberMotorRight.configContinuousCurrentLimit(5, 30000);
    climberMotorLeft.configPeakCurrentLimit(6, 100);
    climberMotorRight.configPeakCurrentLimit(6, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberUp() {
    if(!DriverStation.isFMSAttached() || DriverStation.getMatchTime() < 30) {
      climberMotorLeft.set(TalonSRXControlMode.PercentOutput, .4);
      climberMotorRight.set(TalonSRXControlMode.PercentOutput, .3);
    }
  }

  public void climberDown() {
    if(!DriverStation.isFMSAttached() || DriverStation.getMatchTime() < 30) {
      climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 1);
      climberMotorRight.set(TalonSRXControlMode.PercentOutput, 1);
    }
  }

  public void zeroClimberMotors() {
    climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0);
    climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void constantClimbers() {
    climberMotorLeft.set(TalonSRXControlMode.PercentOutput, 0.04);
    climberMotorRight.set(TalonSRXControlMode.PercentOutput, 0.04);
  }

  public static Climber getInstance() {
    if (m_Instance == null) {
      synchronized (Climber.class) {
        if (m_Instance == null) {
          m_Instance = new Climber();
        }
      }
    }
    return m_Instance;
  }
}
