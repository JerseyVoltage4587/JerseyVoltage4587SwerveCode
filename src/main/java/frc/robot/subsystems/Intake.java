// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  static Intake m_Instance = null;
  private static TalonSRX intakeMotor = new TalonSRX(Constants.ArmConstants.intakeMotor);

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void forwardIntake() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -.6);
  }

  public void backwardIntake() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, .6);
  }

  public void zeroIntakeMotor() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public static Intake getInstance() {
    if (m_Instance == null) {
      synchronized (Intake.class) {
        if (m_Instance == null) {
          m_Instance = new Intake();
        }
      }
    }
    return m_Instance;
  }
}
