// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  static Shooter m_Instance = null;
  private static CANSparkMax shooterTopMotor = new CANSparkMax(Constants.ArmConstants.shooterTopMotor, MotorType.kBrushless);
  private static CANSparkMax shooterBottomMotor = new CANSparkMax(Constants.ArmConstants.shooterBottomMotor, MotorType.kBrushless);
  
  /** Creates a new ShooterAndIntake. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter() {
    shooterTopMotor.set(-.8);
    shooterBottomMotor.set(-.8);
  }

  public void zeroShooter() {
    shooterTopMotor.set(0);
    shooterBottomMotor.set(0);
  }

  public static Shooter getInstance() {
    if (m_Instance == null) {
      synchronized (Shooter.class) {
        if (m_Instance == null) {
          m_Instance = new Shooter();
        }
      }
    }
    return m_Instance;
  }
}
