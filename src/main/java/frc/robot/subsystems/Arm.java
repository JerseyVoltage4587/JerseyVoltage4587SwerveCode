// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  
  static Arm m_Instance = null;
  private static CANSparkMax armMotorLeft = new CANSparkMax(Constants.ArmConstants.armMotorLeft, MotorType.kBrushless);
  private static CANSparkMax armMotorRight = new CANSparkMax(Constants.ArmConstants.armMotorRight, MotorType.kBrushless);

  /** Creates a new Arm. */
  public Arm() {
    armMotorLeft.restoreFactoryDefaults();
    armMotorRight.restoreFactoryDefaults();
    armMotorLeft.setInverted(true);
    armMotorLeft.setIdleMode(IdleMode.kBrake);
    armMotorRight.setIdleMode(IdleMode.kBrake);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void armUp() {
    armMotorLeft.set(-.6);
    armMotorRight.set(-.6);
  }

  public void armDown() {
    armMotorLeft.set(.6);
    armMotorRight.set(.6);
  }

  public void zeroArmMotors() {
    armMotorLeft.set(0);
    armMotorRight.set(0);
  }

  public static Arm getInstance() {
    if (m_Instance == null) {
      synchronized (Arm.class) {
        if (m_Instance == null) {
          m_Instance = new Arm();
        }
      }
    }
    return m_Instance;
  }

}
