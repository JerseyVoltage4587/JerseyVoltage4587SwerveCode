// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }

  //Front Left Swerve Module Motors
  public static final int frontLeftTurnMotor = 6;
  public static final int frontLeftDriveMotor = 7;
  //Front Right Swerve Module Motors
  public static final int frontRightTurnMotor = 0;
  public static final int frontRightDriveMotor = 1;
  //Back Left Swerve Module Motors
  public static final int backLeftTurnMotor = 2;
  public static final int backLeftDriveMotor = 3;
  //Back Right Swerve Module Motors
  public static final int backRightTurnMotor = 4;
  public static final int backRightDriveMotor = 5;

  //Robot Width & Length in inches
  public static final int robotWidth = 29;
  public static final int robotLength = 29;

  //Max Speed of Robot
  public static final double maxSpeed = 4.5;
}
