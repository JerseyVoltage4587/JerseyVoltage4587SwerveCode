// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.OI;
// import frc.robot.subsystems.SwerveSubsystem;

// public class SwerveDriveJoysticks extends Command {
//   /** Creates a new SwerveDriveJoysticks. */

//   SwerveSubsystem m_Swerve = SwerveSubsystem.getInstance();
//   OI m_OI = OI.getInstance();
//   Joystick m_joy;

//   public SwerveDriveJoysticks() {

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_Swerve);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("SwerveJoyInit");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
    
//     m_Swerve.SwerveDriveCommands(m_OI.j.getRawAxis(0), m_OI.j.getRawAxis(1), m_OI.j.getRawAxis(2));

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
