// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SimpleCSVLogger2194;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LeftSpark;

public class LeftSparkTest extends CommandBase {
  /** Creates a new MotorTest. */
  private static LeftSpark m_drive;
  private static double m_lSpeed;
  private static double m_startTime;
  private static boolean m_direction;
  private static boolean m_secondPass;
  private static double stepNumber;
  private static SimpleCSVLogger2194 m_logger = new SimpleCSVLogger2194();
  public String names = "Step,LPWM,LRate,LPos,RPWM,RRate ,RPos, LR-RR,LPWMRPWM, LeftGet, RightGet,LGetRget,Batt\n";
  public String units = "Number,Pct,Cts,Ctspersec,Pct,Cts,Ctspersec \n";

  public LeftSparkTest(LeftSpark drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.set(.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
