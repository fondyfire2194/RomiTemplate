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

public class MotorCompRun extends CommandBase {
  /** Creates a new MotorTest. */
  private static Drivetrain m_drive;
  private static double m_lSpeed;
  private static double m_startTime;
  private static boolean m_direction;
  private static boolean m_secondPass;
  private static double stepNumber;
  private static SimpleCSVLogger2194 m_logger = new SimpleCSVLogger2194();
  public String names = "Step,LPWM,LRate,LPos,RPWM,RRate ,RPos, LR-RR,LPWMRPWM, LeftGet, RightGet,LGetRget,Batt\n";
  public String units = "Number,Pct,Cts,Ctspersec,Pct0,0,Cts,Ctspersec \n";

  private double stepTime = .1;// seconds must be even number
  private double rampIncTime = .02;
  private int n;
  private double numberPassesPerStep;

  public MotorCompRun(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    n = 0;
    numberPassesPerStep = stepTime * 1000 / 20;// ms/scan time ms
    SmartDashboard.putNumber("NPPS", numberPassesPerStep);
    m_startTime = Timer.getFPGATimestamp();
    m_direction = false;
    m_secondPass = false;

    String name = Integer.toString((int) m_startTime) + "Run.txt";
    String names = "Step,LGet,LPWM,LSpeed,LRate,LFRate,LPos,RGet,RPWM,RSpeed,RRate,RFRate,RPos,Yaw,Batt \n";
    String units = "Val,PCT,PCT,PCT,CPS,CPS,CNTS,PCT,PCT,PCT,CPS,CPS,CNTS,DEG<VOLTS \n";
    m_logger.init(name, names, units);
    stepNumber = 0;
    n = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Timer.getFPGATimestamp() - m_startTime > stepTime) {
      if (!m_direction || m_secondPass) {
        m_lSpeed += rampIncTime;
        n++;

      } else {
        m_lSpeed -= rampIncTime;
      }

      m_startTime = Timer.getFPGATimestamp();
      stepNumber++;

    }
    int x = (int) stepNumber;
    SmartDashboard.putNumber("CompH " + String.valueOf(x), m_drive.compSpeed[x]);
    m_drive.tankDrive(m_lSpeed , m_lSpeed+ m_drive.compSpeed[x]);

    m_logger.writeData(stepNumber, m_drive.getLeft(), m_drive.getLeftPWM(), m_drive.getLeftSpeed(),
        m_drive.getLeftRate(), m_drive.getFilLeftRate(), m_drive.getLeftEncoderCount(), m_drive.getRight(),
        m_drive.getRightPWM(), m_drive.getRightSpeed(), m_drive.getRightRate(), m_drive.getFilRightRate(),
        m_drive.getRightEncoderCount(), m_drive.getGyroAngleZ(), RobotController.getBatteryVoltage() * .75);

    if (m_lSpeed > .99)
      m_direction = true;
    if (m_direction && m_lSpeed < -.99)
      m_secondPass = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
    m_logger.close();
    stepNumber = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_direction && m_lSpeed < .1;
    // return m_direction && m_secondPass && m_lSpeed > -.01;
  }
}
