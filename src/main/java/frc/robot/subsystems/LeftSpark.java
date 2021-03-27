// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftSpark extends SubsystemBase {
  /** Creates a new LeftSpark. */
  /**
   * 
   * Motor nominal 150rpm * 120 gear ratio at 4.5 volts no load = 300 revpersec So
   * 300 revpersec / 4.5 = 300/4.5 or 67 rps per volt pi * .07 = .22 meters
   * circumference
   * 
   * Seeing max of .8 meters per sec = 3.6 rps wheel * 120 = 432 rps motor
   * 
   */

  private static final double VOLTS_PER_RPS = 66;// at 8 volts
  private static final double FEED_FORWARD = .9;
  private static final double REF_BAT_VOLTS = 8;
  private static final double MIN_BAT_VOLTS = 7;
  private static PIDController m_PID = new PIDController(0, 0, 0);

  private static Spark m_motor = new Spark(0);
  private static Encoder m_encoder = new Encoder(4, 5);
  private static double m_batteryVolts;
  private static double m_speed;
  private static double max_rps;
  private double kP;

  public LeftSpark() {
    SmartDashboard.putNumber("KP", kP);
  }

  public double getBatteryVoltageRounded() {

    return Math.round(RobotController.getBatteryVoltage() * 75. / 10.);
  }

  public double getFeedForward(double controllerValue) {
    return FEED_FORWARD * getBatteryVoltageRounded();
  }

  public double getEncoderRate() {
    return m_encoder.getRate();
  }

  public double getPidOut(double speedRPS) {
    return m_PID.calculate(getEncoderRate(), speedRPS);
  }

  public void runMotorVolts(double speedRPS) {
    double temp = speedRPS * VOLTS_PER_RPS;
    setVolts(getFeedForward(temp) + getPidOut(speedRPS) * VOLTS_PER_RPS);
  }

  public void setVolts(double volts) {
    m_motor.set(volts / getBatteryVoltageRounded());
  }

  public static void set(double pct) {
    m_motor.set(pct);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kP = SmartDashboard.getNumber("KP", 0);
    max_rps = m_batteryVolts / VOLTS_PER_RPS;
    m_PID.setP(kP);
    SmartDashboard.putNumber("RAte", m_encoder.getRate());
  }
}
