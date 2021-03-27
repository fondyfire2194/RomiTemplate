// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SD;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = .07;// mm
  private static final double kWheelDiameterInch = 2.75591;
  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  public final Spark m_leftMotor = new Spark(0);
  public final Spark m_rightMotor = new Spark(1);
  public Joystick m_joystick = new Joystick(0);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  public final Encoder m_leftEncoder = new Encoder(4, 5);
  public final Encoder m_rightEncoder = new Encoder(6, 7);
  private LinearFilter lFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter rFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private double lfileredRate;
  private double rFilteredRate;
  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  public double[] compSpeed = new double[200];

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double lAxisSpeed, double rAxisSpeed) {
    m_diffDrive.tankDrive(lAxisSpeed, rAxisSpeed);
  }

  public void setLeft(double pct) {
    m_leftMotor.set(pct);
  }

  public void setright(double pct) {
    m_rightMotor.set(pct);
  }

  public double getTurnRate() {
    return 7;
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeter() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
  }

  public double getLeftRate() {
    return m_leftEncoder.getRate();
  }

  public double getRightRate() {
    return m_rightEncoder.getRate();
  }

  public double getFilLeftRate() {
    return lfileredRate;
  }

  public double getFilRightRate() {
    return rFilteredRate;
  }

  public double getLeftPWM() {
    return m_leftMotor.getSpeed();
  }

  public double getRightPWM() {
    return m_rightMotor.getSpeed();
  }

  public double getLeft() {
    return m_leftMotor.get();
  }

  public double getRight() {
    return m_rightMotor.get();
  }

  public double getLeftSpeed() {
    return m_leftMotor.getSpeed();
  }

  public double getRightSpeed() {
    return m_rightMotor.getSpeed();
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public double getJoystickThrottle() {

    double temp = m_joystick.getRawAxis(3);
   // temp += 2;
    return temp;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lfileredRate = lFilter.calculate(getLeftRate());
    rFilteredRate = rFilter.calculate(getRightRate());
    SD.putN2("LRateMPS", getLeftRate());
    SD.putN2("RRateMPS", getRightRate());
    SD.putN2("LFRateMPS", lfileredRate);

    SD.putN2("RFRateMPS", rFilteredRate);
    SD.putN2("FRateDiff", lfileredRate - rFilteredRate);

    SD.putN4("LPWM", getLeftPWM());
    SD.putN4("RPWM", getRightPWM());
    SD.putN4("LPCT", m_leftMotor.get());
    SD.putN4("RPCT", m_rightMotor.get());
    SD.putN4("LSpeed", getLeftSpeed());
    SD.putN4("RSpeed", getRightSpeed());
 SmartDashboard.putNumber("Throttle", getJoystickThrottle());
    for (int i = 0; i < 12; i++) {

      SmartDashboard.putNumber("Comp " + String.valueOf(i), compSpeed[i]);
     
    }
  }
}
