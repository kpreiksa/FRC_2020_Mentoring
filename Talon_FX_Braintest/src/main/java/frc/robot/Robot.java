/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and speed controller inputs also
 * range from -1 to 1 making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent to the Dashboard.
 */
public class Robot extends TimedRobot {
  // private static final int kMotorPort = 0;
  private static final int kJoystickPort = 0;
  private static final int kRightTriggerAxis = 3;
  // private static final int kEncoderPortA = 0;
  // private static final int kEncoderPortB = 1;
  WPI_TalonFX brainmotor;
  TalonFXSensorCollection brainmotorsensors;

  double triggerposition;

  private SpeedController m_motor;
  private Joystick m_joystick;
  private Encoder m_encoder;

  @Override
  public void robotInit() {
    brainmotor = new WPI_TalonFX(1);
    brainmotorsensors = brainmotor.getSensorCollection();
    // m_motor = new PWMVictorSPX(kMotorPort);
    m_joystick = new Joystick(kJoystickPort);
    // m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    // m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    double absolutePosition = brainmotorsensors.getIntegratedSensorAbsolutePosition();
    double velocity = brainmotorsensors.getIntegratedSensorVelocity();
    SmartDashboard.putNumber("Encoder", absolutePosition);
    SmartDashboard.putNumber("Velocity", velocity);
    triggerposition = m_joystick.getRawAxis(kRightTriggerAxis);
    brainmotor.set(triggerposition);
  }

  @Override
  public void teleopPeriodic() {

    // triggerposition = m_joystick.getRawAxis(kRightTriggerAxis);
    // m_motor.set(triggerposition);
    //m_motor.set(m_joystick.getY());
  }
}
