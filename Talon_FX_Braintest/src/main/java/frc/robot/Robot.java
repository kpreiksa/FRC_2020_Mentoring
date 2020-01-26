/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

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
  PIDSubsystem pid_Sliders;
  double triggerposition;

  private SpeedController m_motor;
  private Joystick m_joystick;
  private Encoder m_encoder;

  private PowerDistributionPanel m_pdp;

  
public class Gains {

	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;

	private Gains(final double _kP, final double _kI, final double _kD, final double _kF, final int _kIzone,
        final double _kPeakOutput) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
      kF = _kF;
      kIzone = _kIzone;
      kPeakOutput = _kPeakOutput;
    }

  }

  public static final int kSlotIdx = 0;
  public static final int kPIDLoopIdx = 0;
  public static final int kTimeoutMs = 30;

  public static final double kP = 0.25;
  public static final double kI = 0.01;
  public static final double kD = 5;

  public static Gains kGains_Velocity;

  @Override
  public void robotInit() {
    brainmotor = new WPI_TalonFX(1);
    brainmotorsensors = brainmotor.getSensorCollection();
    // m_motor = new PWMVictorSPX(kMotorPort);
    m_joystick = new Joystick(kJoystickPort);
    SmartDashboard.putNumber("motorcontroltype", 0);
    SmartDashboard.putNumber("motor_desiredposition", 0);
    SmartDashboard.putNumber("motor_desiredvelocity", 0);
    m_pdp = new PowerDistributionPanel(0);
    

    // LiveWindow lw = LiveWindow.

    
    brainmotor.configFactoryDefault();

		/* Config sensor used for Primary PID [Velocity] */

    brainmotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    brainmotor.setSensorPhase(true);

		/* Config the peak and nominal outputs */

		brainmotor.configNominalOutputForward(0, kTimeoutMs);
		brainmotor.configNominalOutputReverse(0, kTimeoutMs);
		brainmotor.configPeakOutputForward(1, kTimeoutMs);
    brainmotor.configPeakOutputReverse(-1, kTimeoutMs);
    
    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);

    
    // m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    // m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the robot
   * mode.
   */
  @Override
  public void robotPeriodic() {
    final double absolutePosition = brainmotorsensors.getIntegratedSensorAbsolutePosition();
    final double relativePosition = brainmotorsensors.getIntegratedSensorPosition();
    final double velocity = brainmotorsensors.getIntegratedSensorVelocity();
    double motorcontroltype = 0;
    SmartDashboard.putNumber("Encoder absolute", absolutePosition);
    SmartDashboard.putNumber("Encoder relative", relativePosition);
    SmartDashboard.putNumber("Velocity", velocity);

    double kP_l;
    double kI_l;
    double kD_l;

    kP_l = SmartDashboard.getNumber("kP", kP);
    kI_l = SmartDashboard.getNumber("kI", kI);
    kD_l = SmartDashboard.getNumber("kD", kD);

    kGains_Velocity = new Gains(kP_l, kI_l, kD_l, 1023.0 / 7200.0, 300, 1.00);

    /* Config the Velocity closed loop gains in slot0 */
    brainmotor.config_kF(kPIDLoopIdx, kGains_Velocity.kF, kTimeoutMs);
    brainmotor.config_kP(kPIDLoopIdx, kGains_Velocity.kP, kTimeoutMs);
    brainmotor.config_kI(kPIDLoopIdx, kGains_Velocity.kI, kTimeoutMs);
    brainmotor.config_kD(kPIDLoopIdx, kGains_Velocity.kD, kTimeoutMs);



    motorcontroltype = SmartDashboard.getNumber("motorcontroltype", 0);
    final double motor_desired_position = SmartDashboard.getNumber("motor_desiredposition", 0);
    final double motor_desired_velocity = SmartDashboard.getNumber("motor_desiredvelocity", 0);
    if (motorcontroltype == 1.0)
    {
      //brainmotor.set(0.3);
      brainmotor.set(TalonFXControlMode.Position,motor_desired_position);
    }
    else if (motorcontroltype == 2.0)
    {
    // motor velocity is in  2048 ticks = 600 rpm
    brainmotor.set(TalonFXControlMode.Velocity,motor_desired_velocity);
    }
    else
    {
      // triggerposition = m_joystick.getRawAxis(kRightTriggerAxis);
      brainmotor.set(TalonFXControlMode.Disabled,0);
    }

    for(int i = 0; i < 16; i++)
    {
      SmartDashboard.putNumber("Current Channel " + Integer.toString(i), m_pdp.getCurrent(i));
    }

    SmartDashboard.putNumber("Output Current", brainmotor.getSupplyCurrent());
    SmartDashboard.putNumber("Temperature", brainmotor.getTemperature());

  }

  @Override
  public void teleopPeriodic() {

    // triggerposition = m_joystick.getRawAxis(kRightTriggerAxis);
    // m_motor.set(triggerposition);
    //m_motor.set(m_joystick.getY());
  }
}
