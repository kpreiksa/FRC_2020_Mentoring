/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

@SuppressWarnings("PMD.ExcessiveImports")
public class FalconMotorSubsystem {

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kTurningEncoderDistancePerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;

  }

  private final String moduleIdentifier;
  private final TalonFX m_driveMotor;
  private final VictorSP m_turningMotor;

  private final TalonFXSensorCollection m_driveMotorSensors;
  private final AnalogPotentiometer m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(1.0, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1.0, 0, 0,
      new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  public static final AnalogPotentiometer turningEncoder = new AnalogPotentiometer(1, 360.0, 0.0);

  public static final int ticksPerTireRotation = 13600;
  /**
   * Creates a new DriveSubsystem.
   */
  public FalconMotorSubsystem() {

    m_driveMotor = new TalonFX(2);
    m_turningMotor = new VictorSP(1);

    this.moduleIdentifier = "Test";

    this.m_driveMotorSensors = m_driveMotor.getSensorCollection(); // new Encoder(driveEncoderPorts[0],
                                                                   // driveEncoderPorts[1]);

    this.m_turningEncoder = turningEncoder;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void spinMotor(final double speed, final double angle) {
    // Calculate the drive output from the drive PID controller.
    // final var driveOutput = m_drivePIDController.calculate(m_driveMotorSensors.getIntegratedSensorVelocity(), speed);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(m_turningEncoder.get(), angle);

    SmartDashboard.putNumber("turningEncoder_" + this.moduleIdentifier, m_turningEncoder.get());
    SmartDashboard.putNumber("driveEncoderVelocity_" + this.moduleIdentifier,
        m_driveMotorSensors.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("driveEncoderPosition_" + this.moduleIdentifier,
        m_driveMotorSensors.getIntegratedSensorPosition());
    SmartDashboard.putNumber("driveEncoderAbsolutePosition_" + this.moduleIdentifier,
        m_driveMotorSensors.getIntegratedSensorAbsolutePosition());
    // SmartDashboard.putNumber("driveOutput_" + this.moduleIdentifier, driveOutput);
    SmartDashboard.putNumber("turnOutput_" + this.moduleIdentifier, turnOutput);

    // System.out.println("Setting State. Drive Motor Output = " + driveOutput + ".
    // Turning motor output = " + turnOutput);

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(ControlMode.PercentOutput, speed);
    m_turningMotor.set(0);

  }
}
