// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import java.lang.AutoCloseable;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SwerveModule {
  boolean busy, finished; 

  private final CANSparkMax m_driveMotor;
  private final TalonSRX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  //private final Encoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderChannels,
      int[] turningEncoderChannels,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new TalonSRX(turningMotorChannel);

    
    m_turningMotor.set(ControlMode.Position, 0.0); //Change control mode of talon, default is PercentVbus (-1.0 to 1.0)
    m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0 ); //Set the feedback device that is hooked up to the talon

    //m_turningMotor.configurePID (0.5, 0.001, 0.00, 0.00, 360, 36, 0); //Set the PID constants (p, i, d)
    m_turningMotor.config_kP(0, 0.5);
    m_turningMotor.config_kI(0, 0.001);
    m_turningMotor.config_kD(0, 0.0);
    
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    // m_turningMotor.enableControl(); //Enable PID control on the talon
    //int currentPosition = talon.getEncPosition();
    int currentPosition = (int) m_turningMotor.getSelectedSensorPosition();

    m_driveEncoder = m_driveMotor.getEncoder();

    //m_turningEncoder = m_turningMotor. new Encoder(turningEncoderChannels[0], turningEncoderChannels[1]);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    ((Encoder) m_driveEncoder).setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    ((Encoder) m_driveEncoder).setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    m_turningMotor.configSelectedFeedbackCoefficient(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_turningMotor.setInverted(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        ((Encoder) m_driveEncoder).getRate(), new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        ((Encoder) m_driveEncoder).getDistance(), new Rotation2d(m_turningMotor.getSelectedSensorPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getSelectedSensorPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(((Encoder) m_driveEncoder).getRate(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(TalonSRXControlMode.Position, state.angle.getRadians());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    ((PIDController) m_driveEncoder).reset();
    // FIXME: is this right? maybe offset
    m_turningMotor.setSelectedSensorPosition(0);
  }
}
