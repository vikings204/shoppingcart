// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;


public class StrafeModule {
  private final CANSparkMax m_driveMotor;
  private final TalonSRX m_turningMotor;
  private final RelativeEncoder m_driveEncoder;

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

  public StrafeModule(int driveMotorChannel,
                      int turningMotorChannel) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new TalonSRX(turningMotorChannel);

    m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0 ); //Set the feedback device that is hooked up to the talon

    m_turningMotor.setSelectedSensorPosition(0);

    m_turningMotor.config_kP(0, 1);
    m_turningMotor.config_kI(0, 0.001);
    m_turningMotor.config_kD(0, 0.0);
    
    m_turningMotor.setNeutralMode(NeutralMode.Brake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningMotor.configSelectedFeedbackCoefficient(1);

    // Set whether turning encoder should be reversed or not
    m_turningMotor.setInverted(Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_turningPIDController.enableContinuousInput(-1023, 1023);
  }

  public void forward(double sp) {
    m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
    if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(0)) < 10) {
      m_driveMotor.set(sp);
    }
  }

  public void strafe(double d) {
    if (d > 0) { // right
      d = Math.abs(d);
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(90));
      if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(90)) < 10) {
        m_driveMotor.set(d);
      }
    } else if (d < 0) { // left
      d = Math.abs(d);
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(-90));
      if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(-90)) < 10) {
        m_driveMotor.set(d);
      }
    } else {
      m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
      m_driveMotor.set(0);
    }
  }

  public void rotate(int deg, double sp) {
    m_turningMotor.set(TalonSRXControlMode.Position, unitConv(deg));
    if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(deg)) < 10) {
      m_driveMotor.set(sp);
    }
  }

  public void setZero() {
    m_turningMotor.setSelectedSensorPosition(0);
  }

  public double unitConv(double d) {
    d = d/360;
    return d*1023;
  }
}
