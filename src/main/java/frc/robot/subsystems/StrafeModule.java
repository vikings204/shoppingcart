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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

import static frc.robot.Constants204.Controller.LEFT_X_MAG_DEADBAND;
import static frc.robot.Constants204.Drivetrain.*;
import frc.robot.util.PolarCoordinate;


public class StrafeModule {
    private final CANSparkMax m_driveMotor;
    private final TalonSRX m_turningMotor;
    private final RelativeEncoder m_driveEncoder;

    private final PIDController m_drivePIDController =
            new PIDController(0.1, 1e-4, 1);

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

        m_turningMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.Analog, 0, 0); //Set the feedback device that is hooked up to the talon

        m_turningMotor.setSelectedSensorPosition(0);

        m_turningMotor.config_kP(0, STRAFE_TURNING_PID_P);
        m_turningMotor.config_kI(0, STRAFE_TURNING_PID_I);
        m_turningMotor.config_kD(0, STRAFE_TURNING_PID_D);

        m_turningMotor.setNeutralMode(NeutralMode.Brake);

        m_driveEncoder = m_driveMotor.getEncoder();

        m_turningMotor.configSelectedFeedbackCoefficient(1);

        // Set whether turning encoder should be reversed or not
        //m_turningMotor.setInverted(Constants.DriveConstants.kFrontLeftTurningEncoderReversed);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningPIDController.enableContinuousInput(-1023, 1023);
    }

    public void forward(double sp) {
        System.out.println("SP: " + sp);
        /* m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
        if (Math.abs(m_turningMotor.getSelectedSensorPosition() - unitConv(0)) < 10) {
            m_driveMotor.set(sp);
        }*/

        m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
        if (sp > 0) { // forward
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
            //sp = Math.abs(sp);
            //m_turningMotor.set(TalonSRXControlMode.Position, unitConv(180));
            if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(0)) < 20) {
                m_driveMotor.set(sp);
                System.out.println("RX" + sp);
            }
            //System.out.println("CONV: " + unitConv(0));
        } else if (sp < 0) { // backward
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());

            //sp = Math.abs(sp);
            //m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
            if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(0)) < 20) {
                m_driveMotor.set(sp);
                System.out.println("LX: " + sp);
            }
            //System.out.println("CONV: " + unitConv(180));
        } else {
            resetPos();
        }
    }

    public void strafe(double d) {
        if (d > 0) { // right
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());
            d = Math.abs(d);
            m_turningMotor.set(TalonSRXControlMode.Position, unitConv(90));
            if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(90)) < 20) {
                m_driveMotor.set(d);
                //System.out.println("RX" + d);
            }
            //System.out.println("CONV: " + unitConv(90));
        } else if (d < 0) { // left
            //System.out.println("SENSOR: " + m_turningMotor.getSelectedSensorPosition());

            d = Math.abs(d);
            m_turningMotor.set(TalonSRXControlMode.Position, unitConv(-90));
            if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(-90)) < 20) {
                m_driveMotor.set(d);
                //System.out.println("LX: " + d);
            }
            //System.out.println("CONV: " + unitConv(-90));
        } else {
            resetPos();
        }
    }

    public void rotate(int deg, double sp) {
        //System.out.println("SP: " + sp);
        if (sp != 0) {
            m_turningMotor.set(TalonSRXControlMode.Position, unitConv(deg));

            if (Math.abs(m_turningMotor.getSelectedSensorPosition() - unitConv(deg)) < 20) {
                m_driveMotor.set(sp);
            }
        } else {
            resetPos();
        }
    }

    public void fullStrafe(PolarCoordinate pc) {
        if (pc.mag > LEFT_X_MAG_DEADBAND) {
            m_turningMotor.set(TalonSRXControlMode.Position, unitConv(pc.deg));
            if (Math.abs(m_turningMotor.getSelectedSensorPosition()-unitConv(pc.deg)) < 20) {
                m_driveMotor.set(pc.mag);
            }
        } else {
            resetPos();
        }
    }

    public void resetPos() {
        m_turningMotor.set(TalonSRXControlMode.Position, unitConv(0));
        m_driveMotor.set(0);
    }

    public void setZero() {
        m_turningMotor.setSelectedSensorPosition(0);
    }

    public double unitConv(double d) {
        d = d / 360;
        return d * 1023;
    }

    public double getTurnEncDeg() {
        double deg = (m_turningMotor.getSelectedSensorPosition() / 1023) * 360;
        if (deg > 180) {
            deg = -(deg-180);
        }
        return deg;
    }
}
