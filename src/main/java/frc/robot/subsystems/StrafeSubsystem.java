// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204;
import frc.robot.util.Math204;
import frc.robot.util.PolarCoordinate;

public class StrafeSubsystem extends SubsystemBase {
    private final StrafeModule m_frontLeft =
            new StrafeModule(
                    Constants204.DrivetrainCAN.FL_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.FL_TURNING_MOTOR_ID
            );

    private final StrafeModule m_rearLeft =
            new StrafeModule(
                    Constants204.DrivetrainCAN.RL_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.RL_TURNING_MOTOR_ID
            );

    private final StrafeModule m_frontRight =
            new StrafeModule(
                    Constants204.DrivetrainCAN.FR_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.FR_TURNING_MOTOR_ID
            );

    private final StrafeModule m_rearRight =
            new StrafeModule(
                    Constants204.DrivetrainCAN.RR_DRIVE_MOTOR_ID,
                    Constants204.DrivetrainCAN.RR_TURNING_MOTOR_ID
            );

    public StrafeSubsystem() {
    }

    @Override
    public void periodic() {
    }

    public void basicDrive(double forward, double strafe, double rot) {
        //System.out.println("FW:"+forward + " ST:"+strafe + " RT:"+rot);
        double r = EQ.rotate(rot);
        double s = EQ.strafe(strafe);
        double f = EQ.forward(forward);
        //System.out.println("F:"+f+" S:"+s+" R:"+r);
        if (r != 0) {
            // FL-135 FR-45 RL-225 RR-315
            m_frontLeft.rotate(135, r);
            m_frontRight.rotate(45, r);
            m_rearLeft.rotate(-135, r);
            m_rearRight.rotate(-45, r);
        } else if (s != 0) {
            m_frontLeft.strafe(s);
            m_frontRight.strafe(s);
            m_rearLeft.strafe(s);
            m_rearRight.strafe(s);
        } else if (f != 0) {
            m_frontLeft.forward(f);
            m_frontRight.forward(f);
            m_rearLeft.forward(f);
            m_rearRight.forward(f);
        } else {
            m_frontLeft.resetPos();
            m_frontRight.resetPos();
            m_rearLeft.resetPos();
            m_rearRight.resetPos();
        }
    }

    public void moreDrive(double sx, double sy, double rot) {
        double r = EQ.rotate(rot);
        PolarCoordinate pc = Math204.CartesianToPolar(sx, sy);
        pc.mag = EQ.strafeMag(pc.mag*-1);
        if (r != 0) {
            // FL-135 FR-45 RL-225 RR-315
            m_frontLeft.rotate(135, r);
            m_frontRight.rotate(45, r);
            m_rearLeft.rotate(-135, r);
            m_rearRight.rotate(-45, r);
        } else if (pc.mag != 0) {
            System.out.println("SX:" + sx + " SY:" + sy);
            //System.out.println("MAG:" + pc.mag + " DEG:" + pc.deg);
            m_frontLeft.fullStrafe(pc);
            m_frontRight.fullStrafe(pc);
            m_rearLeft.fullStrafe(pc);
            m_rearRight.fullStrafe(pc);
        } else {
            m_frontLeft.resetPos();
            m_frontRight.resetPos();
            m_rearLeft.resetPos();
            m_rearRight.resetPos();
        }
    }

    public void setZero() {
        m_frontLeft.setZero();
        m_frontRight.setZero();
        m_rearLeft.setZero();
        m_rearRight.setZero();
    }

    public void rottenest() {
        m_frontLeft.rotate(0, 0.1);
        m_frontLeft.rotate(90, 0.1);
        m_frontLeft.rotate(180, 0.1);
        m_frontLeft.rotate(270, 0.1);
        m_frontLeft.rotate(360, 0.1);
        m_frontLeft.rotate(450, 0.1);
        m_frontLeft.rotate(540, 0.1);
        m_frontLeft.rotate(630, 0.1);
        m_frontLeft.rotate(720, 0.1);
    }

    private static class EQ {
        public static double forward(double in) {
            if (Math.abs(in) < Constants204.Controller.LEFT_Y_DEADBAND) {
                return 0.0;
            } else {
                return in/2;
            }
        }

        public static double strafe(double in) {
            if (Math.abs(in) < Constants204.Controller.LEFT_X_DEADBAND) {
                return 0.0;
            } else {
                return in/2;
            }
        }

        public static double rotate(double in) {
            if (Math.abs(in) < Constants204.Controller.RIGHT_X_DEADBAND) {
                return 0.0;
            } else {
                return in/4;
            }
        }

        public static double strafeMag(double in) {
            if (Math.abs(in) < Constants204.Controller.LEFT_X_MAG_DEADBAND) {
                return 0.0;
            } else {
                return in/2;
            }
        }
    }

    public String TestEncoders() {
        return ("FL DEG: " + m_frontLeft.getTurnEncDeg() +
        "\nFR DEG: " + m_frontRight.getTurnEncDeg() +
        "\nRL DEG: " + m_rearLeft.getTurnEncDeg() +
        "\nRR DEG: " + m_rearRight.getTurnEncDeg() +
        "\n==========================");
    }
}
