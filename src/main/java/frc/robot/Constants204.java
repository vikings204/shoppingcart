// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants204 {
  public static final class CAN {
    public static final int FL_DRIVE_MOTOR_ID = 5;;
    public static final int RL_DRIVE_MOTOR_ID = 6;
    public static final int FR_DRIVE_MOTOR_ID = 7;
    public static final int RR_DRIVE_MOTOR_ID = 8;

    public static final int FL_TURNING_MOTOR_ID = 4;//1;
    public static final int RL_TURNING_MOTOR_ID = 1;//2;
    public static final int FR_TURNING_MOTOR_ID = 3;
    public static final int RR_TURNING_MOTOR_ID = 2;//4;

    public static final int SINGLE_STRAFE_DRIVE_MOTOR = 69;
    public static final int SINGLE_STRAFE_TURNING_MOTOR = 1;
  }

  public static final class Controller {
    public static final int PORT = 1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.0;
  }
}