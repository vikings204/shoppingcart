// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.InterpLUT;

public final class Constants204 {
    public static final class DrivetrainCAN {
        public static final int FL_DRIVE_MOTOR_ID = 12;
        public static final int RL_DRIVE_MOTOR_ID = 15;
        public static final int FR_DRIVE_MOTOR_ID = 22;
        public static final int RR_DRIVE_MOTOR_ID = 25;

        public static final int FL_TURNING_MOTOR_ID = 13;
        public static final int RL_TURNING_MOTOR_ID = 14;
        public static final int FR_TURNING_MOTOR_ID = 23;
        public static final int RR_TURNING_MOTOR_ID = 24;

        public static final int SINGLE_STRAFE_DRIVE_MOTOR = 71;
        public static final int SINGLE_STRAFE_TURNING_MOTOR = 23;
    }

    public static final class ArmCAN {
        public static final int BOOM_MOTOR_ID = 11;
        public static final int DIPPER_MOTOR_ID = 21;
        public static final int /*CLAW_MOTOR_ID*/CLAW_SERVO_PWM_CH = 8; // controlled with PWM, NOT CAN
    }

    public static final class Arm {
        public static final double BOOM_REF_INCREMENT = 0.1;
        public static final double DIPPER_REF_INCREMENT = 1;
        public static final double CLAW_CLOSED_EXPOS = 0;
        public static final double CLAW_OPEN_EXPOS = 1;
    }

    public static final class Controller {
        public static final int PORT = 1;
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double RIGHT_Y_DEADBAND = 0.0;
        public static final double LEFT_X_MAG_DEADBAND = 0.1;
    }
    public static final class Drivetrain {
        public static final double FL_Input_1 = 0.0;
        public static final double FL_Input_2 = 90.0;
        public static final double FL_Input_3 = -90.0;
        public static final double FL_Input_4 = 180.0;
        public static final double FL_Output_1 = 0.0;
        public static final double FL_Output_2 = 90.0;
        public static final double FL_Output_3 = -90.0;
        public static final double FL_Output_4 = 180.0;
        public static final double RL_Input_1 = 0.0;
        public static final double RL_Input_2 = 90.0;
        public static final double RL_Input_3 = -90.0;
        public static final double RL_Input_4 = 180.0;
        public static final double RL_Output_1 = 0.0;
        public static final double RL_Output_2 = 90.0;
        public static final double RL_Output_3 = -90.0;
        public static final double RL_Output_4 = 180.0;
        public static final double FR_Input_1 = 0.0;
        public static final double FR_Input_2 = 90.0;
        public static final double FR_Input_3 = -90.0;
        public static final double FR_Input_4 = 180.0;
        public static final double FR_Output_1 = 0.0;
        public static final double FR_Output_2 = 90.0;
        public static final double FR_Output_3 = -90.0;
        public static final double FR_Output_4 = 180.0;
        public static final double RR_Input_1 = 0.0;
        public static final double RR_Input_2 = 90.0;
        public static final double RR_Input_3 = -90.0;
        public static final double RR_Input_4 = 180.0;
        public static final double RR_Output_1 = 0.0;
        public static final double RR_Output_2 = 90.0;
        public static final double RR_Output_3 = -90.0;
        public static final double RR_Output_4 = 180.0; 
        public static final InterpLUT FL_LUT = new InterpLUT(FL_Input_1, FL_Input_2, FL_Input_3, FL_Input_4, FL_Output_1, FL_Output_2, FL_Output_3, FL_Output_4);
        public static final InterpLUT RL_LUT = new InterpLUT(RL_Input_1, RL_Input_2, RL_Input_3, RL_Input_4, RL_Output_1, RL_Output_2, RL_Output_3, RL_Output_4);
        public static final InterpLUT FR_LUT = new InterpLUT(FR_Input_1, FR_Input_2, FR_Input_3, FR_Input_4, FR_Output_1, FR_Output_2, FR_Output_3, FR_Output_4);
        public static final InterpLUT RR_LUT = new InterpLUT(RR_Input_1, RR_Input_2, RR_Input_3, RR_Input_4, RR_Output_1, RR_Output_2, RR_Output_3, RR_Output_4);

        public static final double STRAFE_TURNING_PID_P = 5;
        public static final double STRAFE_TURNING_PID_I = 0.001;
        public static final double STRAFE_TURNING_PID_D = 0.000;
    }
}