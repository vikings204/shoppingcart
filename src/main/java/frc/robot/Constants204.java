// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.InterpLUT;

public final class Constants204 {
    public static final class DrivetrainCAN {
        public static final int FL_DRIVE_MOTOR_ID = 5;
        public static final int RL_DRIVE_MOTOR_ID = 6;
        public static final int FR_DRIVE_MOTOR_ID = 7;
        public static final int RR_DRIVE_MOTOR_ID = 8;

        public static final int FL_TURNING_MOTOR_ID = 4;//1;
        public static final int RL_TURNING_MOTOR_ID = 1;//2;
        public static final int FR_TURNING_MOTOR_ID = 3;
        public static final int RR_TURNING_MOTOR_ID = 2;//4;

        public static final int SINGLE_STRAFE_DRIVE_MOTOR = 7;
        public static final int SINGLE_STRAFE_TURNING_MOTOR = 3;
    }

    public static final class ArmCAN {
        public static final int BOOM_MOTOR_ID = 71;
        public static final int DIPPER_MOTOR_ID = 72;
        public static final int /*CLAW_MOTOR_ID*/CLAW_SERVO_PWM_CH = 1; // controlled with PWM, NOT CAN
    }

    public static final class Arm {
        public static final double BOOM_REF_INCREMENT = 0.01;
        public static final double DIPPER_REF_INCREMENT = 0.01;
        public static final double CLAW_CLOSED_EXPOS = 0;
        public static final double CLAW_OPEN_EXPOS = 1;
    }

    public static final class Controller {
        public static final int PORT = 1;
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double RIGHT_Y_DEADBAND = 0.0;
    }
    public static final class Drivetrain {
        public static final double FL_Input_1=0.0 ; 
        public static final double FL_Input_2=90.0 ; 
        public static final double FL_Input_3=-90.0 ; 
        public static final double FL_Input_4=180.0 ;
        public static final double FL_Output_1=0.0 ; 
        public static final double FL_Output_2=90.0 ; 
        public static final double FL_Output_3=-90.0 ; 
        public static final double FL_Output_4=180.0 ; 
        public static final InterpLUT FL_LUT = new InterpLUT(FL_Input_1,FL_Input_2,FL_Input_3,FL_Input_4,FL_Output_1,FL_Output_2,FL_Output_3,FL_Output_4);
        public static final InterpLUT BL_LUT = new InterpLUT(FL_Input_1,FL_Input_2,FL_Input_3,FL_Input_4,FL_Output_1,FL_Output_2,FL_Output_3,FL_Output_4);
        public static final InterpLUT FR_LUT = new InterpLUT(FL_Input_1,FL_Input_2,FL_Input_3,FL_Input_4,FL_Output_1,FL_Output_2,FL_Output_3,FL_Output_4);
        public static final InterpLUT BR_LUT = new InterpLUT(FL_Input_1,FL_Input_2,FL_Input_3,FL_Input_4,FL_Output_1,FL_Output_2,FL_Output_3,FL_Output_4);

    
        //FL_LUT.add(FL_Input_1, FL_Output_1);

    }
}