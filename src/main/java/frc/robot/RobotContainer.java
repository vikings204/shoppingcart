// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.StrafeSubsystem;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.hal.CANData;
import frc.robot.subsystems.TagVisionSubsystem;
import frc.robot.util.Gamepad;
import frc.robot.util.Math204;
import frc.robot.util.PolarCoordinate;

import java.util.concurrent.TimeUnit;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    //private final SingleStrafeSubsystem m_singleStrafeDrive = new SingleStrafeSubsystem();
    public final StrafeSubsystem strafeDrive = new StrafeSubsystem();
    public final ArmSubsystem armControl = new ArmSubsystem();
    private final TagVisionSubsystem tagVision = new TagVisionSubsystem();
    public int autoStateMachine = 3;
    Gamepad CONTROLLER = new Gamepad(Constants204.Controller.PORT);
    Joystick JOYSTICK = new Joystick(0);
    private final CAN gyro = new CAN(1, 8, 4);
    private final CANData gyroData = new CANData();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }
    int i = 1;
    /*public Command getTeleopSingleStrafeCommand() {
        return new RunCommand(
                () -> {
                    if (m_driverController.getYButton()) {
                        m_singleStrafeDrive.setZero();
                        //canTest();
                    } else if (m_driverController.getXButton()) {
                        //System.out.println(visionTestSubsystem.printLatestID());
                    }

                    m_singleStrafeDrive.strafe(m_driverController.getLeftX());
                    //System.out.println("LX-CTRL: " + m_driverController.getLeftX());
                }, m_singleStrafeDrive);
    }*/

    public Command getTestStrafeCommand() {
        System.out.println(strafeDrive.TestEncoders() + "\naiushduhw98ahhs9dh9hwhqhh9][a[s]d[][");
        return new RunCommand(() -> {
            System.out.println(strafeDrive.TestEncoders());
            try {
                TimeUnit.SECONDS.sleep(0);
            } catch (InterruptedException e) {
                throw new RuntimeException("how tf you crash the test func: " + e);
            }
        }, strafeDrive);
    }

    public Command getTeleopStrafeCommand() {
        return new RunCommand(() -> {
            //System.out.println("RX: " + CONTROLLER.getRightX());
            //strafeDrive.basicDrive(CONTROLLER.getLeftY(), CONTROLLER.getLeftX(), CONTROLLER.getRightX());
            strafeDrive.moreDrive(CONTROLLER.getLeftX(), CONTROLLER.getLeftY(), CONTROLLER.getRightX());

            double armB=0.0, armD=0.0, armC=0.0;
            if (CONTROLLER.getRightUpperBumper()) { armB = -1; } else if (CONTROLLER.getRightTriggerAxis()>0.2) { armB = 1; }
            if (CONTROLLER.getLeftUpperBumper()) { armD = -1; } else if (CONTROLLER.getLeftTriggerAxis()>0.2) { armD = 1; }
            if (CONTROLLER.getBButton()) { armC = 1; } else if (CONTROLLER.getXButton()) { armC = -1; }
            armControl.setArm(armB, armD, armC);
            if(CONTROLLER.getLeftStickButton()){ Constants204.Drivetrain.strafeDivison = 2;}
            if(CONTROLLER.getRightStickButton()){ Constants204.Drivetrain.strafeDivison = 10;}
        }, strafeDrive);
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> {
            if (autoStateMachine == 0) {autoStateMachine = 3;
                armControl.boomStart= armControl.boomEncoder.getPosition();
                armControl.dipperMax= armControl.dipperEncoder.getPosition();
                System.out.println("Boom Start is Now: "+ armControl.boomStart+"\n Dipper Max is Now: "+armControl.dipperMax);
                if (armControl.boomEncoder.getPosition() < armControl.boomStart + armControl.boomMax) {
                   // armControl.setArm(1, 0, 0);

                } else {
                    autoStateMachine++;
                }
        } else if (autoStateMachine == 1) {autoStateMachine = 3;

                armControl.setArm(0, 0, 1);

                autoStateMachine++;
            }
            else if (autoStateMachine == 2){autoStateMachine = 3;
                strafeDrive.setZero();
                strafeDrive.turningTotalDeg = 0.0;
                System.out.println("You have 0'd the turning encoders");
                autoStateMachine++;
            } 
            else if (autoStateMachine == 3) {
                //System.out.println("driving in autonomous");

                Transform3d tf = tagVision.getTransform();
                if (tf != null) {
                    System.out.println("x=" + tf.getX() + " y=" + tf.getY() + " z=" + tf.getZ() + " rot=" + tf.getRotation());
                    PolarCoordinate pc = Math204.CartesianToPolar(tf.getX(), tf.getY());
                    pc.mag = pc.mag>0.25 ? 0.25 : pc.mag;
                    pc.mag = pc.mag<0.2 ? 0 : pc.mag;
                    System.out.println("deg=" + pc.deg + " mag=" + pc.mag);
                    pc.mag*=-1;
                    strafeDrive.polarDrive(pc, 0);
                } else {
                    System.out.println("tf was null");
                }
                //strafeDrive.moreDrive(0, .21, 0);

                //strafeDrive.m_frontLeft.turningMotor.set(TalonSRXControlMode.Position,  strafeDrive.m_frontLeft.unitConv(135.0));
                //strafeDrive.m_frontRight.turningMotor.set(TalonSRXControlMode.Position,  strafeDrive.m_frontLeft.unitConv(45.0));
                //strafeDrive.m_rearLeft.turningMotor.set(TalonSRXControlMode.Position,  strafeDrive.m_frontLeft.unitConv(-135.0));
                //strafeDrive.m_rearRight.turningMotor.set(TalonSRXControlMode.Position,  strafeDrive.m_frontLeft.unitConv(-45.0));
            }
            //if (autoStateMachine ==2) {
            //    strafeDrive.moreDrive(0, 0, 0);
            //}
        }, strafeDrive);
    }

    public void canTest() {
        //System.out.println(gyrotest.readNext());
        synchronized (this) {
            gyro.readPacketLatest(0, gyroData);
            //System.out.println("DATAZERO: " + gyrodata.data[0]);
            double x = (double) ((gyroData.data[0] & 0xFF) | (gyroData.data[1]) << 8) / (1 << 14);
            double y = (double) ((gyroData.data[2] & 0xFF) | (gyroData.data[3]) << 8) / (1 << 14);
            double z = (double) ((gyroData.data[4] & 0xFF) | (gyroData.data[5]) << 8) / (1 << 14);
            double w = (double) ((gyroData.data[6] & 0xFF) | (gyroData.data[7]) << 8) / (1 << 14);
            /*double y = (double) (gyrodata.data[2] | gyrodata.data[3] << 8) / (1 << 14);
            double z = (double) (gyrodata.data[4] | gyrodata.data[5] << 8) / (1 << 14);
            double w = (double) (gyrodata.data[6] | gyrodata.data[7] << 8) / (1 << 14);*/

            /* System.out.println("DATA_0: " + (0xFF & gyrodata.data[0]));
            System.out.println("DATA_1: " + (0xFF & gyrodata.data[1]));
            System.out.println("DATA_2: " + (0xFF & gyrodata.data[2]));
            System.out.println("DATA_3: " + (0xFF & gyrodata.data[3]));
            System.out.println("DATA_4: " + (0xFF & gyrodata.data[4]));
            System.out.println("DATA_5: " + (0xFF & gyrodata.data[5]));
            System.out.println("DATA_6: " + (0xFF & gyrodata.data[6]));
            System.out.println("DATA_7: " + (0xFF & gyrodata.data[7]));*/

            /*Quaternion q = new Quaternion(w,x,y,z);
            Vector<N3> vector = q.toRotationVector();
            System.out.println("ZERO-ZERO: " + vector.get(0,0));
            System.out.println("ONE-ZERO: " + vector.get(1,0));
            System.out.println("TWO-ZERO: " + vector.get(2,0));*/

            System.out.println("GYRO-X: " + x);
            System.out.println("GYRO-Y: " + y);
            System.out.println("GYRO-Z: " + z);
            System.out.println("GYRO-W: " + w);

            double[] euler = toEuler(x, y, z, w);
            System.out.println("EULER-X: " + euler[0] * 180 / 3.1415);
            System.out.println("EULER-Y: " + euler[1] * 180 / 3.1415);
            System.out.println("EULER-Z: " + euler[2] * 180 / 3.1415);
        }
    }

    public double[] toEuler(double _x, double _y, double _z, double _w) {
        double[] ret = new double[3];
        double sqw = _w * _w;
        double sqx = _x * _x;
        double sqy = _y * _y;
        double sqz = _z * _z;

        ret[0] = Math.atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
        ret[1] = Math.asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
        ret[2] = Math.atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

        return ret;
    }
}