// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants204.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private Command autonomousCommand;
    private Command teleopCommand;

    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        //CameraServer.startAutomaticCapture(); // use for USB camera
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        if (teleopCommand != null) {
            teleopCommand.cancel();
        }

        //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            //m_autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        //m_teleopCommand = m_robotContainer.getTeleopCommand();
        //m_teleopCommand = m_robotContainer.getTeleopCommand2();
        //m_teleopCommand = m_robotContainer.getTeleopSingleStrafeCommand();
        teleopCommand = robotContainer.getTeleopStrafeCommand();
        if (teleopCommand != null) {
            teleopCommand.schedule();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        //teleopCommand.cancel();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        //System.out.println(robotContainer.strafeDrive.TestEncoders());
       
        if (robotContainer.CONTROLLER.getYButton()) {
            robotContainer.strafeDrive.setZero();
            robotContainer.strafeDrive.turningTotalDeg = 0.0;
            System.out.println("You have 0'd the turning encoders");

            //Drivetrain.FL_LUT.add(0.0, robotContainer.strafeDrive.m_frontLeft.getTurnEncDeg());
            //Drivetrain.RL_LUT.add(0.0, robotContainer.strafeDrive.m_rearLeft.getTurnEncDeg());
            //Drivetrain.FR_LUT.add(0.0, robotContainer.strafeDrive.m_frontRight.getTurnEncDeg());
            //  Drivetrain.RR_LUT.add(0.0, robotContainer.strafeDrive.m_rearRight.getTurnEncDeg());
           // System.out.println("You have added 0 to the LUT");
        }
        if (robotContainer.CONTROLLER.getAButton()) {
            robotContainer.armControl.boomStart= robotContainer.armControl.boomEncoder.getPosition();
            robotContainer.armControl.dipperMax= robotContainer.armControl.dipperEncoder.getPosition();
            System.out.println("Boom Start is Now: "+ robotContainer.armControl.boomStart+"\n Dipper Max is Now: "+robotContainer.armControl.dipperMax);
            //robotContainer.strafeDrive.rottenest();
            //Drivetrain.FL_LUT.add(90.0, robotContainer.strafeDrive.m_frontLeft.getTurnEncDeg());
            //Drivetrain.RL_LUT.add(90.0, robotContainer.strafeDrive.m_rearLeft.getTurnEncDeg());
            //Drivetrain.FR_LUT.add(90.0, robotContainer.strafeDrive.m_frontRight.getTurnEncDeg());
            //Drivetrain.RR_LUT.add(90.0, robotContainer.strafeDrive.m_rearRight.getTurnEncDeg());
            //System.out.println("You have added 90 to the LUT");
        }
        if (robotContainer.CONTROLLER.getBButton()) {
            //robotContainer.strafeDrive.rottenest();
            Drivetrain.FL_LUT.add(180.0, robotContainer.strafeDrive.m_frontLeft.getTurnEncDeg());
            Drivetrain.RL_LUT.add(180.0, robotContainer.strafeDrive.m_rearLeft.getTurnEncDeg());
            Drivetrain.FR_LUT.add(180.0, robotContainer.strafeDrive.m_frontRight.getTurnEncDeg());
            Drivetrain.RR_LUT.add(180.0, robotContainer.strafeDrive.m_rearRight.getTurnEncDeg());
            System.out.println("You have added 180 to the LUT");
        }
        if (robotContainer.CONTROLLER.getXButton()) {
            //robotContainer.strafeDrive.rottenest();
            Drivetrain.FL_LUT.add(270.0, robotContainer.strafeDrive.m_frontLeft.getTurnEncDeg());
            Drivetrain.RL_LUT.add(270.0, robotContainer.strafeDrive.m_rearLeft.getTurnEncDeg());
            Drivetrain.FR_LUT.add(270.0, robotContainer.strafeDrive.m_frontRight.getTurnEncDeg());
            Drivetrain.RR_LUT.add(270.0, robotContainer.strafeDrive.m_rearRight.getTurnEncDeg());
            System.out.println("You have added 270 to the LUT");
        }
        if (robotContainer.CONTROLLER.getLeftLowerBumper()) {
            System.out.println("FL 0 " +Drivetrain.FL_LUT.get(0.0)+ "FR 0 " +Drivetrain.FR_LUT.get(0.0)+"RL 0 " +Drivetrain.RL_LUT.get(0.0)+"RR 0 " +Drivetrain.RR_LUT.get(0.0));
            System.out.println("FL 90 " +Drivetrain.FL_LUT.get(90.0)+ "FR 90 " +Drivetrain.FR_LUT.get(90.0)+"RL 90 " +Drivetrain.RL_LUT.get(90.0)+"RR 90 " +Drivetrain.RR_LUT.get(90.0));
            System.out.println("FL 180 " +Drivetrain.FL_LUT.get(180.0)+ "FR 0 " +Drivetrain.FR_LUT.get(0.0)+"RL 0 " +Drivetrain.RL_LUT.get(0.0)+"RR 0 " +Drivetrain.RR_LUT.get(0.0));
            System.out.println("FL 270 " +Drivetrain.FL_LUT.get(270.0)+ "FR 0 " +Drivetrain.FR_LUT.get(0.0)+"RL 0 " +Drivetrain.RL_LUT.get(0.0)+"RR 0 " +Drivetrain.RR_LUT.get(0.0));
        
        } 
        double armB=0.0, armD=0.0, armC=0.0;
        if (robotContainer.CONTROLLER.getRightUpperBumper()) { armB = -1; } else if (robotContainer.CONTROLLER.getRightLowerBumper()) { armB = 1; }
        if (robotContainer.CONTROLLER.getLeftUpperBumper()) { armD = -1; } else if (robotContainer.CONTROLLER.getLeftLowerBumper()) { armD = 1; }
        if (robotContainer.CONTROLLER.getAButton()) { armC = 1; } else if (robotContainer.CONTROLLER.getXButton()) { armC = -1; }
        robotContainer.armControl.setArmTest(armB, armD, armC);
        
    }
}
