// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SingleStrafeSubsystem;
import frc.robot.subsystems.StrafeSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final SingleStrafeSubsystem m_singleStrafeDrive = new SingleStrafeSubsystem();
  private final StrafeSubsystem strafeDrive = new StrafeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(Constants204.Controller.PORT);
  Joystick m_joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
  private void configureButtonBindings() {}

  public Command getTeleopSingleStrafeCommand() {
    return new RunCommand(
            () -> {
              if (m_driverController.getYButton()) {
                m_singleStrafeDrive.setZero();
              }

              m_singleStrafeDrive.strafe(m_driverController.getLeftX());
              //System.out.println("LX-CTRL: " + m_driverController.getLeftX());

            }, m_singleStrafeDrive);
  }

  public Command getTeleopStrafeCommand() {
      return new RunCommand(() -> {
          if (m_driverController.getYButton()) {
              strafeDrive.setZero();
          } else {
              strafeDrive.drive(m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
          }
      }, strafeDrive);
  }
}
