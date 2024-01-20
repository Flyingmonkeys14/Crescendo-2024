// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.subsystems.StabilizerController;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.TeleopSwerve;
//import frc.robot.subsystems.StabilizerController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve m_Swerve = new Swerve();

  
  //public final Joystick m_operator = new Joystick(0);
  //public final Joystick m_driver = new Joystick(1);
  public final XboxController m_operator = new XboxController(1);
  public final XboxController m_driver = new XboxController(0);
/* Drive Controls */
// private final int translationAxis = Joystick.AxisType.kY.value;
// private final int strafeAxis = Joystick.AxisType.kX.value;
// private final int rotationAxis = Joystick.AxisType.kZ.value;

/* Driver Buttons */
// private final JoystickButton zeroGyro = new JoystickButton(m_driver, Joystick.ButtonType.kTrigger.value);
// private final JoystickButton robotCentric = new JoystickButton(m_driver, Joystick.ButtonType.kTop.value);

// private final Joystick m_driver = new Joystick(0);
// private final Joystick m_driver2 = new Joystick(1);

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public RobotContainer() {

  m_Swerve.setDefaultCommand(

  new TeleopSwerve(
    m_Swerve,
    () -> m_driver.getLeftX(),
    () -> m_driver.getLeftY(),
    () -> m_driver.getRightX(),
    () -> m_driver.getAButton())
);}

/**
 * Use this method to define your button-> command mappings. Buttons can be created by
 * instantiating a {@link GenericHID} or one of its subclasses ({@link
 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
 */
private void configureButtonBindings() {
  // Back button zeros the gyroscope
  // new Button(m_controller::getBackButton)
  //         // No requirements because we don't need to interrupt anything
  //          .whenPressed(m_Swerve::reset);

      /* Driver Buttons */
      // zeroGyro.onTrue(new InstantCommand(() -> m_Swerve.zeroGyro()));
    
}


  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, deadband:0.05);

    // square the axis
    value = Math.copySign(value * value, value);

    return value;

  }
}