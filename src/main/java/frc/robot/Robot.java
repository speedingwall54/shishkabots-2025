// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  

  @Override
  public void robotInit() {
    // Enable data logging
    DataLogManager.start();
    
    // Log NetworkTables data
    DataLogManager.logNetworkTables(true);
    
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    
    // Instantiate our RobotContainer
    robotContainer = new RobotContainer();
=======

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
<<<<<<< HEAD
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
=======
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
<<<<<<< HEAD
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
=======
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
<<<<<<< HEAD
}
=======
}
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
