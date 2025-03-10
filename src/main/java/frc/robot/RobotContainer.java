// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
=======
import edu.wpi.first.wpilibj.XboxController;
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
<<<<<<< HEAD
import frc.robot.subsystems.ElevatorSubsystem;
=======
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
<<<<<<< HEAD
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(0, 1, 2);
  private final SendableChooser<Command> autoChooser;

  // The driver's controllers
  private final XboxController xboxController = new XboxController(1);
  private final PS4Controller ps4Controller = new PS4Controller(0);

  // Set which controller to use (true for Xbox, false for PS4)
  private final boolean useXboxController = true;

  private static final double DEADBAND = 0.1;

  private double applyDeadband(double value) {
    if (Math.abs(value) < DEADBAND) {
      return 0.0;
    }
    return value;
  }

  private double getForwardInput() {
    double raw = useXboxController ? -xboxController.getLeftY() : -ps4Controller.getLeftY();
    return applyDeadband(raw);
  }

  private double getStrafeInput() {
    double raw = useXboxController ? -xboxController.getLeftX() : -ps4Controller.getLeftX();
    return applyDeadband(raw);
  }

  private double getRotationInput() {
    double raw = useXboxController ? -xboxController.getRightX() : -ps4Controller.getRightX();
    return applyDeadband(raw);
  }

  public RobotContainer() {
    configureBindings();
=======

  // The driver's controller
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    configureBindings();

>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
    // Set up the default command for the drive subsystem
    driveSubsystem.setDefaultCommand(
        new DefaultDriveCommand(
            driveSubsystem,
<<<<<<< HEAD
            () -> getForwardInput() * 0.5,  // Forward/backward
            () -> getStrafeInput() * 0.5,   // Left/right
            () -> getRotationInput() * 0.5  // Rotation
        )
    );

    autoChooser = AutoBuilder.buildAutoChooser("bottom-start");
  }

  private void configureBindings() {
    // Xbox Controller Bindings
    new JoystickButton(xboxController, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.stop()));
    
    new JoystickButton(xboxController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new DefaultDriveCommand(
                driveSubsystem,
                () -> getForwardInput() * 0.25,
                () -> getStrafeInput() * 0.25,
                () -> getRotationInput() * 0.25
            )
        );

    // PS4 Controller Bindings
    new JoystickButton(ps4Controller, PS4Controller.Button.kCircle.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.stop()));
    
    new JoystickButton(ps4Controller, PS4Controller.Button.kR1.value)
        .whileTrue(
            new DefaultDriveCommand(
                driveSubsystem,
                () -> getForwardInput() * 0.25,
                () -> getStrafeInput() * 0.25,
                () -> getRotationInput() * 0.25
=======
            () -> -driverController.getLeftY(),  // Forward/backward (inverted because forward is negative on the controller)
            () -> -driverController.getRightX()  // Rotation (inverted because right is positive on the controller)
        )
    );
  }

  private void configureBindings() {
    // Add button bindings here
    // Stop the robot when the B button is pressed
    new JoystickButton(driverController, XboxController.Button.kB.value)
        .onTrue(Commands.runOnce(() -> driveSubsystem.stop()));
    
    // Half speed mode while holding right bumper
    new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(
            new DefaultDriveCommand(
                driveSubsystem,
                () -> -driverController.getLeftY() * 0.5,  // Half speed forward/backward
                () -> -driverController.getRightX() * 0.5   // Half speed rotation
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
            )
        );
  }

<<<<<<< HEAD
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create and return the autonomous command
    //return new AutonomousCommand(driveSubsystem, 2.0);
    return autoChooser.getSelected();
  }
}
=======
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
