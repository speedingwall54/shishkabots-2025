package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
<<<<<<< HEAD
 * Default command for the drive subsystem that implements swerve drive control.
 * This command will run by default whenever the drive subsystem is not being used by another command.
 * It takes continuous input from three double suppliers (typically joystick axes) to control the robot's
 * forward/backward movement, left/right movement, and rotation.
 */
public class DefaultDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
=======
 * Default command for the drive subsystem that implements arcade drive control.
 * This command will run by default whenever the drive subsystem is not being used by another command.
 * It takes continuous input from two double suppliers (typically joystick axes) to control the robot's
 * forward/backward movement and rotation.
 */
public class DefaultDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier speedSupplier;
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
    private final DoubleSupplier rotationSupplier;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command will run on
<<<<<<< HEAD
     * @param xSpeed The forward/backward speed supplier
     * @param ySpeed The left/right speed supplier
     * @param rotation The rotation speed supplier
     */
    public DefaultDriveCommand(
            DriveSubsystem subsystem,
            DoubleSupplier xSpeed,
            DoubleSupplier ySpeed,
            DoubleSupplier rotation) {
        driveSubsystem = subsystem;
        xSpeedSupplier = xSpeed;
        ySpeedSupplier = ySpeed;
=======
     * @param speed The forward/backward speed to drive at
     * @param rotation The rotation speed to turn at
     */
    public DefaultDriveCommand(DriveSubsystem subsystem, DoubleSupplier speed, DoubleSupplier rotation) {
        driveSubsystem = subsystem;
        speedSupplier = speed;
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
        rotationSupplier = rotation;
        addRequirements(subsystem); // ensures command has exclusive use of the drive subsystem. 
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
    }

    @Override
    public void execute() {
<<<<<<< HEAD
        driveSubsystem.drive(
            xSpeedSupplier.getAsDouble(),
            ySpeedSupplier.getAsDouble(),
            rotationSupplier.getAsDouble());
=======
        driveSubsystem.arcadeDrive(speedSupplier.getAsDouble(), rotationSupplier.getAsDouble());
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // This command will run forever as we want to keep the robot moving
        return false;
    }
<<<<<<< HEAD
}
=======
}
>>>>>>> de537cc54d7feec1fe24553cac110767a5757556
