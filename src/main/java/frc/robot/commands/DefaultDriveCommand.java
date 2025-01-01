package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Default command for the drive subsystem that implements arcade drive control.
 * This command will run by default whenever the drive subsystem is not being used by another command.
 * It takes continuous input from two double suppliers (typically joystick axes) to control the robot's
 * forward/backward movement and rotation.
 */
public class DefaultDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier rotationSupplier;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param subsystem The drive subsystem this command will run on
     * @param speed The forward/backward speed to drive at
     * @param rotation The rotation speed to turn at
     */
    public DefaultDriveCommand(DriveSubsystem subsystem, DoubleSupplier speed, DoubleSupplier rotation) {
        driveSubsystem = subsystem;
        speedSupplier = speed;
        rotationSupplier = rotation;
        addRequirements(subsystem); // ensures command has exclusive use of the drive subsystem. 
    }

    @Override
    public void initialize() {
        driveSubsystem.stop();
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(speedSupplier.getAsDouble(), rotationSupplier.getAsDouble());
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
}
