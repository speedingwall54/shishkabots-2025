package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
    // Motor controllers for the left side
    private final PWMSparkMax leftFrontMotor = new PWMSparkMax(0);
    private final PWMSparkMax leftRearMotor = new PWMSparkMax(1);

    // Motor controllers for the right side
    private final PWMSparkMax rightFrontMotor = new PWMSparkMax(2);
    private final PWMSparkMax rightRearMotor = new PWMSparkMax(3);

    // Differential drive
    private final DifferentialDrive drive;

    public DriveSubsystem() {
        // Invert the right side motors
        rightFrontMotor.setInverted(true);
        rightRearMotor.setInverted(true);

        // Create the differential drive controller
        drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
    }

    /**
     * Drive the robot using arcade controls.
     *
     * @param speed the commanded forward movement
     * @param rotation the commanded rotation
     */
    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
        // Set rear motors to match front motors
        leftRearMotor.set(leftFrontMotor.get());
        rightRearMotor.set(rightFrontMotor.get());
    }

    /**
     * Drive the robot using tank controls.
     *
     * @param leftSpeed the commanded left speed
     * @param rightSpeed the commanded right speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed);
        // Set rear motors to match front motors
        leftRearMotor.set(leftFrontMotor.get());
        rightRearMotor.set(rightFrontMotor.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Left Front Motor", leftFrontMotor.get());
        SmartDashboard.putNumber("Left Rear Motor", leftRearMotor.get());
        SmartDashboard.putNumber("Right Front Motor", rightFrontMotor.get());
        SmartDashboard.putNumber("Right Rear Motor", rightRearMotor.get());
    }

    /**
     * Stop the drive motors.
     */
    public void stop() {
        drive.stopMotor();
    }

    /**
     * Drive the robot using arcade controls.
     * 
     * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
     */
    public void set(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    /**
     * Drive the robot using tank controls.
     * 
     * @param left The speed of the left side of the robot [-1.0..1.0]
     * @param right The speed of the right side of the robot [-1.0..1.0]
     */
    public void tank(double left, double right) {
        drive.tankDrive(left, right);
    }
}
