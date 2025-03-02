package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
    private final PWMSparkMax motor;
    private final Encoder encoder;
    private final PIDController pidController;
    private final DigitalInput topLimit;
    private final DigitalInput bottomLimit;
    private double target = 0.0;
    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SPEED = -1.0;
    private static final double TOLERANCE = 0.5;
    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    public ElevatorSubsystem(int motorId, int topLimitId, int bottomLimitId) {
        motor = new PWMSparkMax(motorId);
        encoder = new Encoder(0, 1);
        pidController = new PIDController(kP, kI, kD);
        topLimit = new DigitalInput(topLimitId);
        bottomLimit = new DigitalInput(bottomLimitId);
        encoder.reset();
    }

    public void setPosition(double position) {
        target = position;
        pidController.setSetpoint(target);
    }

    public double getPosition() {
        return encoder.getDistance();
    }

    public void setSpeed(double speed) {
        if ((speed > 0 && atTop()) || (speed < 0 && atBottom())) {
            stop();
            return;
        }
        motor.set(Math.max(MIN_SPEED, Math.min(MAX_SPEED, speed)));
    }

    public boolean atTop() {
        return !topLimit.get();
    }

    public boolean atBottom() {
        return !bottomLimit.get();
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - target) < TOLERANCE;
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPosition());
        SmartDashboard.putNumber("Target Position", target);
        SmartDashboard.putBoolean("At Top", atTop());
        SmartDashboard.putBoolean("At Bottom", atBottom());
        SmartDashboard.putNumber("Motor Power", motor.get());  
    }

    public void simulate() {
        double output = pidController.calculate(getPosition(), target);
        motor.set(Math.max(MIN_SPEED, Math.min(MAX_SPEED, output)));
    }
}
