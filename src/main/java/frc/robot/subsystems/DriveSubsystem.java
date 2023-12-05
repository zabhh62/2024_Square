package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final LightsSubsystem lightsSubsystem;

    // The motors on the left side of the drive.
    private final TalonSRX        leftPrimaryMotor         = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT);
    private final TalonSRX        leftFollowerMotor        = new TalonSRX(DriveConstants.LEFT_MOTOR_PORT + 1);

    // The motors on the right side of the drive.
    private final TalonSRX        rightPrimaryMotor        = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT);
    private final TalonSRX        rightFollowerMotor       = new TalonSRX(DriveConstants.RIGHT_MOTOR_PORT + 1);

    private final DigitalInput    targetSensor             = new DigitalInput(0);

    // Conversion from volts to distance in cm
    // Volts distance
    // 0.12 30.5 cm
    // 2.245 609.6 cm
    private final AnalogInput     ultrasonicDistanceSensor = new AnalogInput(0);

    private final double          ULTRASONIC_M             = (609.6 - 30.5) / (2.245 - .12);
    private final double          ULTRASONIC_B             = 609.6 - ULTRASONIC_M * 2.245;


    private double                leftSpeed                = 0;
    private double                rightSpeed               = 0;

    private AHRS                  navXGyro                 = new AHRS() {
                                                               // Override the "Value" in the gyro
                                                               // sendable to use the angle instead of
                                                               // the yaw.
                                                               // Using the angle makes the gyro appear
                                                               // in the correct position accounting
                                                               // for the
                                                               // offset. The yaw is the raw sensor
                                                               // value which appears incorrectly on
                                                               // the dashboard.
                                                               @Override
                                                               public void initSendable(SendableBuilder builder) {
                                                                   builder.setSmartDashboardType("Gyro");
                                                                   builder.addDoubleProperty("Value", this::getAngle, null);
                                                               }
                                                           };

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        leftPrimaryMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);
        leftFollowerMotor.setInverted(DriveConstants.LEFT_MOTOR_REVERSED);

        leftPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        leftFollowerMotor.setNeutralMode(NeutralMode.Brake);

        leftFollowerMotor.follow(leftPrimaryMotor);


        rightPrimaryMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);
        rightFollowerMotor.setInverted(DriveConstants.RIGHT_MOTOR_REVERSED);

        rightPrimaryMotor.setNeutralMode(NeutralMode.Brake);
        rightFollowerMotor.setNeutralMode(NeutralMode.Brake);

        rightFollowerMotor.follow(rightPrimaryMotor);
    }

    public double getUltrasonicDistanceCm() {

        double ultrasonicVoltage = ultrasonicDistanceSensor.getVoltage();

        double distanceCm        = ULTRASONIC_M * ultrasonicVoltage + ULTRASONIC_B;

        return Math.round(distanceCm);
    }

    /**
     * Set the left and right speed of the primary and follower motors
     *
     * @param leftSpeed
     * @param rightSpeed
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {

        this.leftSpeed  = leftSpeed;
        this.rightSpeed = rightSpeed;

        leftPrimaryMotor.set(ControlMode.PercentOutput, leftSpeed);
        rightPrimaryMotor.set(ControlMode.PercentOutput, rightSpeed);

        // NOTE: The follower motors are set to follow the primary
        // motors
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0, 0);
    }

    public boolean isTargetDetected() {
        return !targetSensor.get();
    }

    @Override
    public void periodic() {

        lightsSubsystem.setProximity(isTargetDetected());
        lightsSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("Right Motor", rightSpeed);
        SmartDashboard.putNumber("Left  Motor", leftSpeed);

        SmartDashboard.putNumber("Ultrasonic Voltage", ultrasonicDistanceSensor.getVoltage());
        SmartDashboard.putNumber("Ultrasonic Distance (cm)", getUltrasonicDistanceCm());

        SmartDashboard.putData("Gyro", navXGyro);
    }
}
