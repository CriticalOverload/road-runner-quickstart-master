package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "AutonomousExample2")
public class AutonomousExample2 extends LinearOpMode {
    // Define your hardware components (e.g., motors, servos, etc.) here
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private double wheelRadius = 1.88976;
    private double ticksPerRevolution =  ((((1+(46/17))) * (1+(46/17))) * 28)	;


    // ... Other code ...

    @Override
    public void runOpMode() {
        // Initialization code here...
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        //reverse the needed motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Move the robot 7 inches forward
        moveForward(7.0); // 7 inches

        // Other autonomous tasks...

        // Stop the motors
        stopMotors();

        // End the autonomous op mode
        // ...
    }

    // Method to move the robot forward a specified distance (in inches)
    private void moveForward(double distanceInInches) {
        // Calculate the target encoder counts based on the given distance
        double targetCounts = (distanceInInches / (2 * Math.PI * wheelRadius)) * ticksPerRevolution;

        // Reset the encoder positions
        resetEncoders();

        // Set motor power for forward motion
        setMotorPower(0.5); // Adjust the power as needed

        // Monitor the encoder values until the target is reached
        while (opModeIsActive() && rightFront.getCurrentPosition() < targetCounts && rightFront.getCurrentPosition() < targetCounts) {
            // Continue moving
        }

        // Stop the motors
        stopMotors();
    }

    // Reset motor encoders
    private void resetEncoders() {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Set motor power for all motors
    private void setMotorPower(double power) {
        rightFront.setPower(power);
        leftFront.setPower(power);
        leftFront.setPower(power);
        leftRear.setPower(power);
    }

    // Stop all motors
    private void stopMotors() {
        setMotorPower(0.0);
    }

    // ... Other methods and code ...
}
