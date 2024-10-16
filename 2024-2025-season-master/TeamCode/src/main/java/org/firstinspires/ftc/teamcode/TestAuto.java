package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Good Auto", group = "LinearOpMode")
    public class GoodAuto extends LinearOpMode {
    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo claw;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");
        claw = hardwareMap.servo.get("claw");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        // Reverse right motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        setMotorBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the Start button to be pressed
        waitForStart();

        // Autonomous steps
        moveForward(24, 0.6);  // Move forward 24 inches
    

        telemetry.addLine("moved 24 inches");
        telemetry.update();
    }

    // Helper: Set motor behavior to BRAKE
    private void setMotorBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    // Helper: Move forward a specific distance (in inches)
    private void moveForward(double inches, double power) throws InterruptedException {
        int ticks = inchesToTicks(inches);
        setTargetPosition(ticks);
        setMotorPower(power);
        waitUntilMotorsReachTarget();
        stopMotors();
    }

    // Helper: Move backward
    private void moveBackward(double inches, double power) throws InterruptedException {
        moveForward(-inches, power);
    }

    // Helper: Turn the robot a specific angle (in degrees)
    private void turn(int degrees, double power) throws InterruptedException {
        int ticks = (int)(degrees * 10);  // Adjust factor for turn tuning
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);
        setMotorPower(power);
        waitUntilMotorsReachTarget();
        stopMotors();
    }

    // Helper: Drop an object using the claw
    private void dropClaw() {
        claw.setPosition(0.5);  // Open claw halfway to release object
        sleep(500);             // Wait for the servo to finish moving
    }

    // Helper: Check if an obstacle is detected within 10 inches
    private boolean checkObstacle() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Distance to obstacle:", distance);
        telemetry.update();
        return distance < 10;
    }

    // Helper: Convert inches to motor ticks
    private int inchesToTicks(double inches) {
        final double TICKS_PER_REV = 537.6;  // Example for a GoBilda motor
        final double WHEEL_DIAMETER_INCHES = 4.0;
        double circumference = Math.PI * WHEEL_DIAMETER_INCHES;
        double rotations = inches / circumference;
        return (int)(rotations * TICKS_PER_REV);
    }

    // Helper: Set target position for all motors
    private void setTargetPosition(int ticks) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() + ticks);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Helper: Set motor power
    private void setMotorPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    // Helper: Stop all motors
    private void stopMotors() {
        setMotorPower(0);
    }

    // Helper: Wait until all motors reach their target
    private void waitUntilMotorsReachTarget() {
        while (opModeIsActive() &&
               (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Front Left Position", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Position", frontRight.getCurrentPosition());
            telemetry.update();
        }
    }
}