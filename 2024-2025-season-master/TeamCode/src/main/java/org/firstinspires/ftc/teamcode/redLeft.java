package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//afdfasdfa
@Autonomous(name="CustomAuto", group="Autonomous")
public class CustomAuto extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    static final double COUNTS_PER_MOTOR_REV = 537.6; // Example for goBILDA 5202 motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;   // No gear reduction
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // For a 4" wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // Conversion factor: 1 block = 24 inches
    static final double COUNTS_PER_BLOCK = COUNTS_PER_INCH * 24; // 1 block = 24 inches

    @Override
    public void runOpMode() {

        // Initialize the hardware
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightRear = hardwareMap.get(DcMotor.class, "BR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        
        resetEncoders();

        waitForStart();  // Wait for the game to start (driver presses PLAY)

        // Sequence of movements
        // 1. Move forward 2.3 blocks
        driveStraight(2.3);

        // 2. Strafe to the left 0.35 blocks
        strafeLeft(0.35);

        // 3. Turn 180 degrees to the left
        turnLeft(180);

        // 4. Move forward 2.2 blocks
        driveStraight(2.2);

        // 5. Move backward 2.2 blocks
        driveStraight(-2.2);

        // 6. Strafe to the left 0.3 blocks
        strafeLeft(0.3);

        // 7. Move forward 2.2 blocks
        driveStraight(2.2);

        // 8. Move backward 2.2 blocks
        driveStraight(-2.2);

        // 9. Strafe to the left 0.28 blocks
        strafeLeft(0.28);

        // 10. Move forward 2.2 blocks
        driveStraight(2.2);

        // 11. Move backward 2.2 blocks
        driveStraight(-2.2);

        // 12. Turn 90 degrees to the left
        turnLeft(90);

        // 13. Strafe to the right 0.4 blocks
        strafeRight(0.4);

        // 14. Move forward 1.5 blocks
        driveStraight(1.5);

        // Park
    }

    // Helper method to reset encoders
    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Method to drive straight
    public void driveStraight(double blocks) {
        int targetPosition = (int) (blocks * COUNTS_PER_BLOCK);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.5);  // Adjust power as needed

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
            telemetry.addData("Driving", "Straight");
            telemetry.update();
        }

        setMotorPower(0);  // Stop motors
    }

    // Method to strafe left
    public void strafeLeft(double blocks) {
        int targetPosition = (int) (blocks * COUNTS_PER_BLOCK);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() - targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() + targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() - targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.5);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
            telemetry.addData("Strafing", "Left");
            telemetry.update();
        }

        setMotorPower(0);
    }

    // Method to strafe right
    public void strafeRight(double blocks) {
        int targetPosition = (int) (blocks * COUNTS_PER_BLOCK);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + targetPosition);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - targetPosition);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - targetPosition);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + targetPosition);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.5);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
            telemetry.addData("Strafing", "Right");
            telemetry.update();
        }

        setMotorPower(0);
    }

    // Method to turn left by a certain angle
    public void turnLeft(double degrees) {
        // Assuming 360 degrees corresponds to a specific encoder value for your robot
        int turnCounts = (int) ((degrees / 360) * COUNTS_PER_BLOCK);  // You can adjust this scaling based on testing

        leftFront.setTargetPosition(leftFront.getCurrentPosition() - turnCounts);
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - turnCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + turnCounts);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + turnCounts);

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setMotorPower(0.5);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy()) {
            telemetry.addData("Turning", "Left");
            telemetry.update();
        }

        setMotorPower(0);
    }

    // Set motor mode
    public void setMotorMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    // Set motor power
    public void setMotorPower(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
    }
}