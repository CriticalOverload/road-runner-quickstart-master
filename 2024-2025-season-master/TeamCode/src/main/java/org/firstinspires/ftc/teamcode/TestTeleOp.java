package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "AA Main TeleOp")
public class TestTeleOp extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLS, motorLS2, motorJoint;
    private DistanceSensor distSensor;
    private Servo servo, servoJoint, servoRoller;  // Added servoRoller
    private double powerMod = 0.8;
    private double slidePMod = 1.0;
    private double value = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorLS = hardwareMap.dcMotor.get("LS");
        motorLS2 = hardwareMap.dcMotor.get("LS2");
        motorJoint = hardwareMap.dcMotor.get("JM");
        servoJoint = hardwareMap.servo.get("JS");
        servoRoller = hardwareMap.servo.get("RS");  // Initialize servoRoller
        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        servo.setPosition(0);

        // Set zero power behavior and direction for motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLS2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            value = distSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Distance:", value);

            // Power scaling for driving
            if (gamepad1.right_bumper) {
                powerMod = 0.5;
                telemetry.addData("right bumper","gamepad1");
            } else if (gamepad1.left_bumper) {
                powerMod = 0.3;
                telemetry.addData("left bumper","gamepad1");
            } else {
                powerMod = 0.8;
            }

            // Power scaling for linear slides
            if (gamepad2.right_bumper) {
                slidePMod = 0.85;
                telemetry.addData("right bumper","gamepad2");
            } else if (gamepad2.left_bumper) {
                slidePMod = 0.35;
                telemetry.addData("left bumper","gamepad2");
            } else {
                slidePMod = 0.6;
            }

            // Mecanum drive calculations
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.sin(angle);
            double powerTwo = r * Math.cos(angle);

            motorFrontLeft.setPower((powerOne - rotation) * powerMod);
            motorFrontRight.setPower((powerTwo + rotation) * powerMod);
            motorBackLeft.setPower((powerTwo - rotation) * powerMod);
            motorBackRight.setPower((powerOne + rotation) * powerMod);
            motorLS.setPower(gamepad2.right_stick_y * slidePMod);
            motorLS2.setPower(-gamepad2.right_stick_y * slidePMod); // LS2 mirrors LS

            // Control motorJoint with gamepad2 left stick
            motorJoint.setPower(gamepad2.left_stick_y * powerMod); // Control motorJoint with left stick y

            // Control servoJoint with dpad buttons or triggers
            if (gamepad2.dpad_left) {
                servoJoint.setPosition(0.0);  // Move servoJoint to one extreme
            } else if (gamepad2.dpad_right) {
                servoJoint.setPosition(1.0);  // Move servoJoint to the other extreme
            }

            // Control servoRoller with gamepad2 buttons
            if (gamepad2.b) {
                servoRoller.setPosition(1.0);  // Full speed in one direction
            } else if (gamepad2.x) {
                servoRoller.setPosition(0.0);  // Stop the roller
            } else if (gamepad2.y) {
                servoRoller.setPosition(-1.0);  // Full speed in the opposite direction
            }

            // Claw control
            if (gamepad2.dpad_up) {
                servo.setPosition(0.0);
            } else if (gamepad2.dpad_down) {
                servo.setPosition(0.5);
            }

            if (gamepad2.a) {
                servo.setPosition(0.0);
            } else if (gamepad2.y) {
                servo.setPosition(0.5);
            }

            telemetry.addData("Slide position", motorLS.getCurrentPosition());
            telemetry.addData("Slide position", motorLS2.getCurrentPosition());
            telemetry.update();
        }
    }
}