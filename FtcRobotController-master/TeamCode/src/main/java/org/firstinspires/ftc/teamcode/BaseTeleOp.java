package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//@TeleOp(name = "BASE TELEOP")
public class BaseTeleOp extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;
    private double powerMod;
    //private DistanceSensor distSensor;


    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        //distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        //jointServo.setPosition(0);

        //robot class?????????????????????????????

        //reverse the needed motors
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){
            //value = distSensor.getDistance(DistanceUnit.INCH);
            //telemetry.addData("Distance:", value);

            //Slows down or speeds up motors for wheels
            if(gamepad1.right_bumper) {
                powerMod = 1.0;
                telemetry.addData("right bumper","gamepad1");
            }else if(gamepad1.left_bumper){
                telemetry.addData("left bumper","gamepad1");
                powerMod = 0.6;
            }else{
                powerMod = 0.9;
            }




            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);
//            double powerOne = r*Math.cos(angle);
//            double powerTwo = r*Math.sin(angle);

//            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
//            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
//            motorBackLeft.setPower((powerTwo - (rotation))*(powerMod- 0.1));
//            motorBackRight.setPower((powerOne + (rotation))*(powerMod-0.1));

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*(powerMod));
            motorBackRight.setPower((powerOne + (rotation))*(powerMod ));

            //moves linear accuator up and down
//            if (gamepad2.right_trigger || ) {
//                motorLinearAccuator.setPower(0.4);
//            }
//            else if (gamepad2.dpad_left){
//                motorLinearAccuator.setPower(-0.3);
//            }
//            else {
//                motorLinearAccuator.setPower(0);
//
//            }




        }


    }

}
