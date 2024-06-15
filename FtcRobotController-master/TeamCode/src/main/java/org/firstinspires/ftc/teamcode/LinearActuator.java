package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LinearActuator extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLA, motorLAL;
    //private DistanceSensor distSensor;
    private double powerMod = 0.8;
    private double liftPMod = 0.8;
    private double value = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        //jointServo = hardwareMap.servo.get("JS");
        //servo = hardwareMap.servo.get("claw");
        motorLA = hardwareMap.dcMotor.get("LA");
        motorLAL = hardwareMap.dcMotor.get("LAL");
        //distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        //jointServo.setPosition(0);

        //robot class?????????????????????????????

        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLAL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()){
            //value = distSensor.getDistance(DistanceUnit.INCH);
            //telemetry.addData("Distance:", value);

            //Slows down or speeds up motors for wheels
            if(gamepad1.right_bumper) {
                powerMod = 0.5;
                telemetry.addData("right bumper","gamepad1");
            }else if(gamepad1.left_bumper){
                telemetry.addData("left bumper","gamepad1");
                powerMod = 0.3;
            }else{
                powerMod = 0.8;
            }

            //slows down or speeds up motors for linear slide.
            if(gamepad2.right_bumper) {
                liftPMod = 0.75;
                telemetry.addData("right bumper","gamepad2");
            }else if(gamepad2.left_bumper){
                liftPMod = 0.55;
                telemetry.addData("left bumper","gamepad2");
            }else{
                liftPMod = 0.6;
            }

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);
//            double powerOne = r*Math.cos(angle);
//            double powerTwo = r*Math.sin(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);
            motorLA.setPower(gamepad2.right_stick_y * liftPMod);
            motorLAL.setPower(gamepad2.left_stick_y * liftPMod);


            //Joint Servo code


            //CODE FOR CLAW WHEN WIRED IN
            /*if (gamepad2.a) {
                claw.setPosition(0.0);
            }
            else if (gamepad2.y) {
                claw.setPosition(0.5);
            }*/
            telemetry.addData("Actuator position",motorLAL.getCurrentPosition());
            telemetry.update();


        }


    }
}
