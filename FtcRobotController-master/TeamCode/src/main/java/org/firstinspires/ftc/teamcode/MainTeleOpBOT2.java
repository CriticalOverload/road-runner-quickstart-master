package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "25/7 Teleop")
public class MainTeleOpBOT2 extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLSLeft,motorLSRight, motorRoller;
    //private DistanceSensor distSensor;
    //it declares it in the file dc motor is the variable that your modifying\
    private Servo servo, jointServo, clawServo, rocket;
    private double powerMod = 0.8;
    private double slidePMod = 1.0;
    private double value = 1;
    private double jointServoPosition = 0.0;



    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        clawServo = hardwareMap.servo.get("claw");
        jointServo = hardwareMap.servo.get("JS");
        motorLSLeft = hardwareMap.dcMotor.get("LLS");
        motorLSRight = hardwareMap.dcMotor.get("RLS");
        motorRoller = hardwareMap.dcMotor.get("ROLL");
        rocket = hardwareMap.servo.get("paper");



        //jointServo.setPosition(0);

        //reverse the needed motors

        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLSLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLSRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        waitForStart();

        while(opModeIsActive()){
            //value = distSensor.getDistance(DistanceUnit.INCH);
            //telemetry.addData("Distance:", value);

            //Slows down or speeds up motors for wheels
            if(gamepad1.right_bumper) {
                powerMod = 0.8;
                telemetry.addData("right bumper","gamepad1");
            }else if(gamepad1.left_bumper){
                telemetry.addData("left bumper","gamepad1");
                powerMod = 0.6;}
            else if (gamepad1.a) {
                powerMod = 1.0;
                }
            else if (gamepad1.x) {
                powerMod = -1.0;
            }
            else{
                powerMod = -0.8;
            }

            //slows down or speeds up motors for linear slide.
            if(gamepad2.right_bumper) {
                slidePMod = 0.9;
                telemetry.addData("right bumper","gamepad2");
            }else if(gamepad2.left_bumper){
                slidePMod = 0.45;
                telemetry.addData("left bumper","gamepad2");
            }else{
                slidePMod = 0.5;
            }

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (double)(Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x * -1;//changerd this

            double powerOne = (double)(r)*Math.sin(angle);
            double powerTwo =(double)(r)*Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*(powerMod));
            motorBackRight.setPower((powerOne + (rotation))*(powerMod ));

            motorLSLeft.setPower(-(gamepad2.right_stick_y) * slidePMod);
            motorLSRight.setPower((gamepad2.right_stick_y * slidePMod));

//            moves linear accuator up and down
//            if (gamepad2.right_trigger || ) {
//                motorLinearAccuator.setPower(0.4);
//            }
//            else if (gamepad2.dpad_left){
//                motorLinearAccuator.setPower(-0.3);
//            }
//            else {
//                motorLinearAccuator.setPower(0);
//            }


            //claw Servo code
            if (gamepad2.b) {
                //telemetry.addLine("b");
                //clawServo.setPosition(0.7);
                clawServo.setPosition(0.55);
            }
            else if (gamepad2.x) {
                //telemetry.addLine("x");
                //clawServo.setPosition(0.32);
                clawServo.setPosition(0.47);
            }


//            joint servo
            if (gamepad2.y){
                telemetry.addLine("y button pressed");
                jointServo.setPosition(0.3);
            }
            else if (gamepad2.dpad_down) {
                jointServo.setPosition(1.0);
            }
            else if (gamepad2.dpad_left){
                jointServo.setPosition(1);
            }
            else if (gamepad2.a){
                jointServo.setPosition(0.65);

            }



            //CODE FOR ROCKET LAUNCHER
            if (gamepad2.right_trigger >0 && gamepad2.dpad_up) {
                rocket.setPosition(0.7);
            }
            else if (gamepad2.left_trigger >0 &&  gamepad2.dpad_down) {
                rocket.setPosition(0.0);
            }
            else{
                rocket.setPosition(0.7);
            }
            telemetry.addData("Slide position Left",motorLSLeft.getCurrentPosition());
            telemetry.addData("Slide position Right",motorLSRight.getCurrentPosition());
            telemetry.addData("Motor roller position", motorRoller.getCurrentPosition());
            telemetry.update();


        }


    }

}