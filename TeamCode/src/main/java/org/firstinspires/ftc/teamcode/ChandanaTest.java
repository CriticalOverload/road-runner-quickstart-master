package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

// Specify the opmode type
@TeleOp(name = "ChandanaTest")
public class ChandanaTest extends LinearOpMode {

    //4 wheel motors
    private DcMotor motorFR, motorFL, motorBL, motorBR;

    //1 motor for flywheel
    private DcMotor motorOUT;

    //intake
    private CRServo leftIntake, rightIntake;

    //sensor
    private UltrasonicSensor usSensor;

    //motor
    private DcMotor arm;

    //claw
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorOUT = hardwareMap.dcMotor.get("motorOUT");
        leftIntake = hardwareMap.crservo.get("leftIntake");
        rightIntake = hardwareMap.crservo.get("rightIntake");

        claw = hardwareMap.servo.get("claw");
        usSensor = hardwareMap.ultrasonicSensor.get("usSensor");

        double powerMod = 1.0;
        double intakeMod = 1.0;
        double outtakeMod = 0.64;
        double armMod = 0.3;

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.right_bumper) {
                powerMod = 0.5;
            }
            else {
                powerMod = 1.0;
            }

            double intakeSpeed = gamepad1.left_trigger * intakeMod;
            leftIntake.setPower(intakeSpeed);
            rightIntake.setPower(intakeSpeed);

            if(gamepad1.a) {
                intakeMod = -1.0;
            }
            else {
                intakeMod = 1.0;
            }

            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFL.setPower((powerOne - (rotation)) * powerMod);
            motorFR.setPower((powerTwo + (rotation)) * powerMod);
            motorBL.setPower((powerTwo - (rotation)) * powerMod);
            motorBR.setPower((powerOne + (rotation)) * powerMod);

            if(gamepad2.x) {
                claw.setPosition(1); //open
            }
            if(gamepad2.y) {
                claw.setPosition(0); //close
            }
            /*usSensor.getUltrasonicLevel() > 0.6 {
                leftIntake.setPower(1);
                rightIntake.setPower(1);
                Thread.sleep(1000);
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }*/
        }

    }



}
