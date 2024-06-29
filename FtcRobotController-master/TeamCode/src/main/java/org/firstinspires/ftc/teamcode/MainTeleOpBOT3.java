package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MainTeleOpBOT3 extends LinearOpMode {
    private DcMotor motorFrontRight, motorBackLeft, motorFrontLeft, motorBackRight, motorLS;
    private double powerMod;
    private double slidePMod = 1.0;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorLS = hardwareMap.dcMotor.get("RLS");

        double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (double)(Math.PI/4);
        double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rotation = gamepad1.left_stick_x * -1;//changerd this

        double powerOne = (double)(r)*Math.sin(angle);
        double powerTwo =(double)(r)*Math.cos(angle);

        motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
        motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
        motorBackLeft.setPower((powerTwo - (rotation))*(powerMod));
        motorBackRight.setPower((powerOne + (rotation))*(powerMod));

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLS.setPower((gamepad2.right_stick_y * slidePMod));
    }
}

