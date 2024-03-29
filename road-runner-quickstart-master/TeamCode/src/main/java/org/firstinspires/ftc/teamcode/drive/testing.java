package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "testing")
public class testing extends LinearOpMode {
    private DcMotor leftEncoder, rightEncoder, middleEncoder;


    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("frontEncoder");


        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("left", leftEncoder.getCurrentPosition());
            telemetry.addData("right", rightEncoder.getCurrentPosition());
            telemetry.addData("front", middleEncoder.getCurrentPosition());
            telemetry.update();
        }



    }
}