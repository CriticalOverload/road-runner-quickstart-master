package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "testingv3")
public class testingv3 extends LinearOpMode {
    private DcMotor parallelEncoder, perpendicularEncoder;


    @Override
    public void runOpMode() throws InterruptedException {
        parallelEncoder = hardwareMap.dcMotor.get("parallelEncoder");
        perpendicularEncoder = hardwareMap.dcMotor.get("perpendicularEncoder");


        waitForStart();
        parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(opModeIsActive()){
            telemetry.addData("parallel", parallelEncoder.getCurrentPosition());
            telemetry.addData("perpendicular", perpendicularEncoder.getCurrentPosition());
            telemetry.update();
        }



    }
}
