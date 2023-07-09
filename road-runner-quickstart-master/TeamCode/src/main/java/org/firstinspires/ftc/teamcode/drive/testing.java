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
    private DcMotor motorlr, motorRR, motorLF, motorRF;


    @Override
    public void runOpMode() throws InterruptedException {
        //motorlr = hardwareMap.dcMotor.get('leftRear');
        //motorRR = hardwareMap.dcMotor.get('rightRear');
        //motorLF = hardwareMap.dcMotor.get('leftFront');
        //motorRF = hardwareMap.dcMotor.get('rightFront');

        //motorlr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motorlr.setPower(gamepad1.left_trigger);
        //motorRR.setPower(gamepad1.left_trigger);
        //motorLF.setPower(gamepad1.left_trigger);
        //motorRF.setPower(gamepad1.left_trigger);




    }
}