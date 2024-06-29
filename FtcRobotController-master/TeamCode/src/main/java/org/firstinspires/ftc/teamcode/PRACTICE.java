package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PracOp")
public class PRACTICE extends LinearOpMode {

    private DcMotor bub;

    @Override
    public void runOpMode() throws InterruptedException {
        bub = hardwareMap.dcMotor.get("bubble");

        waitForStart();

        while(opModeIsActive()){

            bub.setPower(gamepad1.right_stick_x);
            telemetry.addData("Position", bub.getCurrentPosition());
            telemetry.update();
        }

    }

}
