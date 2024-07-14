package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
@Autonomous(name = "Practice AUTO")

public class testAuto extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;

    private int signal = 3;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        slides = hardwareMap.dcMotor.get("LS");
        // signal = 2;


        imu = hardwareMap.get(BNO055IMU.class, "imu 1");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR,slides, imu, this, false);
        robot.setupRobot();

        while(!opModeIsActive()){
            telemetry.addLine("signal");
            telemetry.update();
        }
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        //todo: test and update
        //also roadrunner...

        /*
        robot.gyroStrafeEncoder(0.7, 90, 24);
        robot.gyroTurn(90, 0.5);
        robot.gyroStrafeEncoder(0.7, 90, 24);
        robot.gyroStrafeEncoder(0.5, 24, -90);
        robot.gyroTurn(-90, 0.5);
        */
        //robot.skibatry();
        /*robot.gyroStrafeEncoder(0.7,90,45);
        robot.gyroTurn(80,0.7);
        robot.gyroStrafeEncoder(0.7,90,20);
        robot.gyroTurn(-90,0.4);
        robot.gyroStrafeEncoder(0.7,90,48);
        telemetry.addLine("turn 1");*/
        robot.skibiSLide(2,0.4);

    }
}
