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
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueRight")
public class BlueRight extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR, linearSlide;
    private Servo  jointServo, clawServo ;

    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;
    private int signal = 0;



    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        clawServo = hardwareMap.servo.get("claw");
        jointServo = hardwareMap.servo.get("JS");
        linearSlide = hardwareMap.dcMotor.get("LS");
        // signal = 2;
        //int signal = 1;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, linearSlide, imu, this, false);
        robot.setupRobot();


//        while(!opModeIsActive()){
//            telemetry.addLine("signal");
//            telemetry.update();
//        }
//        signal = mainPipeline.getSignal();
//        telemetry.addData("signal",signal);
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        //todo: test and update
        //also roadrunner...

        //attempt to place 2 things for case 2
        clawServo.setPosition(0.35);
        robot.moveSlides("move", 0.6);
        robot.gyroStrafeEncoder(0.6, 90, 31);
        // each zone
        clawServo.setPosition(0.7);
        sleep(1000);
        robot.moveSlides("zone", 0.3);
        clawServo.setPosition(0.35);
        sleep(1000);
        // end zone
        robot.gyroStrafeEncoder(0.6, -90, 7);
        robot.gyroTurn(87, 0.6);
        robot.gyroStrafeEncoder(0.6, 89, 83);
        robot.moveSlides("backboard", 0.4);
        jointServo.setPosition(0.4);
        robot.gyroStrafeEncoder(0.3, 90, 4);
        clawServo.setPosition(0.7);
        sleep(1000);
        robot.gyroStrafeEncoder(0.6, 270, 2);

        //29 in before the backboard
//        robot.moveSlides('g',0.5 );
//        robot.gyroStrafeEncoder(0.6, 90, 15);
//        robot.moveSlides('f', 0.4);
//        jointServo.setPosition(0.4);
//        robot.gyroStrafeEncoder(0.3, 90, 3);
//        clawServo.setPosition(0.7);
//        sleep(2000);
//        robot.gyroStrafeEncoder(0.3, 270, 3);






    }
}
