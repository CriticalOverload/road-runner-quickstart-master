package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="MoyBlueLong")
public class MoyeBlueLong extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorLSLeft,motorLSRight, motorRoller;
    //private DistanceSensor distSensor;
    private Servo servo, jointServo, clawServo, rocket;
    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;


    CVClassBlue mainPipeline;


    OpenCvWebcam webcam1 = null;


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
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        robot = new RobotClass3(motorFrontLeft,motorFrontRight, motorBackLeft, motorBackRight, motorLSRight, motorLSLeft, clawServo, jointServo, imu, this, false);
        robot.setupRobot();

//        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
//        mainPipeline = new CVClass();//create new pipeline
//
//        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
////
        CVClassBlue mainPipeline = new CVClassBlue();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);
        webcam1.setPipeline(mainPipeline);


        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);//Check our camera dimensions
            }

            public void onError(int errorCode) {


            }
        });


        //<<<<<<<<<
        // mainPipeline.init();

        int signal = mainPipeline.getSignal();


        while (!opModeIsActive()) {
            signal = mainPipeline.getSignal();
            telemetry.addData("signal", signal);
            telemetry.update();

        }
        waitForStart();
        signal = mainPipeline.getSignal();

        //robot.gyroStrafeEncoder(0.6, 180, 5);
//        signal = 1;
        //if cone is in the middle


        if (signal == 2) {
            //setup
            //Setup
            robot.spike_set_place(2, true);
        } else if (signal == 3) {
            robot.spike_set_place(3, true);
        } else if (signal == 1) {
            robot.spike_set_place(1, true);


        }


    }
}

