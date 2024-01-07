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

@Autonomous(name="BlueRightCV")
public class BlueRightCV extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR, linearSlide;
    private Servo jointServo, clawServo ;
    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;


    CVClassBlue mainPipeline;


    OpenCvWebcam webcam1 = null;


    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        clawServo = hardwareMap.servo.get("claw");
        jointServo = hardwareMap.servo.get("JS");
        linearSlide = hardwareMap.dcMotor.get("LS");
        //slides = hardwareMap.dcMotor.get("LS");
        // signal = 2;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, imu, this, false);
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



        while(!opModeIsActive()){
            signal = mainPipeline.getSignal();
            telemetry.addData("signal", signal);
            telemetry.update();
        }
        waitForStart();
       signal = mainPipeline.getSignal();
       
        //robot.gyroStrafeEncoder(0.6, 180, 5);
//        signal = 1;
        //if cone is in the middle
        if (signal == 2){
            //Setup
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.12);
            robot.gyroStrafeEncoder(0.6, -93, 8);

            //Turn 180
            robot.gyroTurn(185, 0.6);

            //Move towards drop and placement
            robot.gyroStrafeEncoder(0.6, 90, 23);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);


            // end zone
            robot.gyroStrafeEncoder(0.6, -90, 7);
            robot.gyroTurn(94, 0.6);
            robot.gyroStrafeEncoder(0.6, 89, 32);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 1);
            clawServo.setPosition(0.7);
            sleep(1000);
            jointServo.setPosition(0.12);
            robot.gyroStrafeEncoder(0.6, 270, 2);
        }
        else if (signal == 1){
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.12);
            robot.gyroStrafeEncoder(0.6, -93, 8);
            robot.gyroTurn(183, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 17);
            robot.gyroTurn(87, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 7);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            // end zone
            robot.gyroStrafeEncoder(0.6, 90, 71);
            robot.gyroStrafeEncoder(0.6, 180, 3);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 4);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.gyroStrafeEncoder(0.6, 270, 2);
        }

        else if (signal == 3){
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.12);
            robot.gyroStrafeEncoder(0.6, -93, 8);
            robot.gyroTurn(187, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 17);
            robot.gyroTurn(-93, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 2);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            robot.gyroTurn(189, 0.6);
            // end zone
            robot.gyroStrafeEncoder(0.6, 90, 79);
            robot.gyroStrafeEncoder(0.6, 0, 3);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 4);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.gyroStrafeEncoder(0.6, 270, 2);

        }



    }
}
