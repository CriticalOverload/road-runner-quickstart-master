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

//@Autonomous(name="AltRedLeftCV")
public class AltRedLeftCV extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR, linearSlide, motorLinearAccuatorJoint;
    private Servo jointServo, clawServo ;
    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;


    CVClassRed mainPipeline;


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
        motorLinearAccuatorJoint = hardwareMap.dcMotor.get("MLAJ");

        //slides = hardwareMap.dcMotor.get("LS");
        // signal = 2;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, linearSlide, motorLinearAccuatorJoint, imu, this, false);
        robot.setupRobot();

//        int camViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        cam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camViewID);
//        mainPipeline = new CVClass();//create new pipeline
//
//        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {//on-ing the camera
////
        CVClassRed mainPipeline = new CVClassRed();
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
            //setup
            //Setup
//            clawServo.setPosition(0.35);
//            robot.moveSlides("move", 0.6);
//            robot.moveMLAJ("down", 0.4);
//            jointServo.setPosition(0.23);
//            robot.gyroStrafeEncoder(0.6, -90, 8);
//
//            //Turn 180
//            robot.gyroTurn(180, 0.4);
//
//            //Move towards drop and placement
//            robot.gyroStrafeEncoder(0.6, 90, 29);
//            robot.gyroStrafeEncoder(0.6, -90, 5.5);
//            robot.gyroStrafeEncoder(0.6, 180, 5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//
//
//            // end zone
//            robot.gyroStrafeEncoder(0.6, -90, 6);
//            robot.gyroTurn(-84, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 27);
//            robot.gyroTurn(-5, 0.4);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 4.25);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 25);
            //setup
            //Setup
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);
            robot.gyroStrafeEncoder(0.6, -90, 8);

            //Turn 180
            robot.gyroTurn(180, 0.4);

            //Move towards drop and placement
            robot.gyroStrafeEncoder(0.6, 90, 29);
            robot.gyroStrafeEncoder(0.6, -90, 7);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);


            // end zone
//            robot.gyroStrafeEncoder(0.6, -90, 6);
//            robot.gyroTurn(-84, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 78);
//            robot.gyroTurn(-5, 0.4);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 8);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 29);
//            robot.gyroStrafeEncoder(0.6 ,90, 4);
        }
        else if (signal == 3){
//            clawServo.setPosition(0.35);
//            robot.moveSlides("move", 0.6);
//            robot.moveMLAJ("down", 0.4);
//            jointServo.setPosition(0.23);
//
//            // 180
//            robot.gyroStrafeEncoder(0.6, -90, 8);
//            robot.gyroTurn(180, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 29);
//
//            // move towards zone and drop pixel
//            robot.gyroStrafeEncoder(0.6, -90, 6);
//            robot.gyroTurn(-90, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 1.5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//
//            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 22);
//            robot.gyroStrafeEncoder(0.6, 0, 16);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 3);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 3);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 18);
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);

            // 180
            robot.gyroStrafeEncoder(0.6, -90, 8);
            robot.gyroTurn(180, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 15);

            // move towards zone and drop pixel
            robot.gyroTurn(-90, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 11);

            // move towards zone and drop pixel
            robot.gyroStrafeEncoder(0.6, -90, 2);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
//
//            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 80);
//            robot.gyroStrafeEncoder(0.6, 0, 16);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 3.5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 3);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 20);
//            robot.gyroStrafeEncoder(0.6 ,90, 4);

        }

        else if (signal == 1){
//            clawServo.setPosition(0.35);
//            robot.moveSlides("move", 0.6);
//            robot.moveMLAJ("down", 0.4);
//            jointServo.setPosition(0.23);
//            //180 turn
//            robot.gyroStrafeEncoder(0.6, -90, 8);
//            robot.gyroTurn(180, 0.4);
//            //move towards zone and place pixel
//            robot.gyroStrafeEncoder(0.6, 90, 17);
//            robot.gyroTurn(90, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 11);
//            robot.gyroStrafeEncoder(0.6, -90, 1);
//            robot.gyroStrafeEncoder(0.6, 0, 2);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//            robot.gyroStrafeEncoder(0.6, -90, 10);
//            robot.gyroTurn(180, 0.4);
//            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 22);
//            robot.gyroStrafeEncoder(0.6, 0, 4);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 4);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 31);
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);
            //180 turn
            robot.gyroStrafeEncoder(0.6, -90, 8);
            robot.gyroTurn(180, 0.4);
            //move towards zone and place pixel
            robot.gyroStrafeEncoder(0.6, 90, 16);
            robot.gyroTurn(90, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 11);
            robot.gyroStrafeEncoder(0.6, -90, 8);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            robot.gyroStrafeEncoder(0.6, -90, 10);
            //robot.gyroTurn(182, 0.4);
            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 80);
//            robot.gyroStrafeEncoder(0.6, 180, 4);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 6);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 30);
//            robot.gyroStrafeEncoder(0.6 ,90, 4);

        }



    }
}
