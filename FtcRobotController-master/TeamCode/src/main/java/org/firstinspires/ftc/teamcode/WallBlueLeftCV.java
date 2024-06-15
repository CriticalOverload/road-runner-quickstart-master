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

//@Autonomous(name="WallBlueLeftCV")
public class WallBlueLeftCV extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR, linearSlide, motorLinearAccuatorJoint;
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
        motorLinearAccuatorJoint = hardwareMap.dcMotor.get("MLAJ");

        //slides = hardwareMap.dcMotor.get("LS");
        // signal = 2;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, linearSlide, motorLinearAccuatorJoint, clawServo, jointServo, imu, this, false);
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
//            //Setup
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
//            robot.gyroStrafeEncoder(0.6, -90, 4);
//            robot.gyroStrafeEncoder(0.6, 0, 5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//
//
//            // end zone
//            robot.gyroStrafeEncoder(0.6, -90, 6);
//            robot.gyroTurn(90, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 27);
//            robot.gyroTurn(-5, 0.4);
//            robot.moveSlides("backboard", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 3);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 180, 28);
            robot.closeClaw();
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);
            robot.gyroStrafeEncoder(0.6, -90, 8);

            //Turn 180
            robot.gyroTurn(180, 0.4);

            //Move towards drop and placement
            robot.gyroStrafeEncoder(0.6, 90, 29);
            robot.gyroStrafeEncoder(0.6, -90, 7);
            robot.gyroStrafeEncoder(0.6, 0, 5);
            robot.openClaw();
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            robot.closeClaw();
            sleep(1000);


            // end zone
            robot.gyroStrafeEncoder(0.6, -90, 6);
            robot.gyroTurn(90, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 27);
            robot.gyroTurn(-5, 0.4);
            robot.moveSlides("backboard", 0.4);
            //robot.gyroStrafeEncoder(0.6, 0, 3);
            jointServo.setPosition(0.5);
            robot.gyroStrafeEncoder(0.3, 90, 8);
            robot.openClaw();
            sleep(1000);
            jointServo.setPosition(0.42);
            robot.gyroStrafeEncoder(0.6, 270, 2);

            jointServo.setPosition(0.23);
            sleep(1000);
            robot.closeClaw();
            robot.moveSlides("final", 0.4);
            robot.gyroStrafeEncoder(0.6, 0, 24);
            robot.gyroStrafeEncoder(0.6 ,90, 4);




            //garage
        }
        else if (signal == 1){
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
//            robot.gyroTurn(90, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 2);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//
//            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 23);
//            robot.gyroStrafeEncoder(0.6, 180, 8);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, .5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 3);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
            robot.closeClaw();
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);

            // 180
            robot.gyroStrafeEncoder(0.6, -90, 8);
            robot.gyroTurn(180, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 29);

            // move towards zone and drop pixel
            robot.gyroStrafeEncoder(0.6, -90, 6);
            robot.gyroTurn(92, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 2);
            robot.openClaw();
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            robot.closeClaw();
            sleep(1000);

            // end zone
            robot.gyroStrafeEncoder(0.6, 90, 23);
            robot.gyroStrafeEncoder(0.6, 180, 13);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.5);
            robot.gyroStrafeEncoder(0.3, 90, 2);
            robot.openClaw();
            sleep(1000);
            jointServo.setPosition(0.42);
            robot.gyroStrafeEncoder(0.6, 270, 3);

            jointServo.setPosition(0.23);
            sleep(1000);
            robot.closeClaw();
            robot.moveSlides("final", 0.4);
            robot.gyroStrafeEncoder(0.6, 0, 30);
            robot.gyroStrafeEncoder(0.6 ,90, 4);

        }

        else if (signal == 3){
            //RUN 2
            robot.closeClaw();
            robot.moveSlides("move", 0.6);
            robot.moveMLAJ("down", 0.4);
            jointServo.setPosition(0.23);
            robot.gyroStrafeEncoder(0.6, -90, 8);

            //Turn 180
            robot.gyroTurn(180, 0.4);

            //Move towards drop and placement
            robot.gyroStrafeEncoder(0.6, 90, 29);
            robot.gyroStrafeEncoder(0.6, -90, 7);
            robot.gyroStrafeEncoder(0.6, 0, 5);
            robot.openClaw();
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            robot.closeClaw();
            sleep(1000);


            // end zone
            robot.gyroStrafeEncoder(0.6, -90, 6);
            robot.gyroTurn(90, 0.4);
            robot.gyroStrafeEncoder(0.6, 90, 27);
            robot.gyroTurn(-5, 0.4);
            robot.moveSlides("backboard", 0.4);
            //robot.gyroStrafeEncoder(0.6, 0, 3);
            jointServo.setPosition(0.5);
            robot.gyroStrafeEncoder(0.3, 90, 8);
            robot.openClaw();
            sleep(1000);
            jointServo.setPosition(0.42);
            robot.gyroStrafeEncoder(0.6, 270, 2);

            jointServo.setPosition(0.23);
            sleep(1000);
            robot.closeClaw();
            robot.moveSlides("final", 0.4);
            robot.gyroStrafeEncoder(0.6, 0, 24);
            robot.gyroStrafeEncoder(0.6 ,90, 4);


//            //setup
//            clawServo.setPosition(0.35);
//            robot.moveSlides("move", 0.6);
//            robot.moveMLAJ("down", 0.4);
//            jointServo.setPosition(0.23);
//            //180 turn
//            robot.gyroStrafeEncoder(0.6, -90, 8);
//            robot.gyroTurn(180, 0.4);
//            //move towards zone and place pixel
//            robot.gyroStrafeEncoder(0.6, 90, 16);
//            robot.gyroTurn(-90, 0.4);
//            robot.gyroStrafeEncoder(0.6, 90, 11);
//            robot.gyroStrafeEncoder(0.6, -90, 2.7);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            robot.moveSlides("zone", 0.3);
//            clawServo.setPosition(0.35);
//            sleep(1000);
//            robot.gyroStrafeEncoder(0.6, -90, 10);
//            robot.gyroTurn(180, 0.4);
//            // end zone
//            robot.gyroStrafeEncoder(0.6, 90, 22);
//            //robot.gyroStrafeEncoder(0.6, 0, 7);
//            robot.moveSlides("backboard", 0.4);
//            jointServo.setPosition(0.5);
//            robot.gyroStrafeEncoder(0.3, 90, 5);
//            clawServo.setPosition(0.7);
//            sleep(1000);
//            jointServo.setPosition(0.42);
//            robot.gyroStrafeEncoder(0.6, 270, 2);
//            jointServo.setPosition(0.23);
//            clawServo.setPosition(0.32);
//            robot.moveSlides("final", 0.4);
//            robot.gyroStrafeEncoder(0.6, 0, 17);
//            //change this
//            robot.gyroStrafeEncoder(0.6, 0, 18);
//            robot.gyroStrafeEncoder(0.6 ,90, 4);


        }



    }
}
