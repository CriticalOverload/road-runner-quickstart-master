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
import org.firstinspires.ftc.teamcode.FinalOpenCV2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="BlueLeft")
public class BlueLeft extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR;
    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;


    FinalOpenCV2 mainPipeline;


    OpenCvWebcam webcam1 = null;


    @Override
    public void runOpMode() throws InterruptedException{
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
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
        FinalOpenCV2 mainPipeline = new FinalOpenCV2();
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

        //if cone is in the middle
        if (signal == 2){
            robot.gyroStrafeEncoder(0.6,90,12);
//            robot.gyroTurn(90,0.6);
//            robot.gyroStrafeEncoder(0.6,97,24);

        }
        //if cone is in the left
        else if (signal == 1) {
            robot.gyroStrafeEncoder(0.6, 180, 5);
//            robot.gyroTurn(90, 0.6);
//            robot.gyroStrafeEncoder(0.6, 0, 19);
//            robot.gyroStrafeEncoder(0.6, 90,24);

        }

        //if cone is in the right
        else if (signal  == 3){
            robot.gyroStrafeEncoder(0.7, 0, 5);
//            robot.gyroTurn(-70, 0.7);
//            robot.gyroStrafeEncoder(0.5, 180, 19);
//            robot.gyroTurn(160, 0.7);
//            //robot.gyroStrafeEncoder(0.5, 90 ,6);
//            robot.gyroStrafeEncoder(0.7, 90, 24);
        }



    }
}
