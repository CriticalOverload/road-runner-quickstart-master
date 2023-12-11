//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
//import org.opencv.core.Mat;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Autonomous(name="BlueRight")
//public class BlueRight extends LinearOpMode {
//    private DcMotor motorFL, motorBR, motorBL, motorFR;
//    private DcMotor slides;
//
//    private BNO055IMU imu;
//
//    private RobotClass3 robot;
//    private OpenCVExample mainPipeline;
//
//    private int signal = 0;
//
//
//
//    @Override
//    public void runOpMode() throws InterruptedException{
//        motorFR = hardwareMap.dcMotor.get("FR");
//        motorBR = hardwareMap.dcMotor.get("BR");
//        motorFL = hardwareMap.dcMotor.get("FL");
//        motorBL = hardwareMap.dcMotor.get("BL");
//        //slides = hardwareMap.dcMotor.get("LS");
//        // signal = 2;
//        //int signal = 1;
//
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, imu, this, false);
//        robot.setupRobot();
//
//
////        while(!opModeIsActive()){
////            telemetry.addLine("signal");
////            telemetry.update();
////        }
////        signal = mainPipeline.getSignal();
////        telemetry.addData("signal",signal);
//        waitForStart();
//        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
//        //1. read signal
//        //todo: test and update
//        //also roadrunner...
//
//        if (signal == 2){
//            robot.gyroStrafeEncoder(0.6,90,24);
//            robot.gyroTurn(80,0.6);
//            robot.gyroStrafeEncoder(0.6,97,72);
//
//        }
//        else if (signal == 1) {
//            robot.gyroStrafeEncoder(0.6, 90, 5);
//            robot.gyroTurn(90, 0.6);
//            robot.gyroStrafeEncoder(0.6, 0, 25);
//            robot.gyroTurn(180, 0.6);
//            robot.gyroStrafeEncoder(0.6, 90, 72);
//
//        }
//        else if (signal  == 3){
//            robot.gyroStrafeEncoder(0.7, 90, 5);
//            robot.gyroTurn(-90, 0.7);
//            robot.gyroStrafeEncoder(0.7, 180, 25);
//            robot.gyroTurn(180, 0.7);
//            //robot.gyroStrafeEncoder(0.5, 90 ,6);
//            robot.gyroStrafeEncoder(0.7, 90, 72);
//        }
//
//
//
//    }
//}
