//package org.firstinspires.ftc.teamcode;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
////import static org.firstinspires.ftc.teamcode.OpenCVExample.signal;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import org.openftc.easyopencv.OpenCvWebcam;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//
//@Autonomous
//public class OpenCVExample extends OpMode {
//
//    public static int signal;
//    private DcMotor motorFL, motorBR, motorBL, motorFR;
//    private BNO055IMU imu;
//
//    private RobotClass3 robot;// Adjust motor names based on your robot configuration
//
//    OpenCvWebcam webcam1 = null;
//    //public int signal;
//    @Override
//
//    public void init() {
//
////        motorFL = hardwareMap.dcMotor.get("FL");
////        motorFR = hardwareMap.dcMotor.get("FR");
////        motorBL = hardwareMap.dcMotor.get("BL");
////        motorBR = hardwareMap.dcMotor.get("BR");
////        imu = hardwareMap.get(BNO055IMU.class, "imu");
////
////
////        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, imu, this, false);
////        robot.setupRobot();
//
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        webcam1.setPipeline(new examplePipeline());
//        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            public void onOpened() {
//                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);//Check our camera dimensions
//            }
//
//            public void onError(int errorCode) {
//
//
//            }
//        });
//    }
//
//    @Override
//    public void loop() {
//
//        if (signal == 2){
//            telemetry.addLine("SIGNAL 2");
//
////            robot.gyroStrafeEncoder(0.6,90,24);
////            robot.gyroTurn(90,0.6);
////            robot.gyroStrafeEncoder(0.6,97,24);
//
//        }
//        else if (signal == 1) {
//            telemetry.addLine("SIGNAL 1");
//
////            robot.gyroStrafeEncoder(0.6, 90, 5);
////            robot.gyroTurn(90, 0.6);
////            robot.gyroStrafeEncoder(0.6, 0, 19);
////            robot.gyroStrafeEncoder(0.6, 90,24);
//
//        }
//        else if (signal  == 3){
//            telemetry.addLine("SIGNAL 3");
////            robot.gyroStrafeEncoder(0.7, 90, 5);
////            robot.gyroTurn(-70, 0.7);
////            robot.gyroStrafeEncoder(0.5, 180, 19);
////            robot.gyroTurn(160, 0.7);
////            //robot.gyroStrafeEncoder(0.5, 90 ,6);
////            robot.gyroStrafeEncoder(0.7, 90, 24);
//        }
//
//        // Update telemetry if needed
//        //telemetry.addData("Signal", signal);
//        telemetry.update();
//    }
//
//    public static int getSignal() {
//        return signal;
//    }
//}
//
//
//
//    class examplePipeline extends OpenCvPipeline {
//        Mat YCbCr = new Mat();
//        Mat leftCrop;
//        Mat middleCrop;
//        Mat rightCrop;
//        static double leftavgfin;
//        static double rightavgfin;
//        static double middleavgfin;
//        int signal2;
//        Mat outPut = new Mat();
//        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);//Change this to the color you're detecting
//
//        public Mat processFrame(Mat input) {
//            signal2 = mainPipeline.getSignal();
//            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//Or this?
//            //telemetry.addLine("pipeline running");
//            Rect leftRect = new Rect(1, 1, 219, 359);
//            Rect middleRect = new Rect(220, 1, 199, 359);
//            Rect rightRect = new Rect(420, 1, 219, 359); //Splits manually
//
//            input.copyTo(outPut); //Making the border things
//            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
//            Imgproc.rectangle(outPut, middleRect, rectColor, 2);
//            Imgproc.rectangle(outPut, rightRect, rectColor, 2);
//
//            leftCrop = YCbCr.submat(leftRect);
//            middleCrop = YCbCr.submat(middleRect);
//            rightCrop = YCbCr.submat(rightRect);
//
//            Core.extractChannel(leftCrop, leftCrop, 2);
//            Core.extractChannel(middleCrop, middleCrop, 2);
//            Core.extractChannel(rightCrop, rightCrop, 2);
//
//            Scalar leftavg = Core.mean(leftCrop);
//            Scalar middleavg = Core.mean(middleCrop);
//            Scalar rightavg = Core.mean(rightCrop);
//
//            leftavgfin = leftavg.val[0];
//            middleavgfin = middleavg.val[0];
//            rightavgfin = rightavg.val[0];
//
//            // if (leftavgfin > rightavgfin){
//            //     telemetry.addLine("Left Greater: " + leftavgfin);
//            //     telemetry.addLine("Right: " + rightavgfin);
//
//            // }
//            // else{
//            //     telemetry.addLine("Left: " + leftavgfin);
//            //     telemetry.addLine("Right Greater: " + rightavgfin);
//            // }
//
//
//
//            if (rightavgfin > leftavgfin) {
//                //telemetry.addLine("this is numero uno");
//                signal2 = 1;
////                telemetry.addLine("mid: " + middleavgfin);
////                telemetry.addLine("right:" + rightavgfin);
////                telemetry.addLine("left: " + leftavgfin);
//
//            } else if ((middleavgfin > rightavgfin) && (middleavgfin > leftavgfin)) {
//
//                signal2 = 3;
//                //telemetry.addLine("this is numero tres");
////                telemetry.addLine("mid: " + middleavgfin);
////                telemetry.addLine("right:" + rightavgfin);
////                telemetry.addLine("left: " + leftavgfin);
//            } else {
//                //telemetry.addLine("this is numero dos");
//                signal2 = 2;
//                //telemetry.addLine("mid: " + middleavgfin);
//                //telemetry.addLine("right:" + rightavgfin);
//                //telemetry.addLine("left: " + leftavgfin);
//
//
//            }
//            return (outPut);
//
//
//        }
//
//
//
//
//
//    }
//
//
//
//
