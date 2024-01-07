//package org.firstinspires.ftc.teamcode;
////import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
////import static org.firstinspires.ftc.teamcode.OpenCVExample.signal;
//
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
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
//public class FinalOpenCV2 extends OpenCvPipeline{
//
//    //send to auto
//    private int signal;
//
//    //webcam
//    OpenCvWebcam webcam1 = null;
//
//    //stuff for cv
//    Mat YCbCr = new Mat();
//    Mat leftCrop;
//    Mat middleCrop;
//    Mat rightCrop;
//    double leftavgfin;
//    double rightavgfin;
//    double middleavgfin;
//    Mat outPut = new Mat();
//    Scalar rectColor = new Scalar(142.0, 16.0, 1.0);//Change this to the color you're detecting
//
//    //send signal to ato
//    public int getSignal(){
//        return signal;
//    }
//
//    //all the cv magic
//    public Mat processFrame(Mat input) {
//        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
////        telemetry.addLine("test");
////        telemetry.addLine("pipeline running");
//        Rect leftRect = new Rect(1, 1, 219, 359);
//        Rect middleRect = new Rect(220, 1, 199, 359);
//        Rect rightRect = new Rect(420, 1, 219, 359); //Splits manually
//
//        input.copyTo(outPut); //Making the border things
//        Imgproc.rectangle(outPut, leftRect, rectColor, 2);
//        Imgproc.rectangle(outPut, middleRect, rectColor, 2);
//        Imgproc.rectangle(outPut, rightRect, rectColor, 2);
//
//        leftCrop = YCbCr.submat(leftRect);
//        middleCrop = YCbCr.submat(middleRect);
//        rightCrop = YCbCr.submat(rightRect);
//
//        Core.extractChannel(leftCrop, leftCrop, 2);
//        Core.extractChannel(middleCrop, middleCrop, 2);
//        Core.extractChannel(rightCrop, rightCrop, 2);
//
//        Scalar leftavg = Core.mean(leftCrop);
//        Scalar middleavg = Core.mean(middleCrop);
//        Scalar rightavg = Core.mean(rightCrop);
//
//        leftavgfin = leftavg.val[0];
//        middleavgfin = middleavg.val[0];
//        rightavgfin = rightavg.val[0];
//
//
//
//        signal = 100;
//
//
//        if ((rightavgfin > leftavgfin) && (rightavgfin > middleavgfin)) {
//
//            signal = 1;
////            telemetry.addData("this is numero uno", signal  );
////            telemetry.update();
//            //telemetry.addLine("mid: " + middleavgfin);
//            //telemetry.addLine("right:" + rightavgfin);
//            //telemetry.addLine("left: " + leftavgfin);
//
//            //right box
//        } else if ((middleavgfin > rightavgfin) && (middleavgfin > leftavgfin)) {
//            signal = 2;
////            telemetry.addData("this is numero tres", signal);
////            telemetry.update();
//            //telemetry.addLine("mid: " + middleavgfin);
//            //telemetry.addLine("right:" + rightavgfin);
//            //telemetry.addLine("left: " + leftavgfin);
//
//            //middle box
//        } else if ((leftavgfin > rightavgfin) && (leftavgfin > middleavgfin)) {
//            signal = 3;
////            telemetry.addLine("signal");
////            telemetry.addData("this is numero dos", signal);
////            telemetry.update();
////              telemetry.addLine("mid: " + middleavgfin);
////              telemetry.addLine("right:" + rightavgfin);
////              telemetry.addLine("left: " + leftavgfin);
//
//
//        }
//        else {
//            signal = 4;
//        }
//        return (outPut);
//
//
//    }
//
//}