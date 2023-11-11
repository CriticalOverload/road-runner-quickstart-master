package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


@Autonomous
public class OpenCVExample extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override

    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam1.setPipeline(new examplePipeline());
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);//Check our camera dimensions
            }

            public void onError(int errorCode) {


            }
        });
    }

    @Override
    public void loop() {
    }
    class examplePipeline extends OpenCvPipeline{
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat middleCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        double middleavgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);//Change this to the color you're detecting

        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);//Or this?
            telemetry.addLine( "pipeline running");
            Rect leftRect = new Rect(1, 1, 219, 359);
            Rect middleRect = new Rect(220, 1, 199, 359);
            Rect rightRect = new Rect( 420, 1, 219, 359); //Splits manually

            input.copyTo(outPut); //Making the border things
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, middleRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            middleCrop = YCbCr.submat(middleRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(middleCrop, middleCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar middleavg = Core.mean(middleCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            middleavgfin = middleavg.val[0];
            rightavgfin = rightavg.val[0];

            // if (leftavgfin > rightavgfin){
            //     telemetry.addLine("Left Greater: " + leftavgfin);
            //     telemetry.addLine("Right: " + rightavgfin);

            // }
            // else{
            //     telemetry.addLine("Left: " + leftavgfin);
            //     telemetry.addLine("Right Greater: " + rightavgfin);
            // }

            if (rightavgfin > leftavgfin){
                telemetry.addLine("Left");
                telemetry.addLine("mid: "+ middleavgfin);
                telemetry.addLine("right:"+ rightavgfin);
                telemetry.addLine("left: "+ leftavgfin);
           } else if ((middleavgfin > rightavgfin) && (middleavgfin > leftavgfin)) {
                telemetry.addLine("Right");
                telemetry.addLine("mid: "+ middleavgfin);
                telemetry.addLine("right:"+ rightavgfin);
                telemetry.addLine("left: "+ leftavgfin);
           } else{
               telemetry.addLine("Middle");
               telemetry.addLine("mid: "+ middleavgfin);
               telemetry.addLine("right:"+ rightavgfin);
               telemetry.addLine("left: "+ leftavgfin);


           }
            return(outPut);


        }
    }

}