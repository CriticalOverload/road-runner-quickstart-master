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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="BlueLeft")
public class SlideAUTO extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;
    private DcMotor motorLS;
    private Servo clawServo, jointServo;

    private BNO055IMU imu;

    private RobotClass3 robot;

    private int signal = 3;

    @Override
    public void runOpMode() throws InterruptedException{
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        clawServo = hardwareMap.servo.get("claw");
        jointServo = hardwareMap.servo.get("JS");
        motorLS = hardwareMap.dcMotor.get("LS");
        //motorLinearAccuator = hardwareMap.dcMotor.get("MLA");
        //motorLinearAccuatorJoint = hardwareMap.dcMotor.get("MLAJ");
        


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, imu, this, false);
        robot.setupRobot();

        while(!opModeIsActive()){
            telemetry.addLine("signal");
            telemetry.update();
        }
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        //todo: test and update
        //also roadrunner...

        robot.moveSlides('u', 0.6);
        jointServo.setPosition(0.5);
        clawServo.setPosition(0.3);
        robot.moveSlides('d', 0.5);




    }
}
