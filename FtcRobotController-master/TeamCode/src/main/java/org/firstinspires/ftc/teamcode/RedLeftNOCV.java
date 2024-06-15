package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RedLeftNOCV extends LinearOpMode {
    private DcMotor motorFL, motorBR, motorBL, motorFR, linearSlide;
    private Servo  jointServo, clawServo ;

    private DcMotor slides;

    private BNO055IMU imu;

    private RobotClass3 robot;
    private int signal = 3;



    @Override
    public void runOpMode() throws InterruptedException {
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");
        clawServo = hardwareMap.servo.get("claw");
        jointServo = hardwareMap.servo.get("JS");
        linearSlide = hardwareMap.dcMotor.get("LS");
        // signal = 2;
        //int signal = 1;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        robot = new RobotClass3(motorFL, motorFR, motorBL, motorBR, linearSlide, imu, this, false);
        robot.setupRobot();


//        while(!opModeIsActive()){
//            telemetry.addLine("signal");
//            telemetry.update();
//        }
//        signal = mainPipeline.getSignal();
//        telemetry.addData("signal",signal);
        waitForStart();
        //if webcam on the back, then start facing the back. orientation of initial robot only matters up till #2
        //1. read signal
        //todo: test and update
        //also roadrunner...
        if (signal == 2){
            //setup
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.18);

            //180 turn
            robot.gyroStrafeEncoder(0.6, -93, 8);
            robot.gyroTurn(180, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 22);
            // each zone
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            // end zone
            robot.gyroStrafeEncoder(0.6, -90, 7.5);
            robot.gyroTurn(-89, 0.6);
            robot.gyroStrafeEncoder(0.6, 89, 79);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 3);
            clawServo.setPosition(0.7);
            sleep(1000);
            jointServo.setPosition(0.3);
            robot.gyroStrafeEncoder(0.6, 0, 1.5);
        }
        else if (signal == 3){
            //setup
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.18);
            // 180
            robot.gyroStrafeEncoder(0.6, -93, 8);
            robot.gyroTurn(180, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 15.5);
            // up to zone
            robot.gyroTurn(-90, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 7);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            // end zone
            robot.gyroStrafeEncoder(0.6, 90, 71);
            robot.gyroStrafeEncoder(0.6, 0, 7);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 4);
            clawServo.setPosition(0.7);
            sleep(1000);
            jointServo.setPosition(0.3);
            robot.gyroStrafeEncoder(0.6, 270, 2);
        }

        else if (signal == 1){
            //setup
            clawServo.setPosition(0.35);
            robot.moveSlides("move", 0.6);
            jointServo.setPosition(0.12);
            // 180
            robot.gyroStrafeEncoder(0.6, -90, 8);
            robot.gyroTurn(180, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 14.5);
            // up to zone
            robot.gyroTurn(78, 0.6);
            robot.gyroStrafeEncoder(0.6, 90, 1.5);
            clawServo.setPosition(0.7);
            sleep(1000);
            robot.moveSlides("zone", 0.3);
            clawServo.setPosition(0.35);
            sleep(1000);
            robot.gyroStrafeEncoder(0.6, -90, 3);


            // end zone
            robot.gyroStrafeEncoder(0.6, -90, 79);
            robot.gyroTurn(180, 0.6);
            robot.gyroStrafeEncoder(0.6, 180, 3.5);
            robot.moveSlides("backboard", 0.4);
            jointServo.setPosition(0.4);
            robot.gyroStrafeEncoder(0.3, 90, 2);
            clawServo.setPosition(0.7);
            sleep(1000);
            jointServo.setPosition(0.3);
            robot.gyroStrafeEncoder(0.6, 270, 2);

        }
        }









}

