package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

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
public class PracticeAuto extends LinearOpMode {
  //  private DcMotor bob;
    private RobotClass3 robot;
    private BNO055IMU imu;
    private DcMotor motorFL, motorBR, motorBL, motorFR;



    @Override
    public void runOpMode() throws InterruptedException{
        //bob = hardwareMap.dcMotor.get("bubble");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBR = hardwareMap.dcMotor.get("BR");
        motorFL = hardwareMap.dcMotor.get("FL");
        motorBL = hardwareMap.dcMotor.get("BL");

        robot = new RobotClass3( motorFL,  motorFR,  motorBL,  motorBR, imu, this, false);
        robot.setupRobot();

        waitForStart();

        //robot.moveBob("1turn", 0.3);

        // move forward
        robot.gyroStrafeEncoder(0.3,180,24);
        // move left to center of the bridge
        robot.gyroStrafeEncoder(0.4, 90, 100);
        robot.gyroTurn(180, 0.8);
        robot.gyroStrafeEncoder(0.3,90,100);
        robot.gyroTurn(180,0.4);
        //sigma lines



    }


}
