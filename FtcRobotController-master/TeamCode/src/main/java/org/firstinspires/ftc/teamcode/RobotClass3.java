package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class RobotClass3 {


    private DcMotor motorFL, motorBR, motorBL, motorFR, motorLS;//our motor
    private DcMotor[] motors;//beware.... uhh
    //private DcMotor viperslide;
    private Servo claw;

    //deadwheel stuff
    private DcMotor leftEncoder, rightEncoder, backEncoder;
    boolean deadWheels = false;

    //sensors
    private DistanceSensor distSensor;
    private TouchSensor stopButton;

    //imu stuff
    private BNO055IMU imu;
    private Orientation angles, lastAngles, startAngles;
    private double globalAngle;

    //Declare an opmode and a telemetry object
    private LinearOpMode opMode;
    private Telemetry telemetry;//if we're using driver station

    //for ftcdashboard
//    private TelemetryPacket packet;
//    private FtcDashboard dash;
    private boolean yesDash;

    //for PID
    private double lastError=0;
    private double ckp, cki, ckd;//there is a c in front to remind me to CHANGE!!! TODO!!!!!!!!!!!
    private double integral=0;

    private double robotIntegral=0;
    private double robotLastError=0;

    //drive distance calculation
    private final double DRIVE_WHEEL_CIRCUMFERENCE = Math.PI * 3.77953;
    //
    // private final double MOTOR_RPM = 435;
    // private final double MOTOR_SPR = IDK WHAT THIS IS 60/MOTOR_RPM;//CHANGE
    // private final double SECONDS_PER_CM = IDK WHAT THIS IS EITEHR MOTOR_SPR/DRIVE_WHEEL_CIRCUMFERENCE;
    private final double TICKS_PER_REV = (1+(double)46/17) * (1+(double)46/17) * 28;
    private final double TICKS_PER_IN = TICKS_PER_REV / DRIVE_WHEEL_CIRCUMFERENCE;

    // private final double ENCODER_WHEEL_CIRCUMFERENCE = Math.Pi * diameter!!!!!!!!;
    // private final double ENCODER_TICKS_PER_REV = INSERTNUMBERHERE;
    private final double ENCODER_TICKS_PER_IN = 307.699557;


    /**
     * Base only Constructor
     * @param motorFL front left motor
     * @param motorFR front right motor
     * @param motorBL back left motor
     * @param motorBR back right motor
     * @param imu imu
     * @param opMode From the opMode we get telemetry
     * @param yesDash if we're using the dashboard
     * */
    public RobotClass3(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, BNO055IMU imu, LinearOpMode opMode, boolean yesDash){
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        this.imu = imu;
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        motors = new DcMotor[]{this.motorFL, this.motorBR, this.motorBL, this.motorFR};
        this.yesDash = yesDash;
    }
    public void setupRobot() throws InterruptedException{
        reverseMotors();
        for(DcMotor m : motors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //viperslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setIMUParameters();
        resetEncoders();
        //resetSlides();
        resetAngle();

        if(yesDash)
            telemetry.addLine("abdkd");
            //setupDashboard();

        while (!imu.isGyroCalibrated()) {
            if(yesDash){
                telemetry.addData("IMU", "calibrating...");
                telemetry.update();
                sleep(50);
            }
            else {
                telemetry.addData("IMU", "calibrating...");
                telemetry.update();
                sleep(50);
            }
        }

        telemetry.addData("IMU", "ready");
        telemetry.update();
    }
    private BNO055IMU.Parameters getIMUParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        parameters.mode                = BNO055IMU.SensorMode.IMU;

        return parameters;
    }
//    public void setupDashboard(){
//        packet = new TelemetryPacket();
//        dash = FtcDashboard.getInstance();
//        packet.put("setup","done");
//        dash.sendTelemetryPacket(packet);
//    }
    public void completeStop(){
        for(DcMotor m : motors) {
            m.setPower(0);
        }
    }
    private void setIMUParameters(){
        BNO055IMU.Parameters parameters = getIMUParameters();
        imu.initialize(parameters);
    }
    public void reverseMotors() throws InterruptedException{
        //reverse needed motors here!!
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.REVERSE);
    }
    public void resetEncoders(){
        for(DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(deadWheels){
            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        lastError = 0;
    }
    public void resetAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        startAngles = lastAngles;
        globalAngle = 0;
    }

    public void turn(double power){
        motorFL.setPower(-power);
        motorBL.setPower(-power);
        motorFR.setPower(power);
        motorBR.setPower(power);
    }
    private void composeAngleTelemetry(){
        telemetry.addData("Start Angle", startAngles.firstAngle);
        telemetry.addData("Current Angle", angles.firstAngle);
        telemetry.addData("Global Angle", globalAngle);
    }
    public void moveSlides(char level, double power){
        double circumference = 4.409;//circumference of pulley for hub
//        double groundRN = ; // rotations needed to reach point from 0
//        double smallRN =;
        double medRN =26/circumference;
        double highEP =37/circumference;// (inches from ground / circumference)* ticks

        //setup this with pid stuff
        int target;
        switch(level){//todo!!!!!!!!!!!!!!!!!!!!!
            default:
            case 'u':
                target = -8927;
                break;
            case 'd':
                target = 0;
                break;
        }
        if(motorLS.getCurrentPosition() < target) {
            motorLS.setPower(power);
        }
        else{
            motorLS.setPower(-power);
        }
        motorLS.setTargetPosition(target);
        while((!stopButton.isPressed())&&(Math.abs(motorLS.getCurrentPosition() - target) > 5 && opMode.opModeIsActive()));
        motorLS.setPower(0);

    }
    public void gyroTurn(int degrees, double power) throws InterruptedException{
        //restart angle tracking
        resetAngle();
        if(degrees == 0)
            return;
        if(degrees < 0){
            turn(power*-1);
        }
        else{
            turn(power);
        }

        //Rotate until current angle is equal to the target angle
        //getAngle()-degrees
        /*while(opMode.opModeIsActive() && Math.abs(getAngle()-degrees) > 10){
            composeAngleTelemetry();
            telemetry.addData("Target angle", degrees);
            telemetry.update();
        }*///doesn't work the way we want it to... may edit later but not urgent todo
        if (degrees < 0){
            while (opMode.opModeIsActive() && getAngle() > degrees+10){
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }else{
            while (opMode.opModeIsActive() && getAngle() < degrees-10) {
                composeAngleTelemetry();
                //display the target angle
                telemetry.addData("Target angle", degrees);
                telemetry.update();
            }
        }

        completeStop();
        //Wait .5 seconds to ensure robot is stopped before continuing
        sleep(250);
        resetAngle();
    }

    public void correctedTankStrafe(double leftPower, double rightPower, double correction){
        motorFL.setPower(leftPower - correction);
        motorFR.setPower(rightPower + correction);
        motorBL.setPower(rightPower - correction);
        motorBR.setPower(leftPower + correction);
    }
    public double getAngle(){
        //Get a new angle measurement
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Get the difference between current angle measurement and last angle measurement
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        //Process the angle to keep it within (-180,180)
        //(Once angle passes +180, it will rollback to -179, and vice versa)
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the change in angle since last measurement (deltaAngle)
        //to the change in angle since last reset (globalAngle)
        globalAngle += deltaAngle;
        //Set last angle measurement to current angle measurement
        lastAngles = angles;

        return globalAngle;
    }
    public double getCorrection(){
        //Get the current angle of the robot
        double angle = getAngle();

        //Use the angle to calculate the correction
        if (angle == 0){
            //If angle = 0, robot is moving straight; no correction needed
            return 0;
        }else{
            //If angle != 0, robot is not moving straight
            //Correction is negative angle (to move the robot in the opposite direction)
            //multiplied by gain; the gain is the sensitivity to angle
            //We have determined that .1 is a good gain; higher gains result in overcorrection
            //Lower gains are ineffective
            return -angle*0.02;//gain;<<<< make var?? todo do we even need gain?
        }
    }
    public void gyroStrafeEncoder(double power, double angle, double in) throws InterruptedException{
        double ticks = in * TICKS_PER_IN;

        //convert direction (degrees) into radians
        double newDirection = angle * Math.PI/180 - Math.PI/4;
        //calculate powers needed using direction
        double leftPower = Math.cos(newDirection) * power;
        double rightPower = Math.sin(newDirection) * power;
        telemetry.addData("left: ",leftPower);
        telemetry.addData("right: ",rightPower);
        telemetry.update();
        resetEncoders();
        resetAngle();
//        setNewGain(0.02);

        while(Math.abs(motorBR.getCurrentPosition()) < ticks && opMode.opModeIsActive()){
            double correction = getCorrection();
            composeAngleTelemetry();
            telemetry.addData("correction",correction);
            telemetry.update();
            correctedTankStrafe(leftPower, rightPower, correction);
        }
        completeStop();
        sleep(250);
        resetAngle();
        resetEncoders();
    }


}
