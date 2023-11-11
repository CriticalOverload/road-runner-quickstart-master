package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name = "AutonomousExample")
public class AutonomousExample extends LinearOpMode {
    // Define your hardware components (e.g., motors, servos, etc.) here
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private DcMotor parallelEncoder, perpendicularEncoder;


    // Define your deadwheel encoder objects here
    // Example: private Encoder parallelEncoder;
    // Example: private Encoder perpendicularEncoder;

    // Constants for your robot's physical configuration and odometry
    private double wheelRadius = 2.0; // Radius of the deadwheels in inches
    private double ticksPerRevolution = 1440.0; // Number of encoder ticks per revolution
    private double robotWidth = 10.0; // Width between deadwheels in inches

    // Variables for tracking position and orientation
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotTheta = 0.0;

    // Previous encoder values
    private int prevParallelEncoderValue = 0;
    private int prevPerpendicularEncoderValue = 0;

    @Override
    public void runOpMode() {
        // Initialize your hardware components and encoders here
        motorFrontRight = hardwareMap.get(DcMotor.class, "rightFront");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        motorBackLeft = hardwareMap.get(DcMotor.class, "leftRear");
        motorBackRight = hardwareMap.get(DcMotor.class, "rightRear");
        parallelEncoder = hardwareMap.dcMotor.get("parallelEncoder");
        perpendicularEncoder = hardwareMap.dcMotor.get("perpendicularEncoder");

        // Initialize and configure your deadwheel encoders here
        // Example: parallelEncoder = hardwareMap.get(Encoder.class, "parallel_encoder");
        // Example: perpendicularEncoder = hardwareMap.get(Encoder.class, "perpendicular_encoder");

        // Configure the encoders as needed (e.g., set direction, mode, etc.)

        waitForStart();

        while (opModeIsActive()) {
            // Call the updateOdometry method to update position and orientation
            updateOdometry();

            // Implement your autonomous tasks using the updated position and orientation
            // Example: Drive forward for a certain distance
            // Example: Rotate to a specific angle

            // Use robotX, robotY, and robotTheta for precise navigation

            // Stop the motors when the tasks are complete
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // End the autonomous op mode
            break;
        }
    }

    private void updateOdometry() {
        // Get parallel deadwheel encoder value (ticks or counts)
        int parallelEncoderValue = parallelEncoder.getCurrentPosition();
        // Get perpendicular deadwheel encoder value (ticks or counts)
        int perpendicularEncoderValue = perpendicularEncoder.getCurrentPosition();

        // Calculate changes in encoder values
        int parallelEncoderDelta = parallelEncoderValue - prevParallelEncoderValue;
        int perpendicularEncoderDelta = perpendicularEncoderValue - prevPerpendicularEncoderValue;

        // Update previous encoder values
        prevParallelEncoderValue = parallelEncoderValue;
        prevPerpendicularEncoderValue = perpendicularEncoderValue;

        // Calculate the distance traveled by each deadwheel
        double parallelDistance = (parallelEncoderDelta / ticksPerRevolution) * (2 * Math.PI * wheelRadius);
        double perpendicularDistance = (perpendicularEncoderDelta / ticksPerRevolution) * (2 * Math.PI * wheelRadius);

        // Calculate the change in robot position (x, y) and orientation (theta)
        double deltaParallelDistance = parallelDistance;
        double deltaPerpendicularDistance = perpendicularDistance;
        double deltaTheta = (deltaParallelDistance - deltaPerpendicularDistance) / robotWidth; // robotWidth is the distance between the deadwheels

        // Update the robot's position and orientation
        robotTheta += deltaTheta;
        double deltaX = deltaParallelDistance * Math.cos(robotTheta);
        double deltaY = deltaParallelDistance * Math.sin(robotTheta);
        robotX += deltaX;
        robotY += deltaY;
    }
}
