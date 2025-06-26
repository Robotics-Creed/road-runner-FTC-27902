package org.firstinspires.ftc.teamcode.Opmodes.Teleops.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.Teleops.StarterBot2025TeleOpJava;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
//import org.firstinspires.ftc.teamcode.subsystems.Claw;

import com.google.gson.annotations.Until;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "SpecimenToObservationZone", group = "Autonomous")
public class CompetitionSpecimenAuto extends LinearOpMode {

    private DcMotor leftFront, leftBack, rightFront, rightBack, arm;
    private Servo claw;

    private static final double CLAW_CLOSED_POSITION = 0.7;
    private static final double CLAW_OPEN_POSITION = 0.55;

    // Encoder constants (you may need to adjust based on your wheel diameter and gear ratio)
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;  // eg: GoBilda 5202 series
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;    // No gear reduction
    static final double     WHEEL_DIAMETER_INCHES   = 3.78;   // GoBilda Mecanum wheels
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    //claw
    public static double ClawServo_CLOSE = 0.5;
    public static final double ClawServo_OPEN = 0.68;
    boolean isClawOpen = false;

    //arm


    private int ARM_CURRENT_POS = 0;
    private static final int ARM_POSITION_INIT = 100;
    private static final int ARM_POSITION_INTAKE = 475;
    private static final int ARM_POSITION_WALL_GRAB = 4300;
    private static final int ARM_POSITION_WALL_UNHOOK = 4100;
    private static final int ARM_POSITION_HOVER_HIGH = 1550;
    private static final int ARM_POSITION_CLIP_HIGH = 1950;

    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    private int CurrentArmPos = arm.getCurrentPosition();


    @Override
    public void runOpMode() throws InterruptedException {
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        //claw
        claw = hardwareMap.get(Servo.class, "claw");

        //arm
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set directions
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        // Stop and reset encoders
        resetDriveEncoders();

        // Set to RUN_USING_ENCODER
        setRunUsingEncoders();

        // Wait for game to start
        telemetry.addLine("Ready to run auto");
        telemetry.update();

        waitForStart();


        if (opModeIsActive()) {

            // Close claw to grab specimen
            claw.setPosition(CLAW_CLOSED_POSITION);
            sleep(100); // wait for claw to grab

            while (CurrentArmPos != ARM_POSITION_HOVER_HIGH) {
                if (CurrentArmPos < ARM_POSITION_HOVER_HIGH) {
                    arm.setPower(0.8);
                }
                else
                    arm.setPower(0);
            }

            // Drive forward to observation zone (adjust inches as needed)
            encoderDrive(1, 24, 24, 3.0);  // Move forward 24 inches

            sleep(500);

            // Arm moves up and clips
            while (CurrentArmPos != ARM_POSITION_CLIP_HIGH) {
                if (CurrentArmPos < ARM_POSITION_CLIP_HIGH) {
                    arm.setPower(0.8);
                }
                else
                    arm.setPower(0);
            }

            sleep(100);

            // Open claw to drop specimen
            claw.setPosition(CLAW_OPEN_POSITION);

            sleep(200);

            // Optionally back up a little
            encoderDrive(0.4, -6, -6, 0.5);

            encoderStrafeRight(0.8, 35,0.5);

            encoderDrive(0.7,12,12,0.5);

            encoderStrafeRight(0.8, 12,0.5);

            encoderDrive(0.7,-45,-45,0.5);
            encoderStrafeRight(0.8, 12,0.5);

            encoderDrive(0.7,-45,-45,0.5);


        }
    }

    private void resetDriveEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderStrafeRight(double speed, double inches, double timeoutS) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        int leftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
        int rightFrontTarget = rightFront.getCurrentPosition() - moveCounts;
        int leftBackTarget = leftBack.getCurrentPosition() - moveCounts;
        int rightBackTarget = rightBack.getCurrentPosition() + moveCounts;

        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetRuntime();
        while (opModeIsActive() &&
                (getRuntime() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Running to %7d :%7d", leftFrontTarget, rightFrontTarget);
            telemetry.update();
        }

        stopDriveMotors();
        setRunUsingEncoders();
    }
    private void encoderStrafeLeft(double speed, double inches, double timeoutS) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        int LeftFrontTarget = leftFront.getCurrentPosition() - moveCounts;
        int RightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        int LeftBackTarget = leftBack.getCurrentPosition() + moveCounts;
        int RightBackTarget = rightBack.getCurrentPosition() - moveCounts;

        leftFront.setTargetPosition(LeftFrontTarget);
        rightFront.setTargetPosition(RightFrontTarget);
        leftBack.setTargetPosition(LeftBackTarget);
        rightBack.setTargetPosition(RightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetRuntime();
        while (opModeIsActive() &&
                (getRuntime() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Running to %7d :%7d", LeftFrontTarget, RightFrontTarget);
            telemetry.update();
        }

        stopDriveMotors();
        setRunUsingEncoders();
    }
    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        int newLeftBackTarget = leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        int newRightBackTarget = rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

        leftFront.setTargetPosition(newLeftFrontTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetRuntime();
        while (opModeIsActive() &&
                (getRuntime() < timeoutS) &&
                (leftFront.isBusy() && rightFront.isBusy() &&
                        leftBack.isBusy() && rightBack.isBusy())) {
            telemetry.addData("Path", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget);
            telemetry.update();
        }

        stopDriveMotors();
        setRunUsingEncoders();
    }

    private void stopDriveMotors() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
