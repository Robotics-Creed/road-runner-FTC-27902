package org.firstinspires.ftc.teamcode.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Odometry.gobilda.GoBildaPinpointDriver;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="StarterBot_V1_Java", group="Linear Opmode")
public class StarterBot2025TeleOpJava extends LinearOpMode {

    // Declare OpMode members.
    GoBildaPinpointDriver odo;

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront= null;
    private DcMotor rightBack = null;
    private DcMotor arm = null;
    private DcMotor wrist = null;
    private Servo claw = null;
    private CRServo intake = null;


    // Arm and Wrist target positions for each state
    private static final int ARM_POSITION_INIT = 100;
    private static final int ARM_POSITION_INTAKE = 360;
    private static final int ARM_POSITION_WALL_GRAB = 4450;
    private static final int ARM_POSITION_WALL_UNHOOK = 1200;
    private static final int ARM_POSITION_HOVER_HIGH = 2000;
    private static final int ARM_POSITION_CLIP_HIGH = 1900;
    private static final int ARM_POSITION_LOW_BASKET = 1900;

    
    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 250;
    private static final int WRIST_POSITION_SPEC = 0;


    
    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = true;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;
    
    //target position
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        //Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        waitForStart();
        resetRuntime();

        odo.update();

        if (gamepad1.right_stick_button) {
            odo.recalibrateIMU(); // recalibrates the IMU without resetting position
        }

        Pose2D pos = odo.getPosition();
        String data =
                String.format(
                        Locale.US,
                        "{X: %.3f, Y: %.3f, H: %.3f}",
                        pos.getX(DistanceUnit.MM),
                        pos.getY(DistanceUnit.MM),
                        pos.getHeading(AngleUnit.DEGREES));


        // Initialize the hardware variables.
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Stop and reset encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        

        //Set zero power behavior
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        opModeInInit();
        claw.setPosition(CLAW_CLOSED_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // State machine logic
            switch (currentState) {
                case INIT:
                    targetArm = ARM_POSITION_INIT;
                    targetWrist = WRIST_POSITION_INIT;
                    telemetry.addData("State", "INIT");
                    break;
                case INTAKE:
                    targetArm = ARM_POSITION_INTAKE;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "INTAKE");
                    break;

                case WALL_GRAB:
                    targetArm = ARM_POSITION_WALL_GRAB;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_GRAB");
                    break;

                case WALL_UNHOOK:
                    targetArm = ARM_POSITION_WALL_UNHOOK;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_UNHOOK");
                    break;

                case HOVER_HIGH:
                    targetArm = ARM_POSITION_HOVER_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "HOVER_HIGH");
                    break;
                    
                case CLIP_HIGH:
                    targetArm = ARM_POSITION_CLIP_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "CLIP_HIGH");
                    break;
                case LOW_BASKET:
                    targetArm = ARM_POSITION_LOW_BASKET;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "LOW_BASKET");
                    break;
                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }
            
            
            

            // Handle state transitions based on gamepad input
            if (gamepad1.a) {
                currentState = RobotState.INTAKE;
            } else if (gamepad1.b && !lastGrab) {
                if(currentState == RobotState.WALL_GRAB){
                    currentState = RobotState.WALL_UNHOOK;
                }else{
                    currentState = RobotState.WALL_GRAB;
                }
            } else if (gamepad1.y && !lastHook) {
                if(currentState == RobotState.HOVER_HIGH){
                    currentState = RobotState.CLIP_HIGH;
                }else{
                    currentState = RobotState.HOVER_HIGH;
                }
            } else if (gamepad1.x) { 
                currentState = RobotState.LOW_BASKET;           
            } else if (gamepad1.left_bumper) {
                currentState = RobotState.INIT;
            } else if (gamepad1.dpad_up){ //manual control
                currentState = RobotState.MANUAL;
                targetArm += 10;
            } else if (gamepad1.dpad_down){
                currentState = RobotState.MANUAL;
                targetArm -= 10;
            } else if (gamepad1.dpad_left){
                currentState = RobotState.MANUAL;
                targetWrist += 1;
            } else if (gamepad1.dpad_right){
                currentState = RobotState.MANUAL;
                targetWrist -= 1;
            }
            
            lastGrab = gamepad1.b;
            lastHook = gamepad1.y;

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            // Control intake servo with triggerss
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger>0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }
            
            //DRIVE
            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral  =  gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);



            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setTargetPosition(targetWrist);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setPower(1);

            // Send telemetry data to the driver station
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Wrist Power", wrist.getPower());
            telemetry.addData("Position", data);


            telemetry.update();
        }

    }
}
