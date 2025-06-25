package org.firstinspires.ftc.teamcode.Opmodes.Teleops.Autonomous;

import android.service.controls.actions.CommandAction;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

@Autonomous(name = "Clip", group = "Autonomous")
public class Clip1 extends LinearOpMode {

    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); // these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.addData("PInpoint Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        long pickTime = 200;
        long scoreTime = 300;

        Pose2d initialPose = new Pose2d(3, -61, Math.toRadians(90));
        Pose2d secondPose = new Pose2d(37, -25, Math.toRadians(90));
        Pose2d pickupPose = new Pose2d(36, -50, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);



        TrajectoryActionBuilder pickup = drive.actionBuilder(pickupPose)
                .splineTo(new Vector2d(36,-50), Math.toRadians(90));

        TrajectoryActionBuilder initScore = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(3, -30), Math.toRadians(90));

        TrajectoryActionBuilder samplePush = drive.actionBuilder(secondPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(37, -25, Math.toRadians(90)), Math.toRadians(90))


                // push sample 1
                .splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -25, Math.toRadians(90)), Math.toRadians(90))

                //push sample 2
                .splineToLinearHeading(new Pose2d(51, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(56, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, -25, Math.toRadians(90)), Math.toRadians(90))

                //push sample 3
                .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(62, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(62, -49, Math.toRadians(90)), Math.toRadians(270));

        waitForStart();

        if (isStopRequested())
            return;


        Actions.runBlocking(
                new SequentialAction(
                        initScore.build())
        );


        sleep(50);

        Actions.runBlocking(
                new SequentialAction(
                        samplePush.build())
        );

        sleep(50);

        Actions.runBlocking(
                new ParallelAction(
                        pickup.build()

                )
        );

    }
}
