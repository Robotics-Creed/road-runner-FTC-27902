package org.firstinspires.ftc.teamcode.Opmodes.Teleops.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrive;

@Autonomous(name = "Clip", group = "Autonomous")
public class NewClip1Code extends LinearOpMode {

    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Pinpoint Status", "Initialized");
        telemetry.update();

        Pose2d initialPose = new Pose2d(3, -61, Math.toRadians(90));
        Pose2d secondPose = new Pose2d(37, -25, Math.toRadians(90));
        Pose2d pickupPose = new Pose2d(36, -50, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder pickup = drive.actionBuilder(pickupPose)
                .splineTo(new Vector2d(36, -50), Math.toRadians(90));

        TrajectoryActionBuilder initScore = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(3, -30), Math.toRadians(90));

        TrajectoryActionBuilder samplePush = drive.actionBuilder(secondPose)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(37, -25, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(45, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -25, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(51, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(55, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(56, -49, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, -25, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62, -10, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(62, -25, Math.toRadians(90)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(62, -49, Math.toRadians(90)), Math.toRadians(270));

        waitForStart();

        if (isStopRequested()) return;

        // Run all actions in one sequence
        Actions.runBlocking(
            new SequentialAction(
                initScore.build(),
                samplePush.build(),
                pickup.build()
            )
        );
    }
}
