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
public final class Clip extends LinearOpMode {

    private MecanumDrive drive;
    private GoBildaPinpointDriver odo;

    // Define all poses
    private final Pose2d initialPose = new Pose2d(3, -61, Math.toRadians(90));
    private final Pose2d secondPose = new Pose2d(37, -25, Math.toRadians(90));

    // Define a reusable path
    private SequentialAction samplePath() {
        return new SequentialAction(
            drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(secondPose, Math.toRadians(90))
                .build()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Odometry configuration
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        odo.resetPosAndIMU();

        telemetry.addData("Pinpoint Status", "Initialized");
        telemetry.update();

        // Drive setup with initial pose
        drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if (isStopRequested()) return;

        // Run the planned path
        Actions.runBlocking(samplePath());
    }
}
