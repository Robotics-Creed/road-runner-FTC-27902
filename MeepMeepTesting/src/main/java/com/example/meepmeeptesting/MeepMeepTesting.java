package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.785)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(90)))
                        .forward(28)
                        .waitSeconds(2)
                        .splineToConstantHeading(new Vector2d(0,-34), Math.toRadians(0))
                        .strafeRight(27)
                        .splineToConstantHeading(new Vector2d(42,-12), Math.toRadians(0))
                        .back(45)
                        //block 2
                        .forward(45)
                        .splineToConstantHeading(new Vector2d(52,-13), Math.toRadians(0))
                        .back(45)
                        //block 3
                        .forward(45)
                        .splineToConstantHeading(new Vector2d(62,-13), Math.toRadians(0))
                        .back(45)
                        .forward(5)
                        //grab block 2
                        .splineTo(new Vector2d(34,-50), Math.toRadians(270))
                        .waitSeconds(1)
                        //score block 2
                        .splineTo(new Vector2d(0,-32), Math.toRadians(90))
                        .waitSeconds(1)
                        //grab block 3
                        .back(2)
                        .splineTo(new Vector2d(34,-50), Math.toRadians(90))
                        //score block 3

                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}