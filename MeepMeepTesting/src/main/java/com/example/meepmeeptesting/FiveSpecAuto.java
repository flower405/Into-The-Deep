package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FiveSpecAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15.5, -64, Math.toRadians(90)))
                .strafeToConstantHeading(new Vector2d(0, -34))
                .splineToSplineHeading(new Pose2d(10, -45, Math.toRadians(80)), Math.toRadians(0)) // sample 1 in
                .splineToLinearHeading(new Pose2d(38, -39, Math.toRadians(45)), Math.toRadians(45)) // sample 1 in
//                .strafeToLinearHeading(new Vector2d(39, -47), Math.toRadians(315)) // sample 1 out
//                .strafeToLinearHeading(new Vector2d(43, -37), Math.toRadians(45)) // sample 2 in
//                .strafeToLinearHeading(new Vector2d(43, -48), Math.toRadians(315)) // sample 2 out
//                .strafeToLinearHeading(new Vector2d(55.5, -37.5), Math.toRadians(45)) // sample 3 in
//                .strafeToLinearHeading(new Vector2d(46, -47), Math.toRadians(305)) // sample 3 out
//                .strafeToLinearHeading(new Vector2d(37, -47), Math.toRadians(90)) // specimen 1 position
//                .strafeToConstantHeading(new Vector2d(37, -59))
//                .strafeToLinearHeading(new Vector2d(-4, -30), Math.toRadians(90)) // specimen 1
//                .splineToConstantHeading(new Vector2d(0,-36), Math.toRadians(315)) // specimen 2 pickup
//                .splineToConstantHeading(new Vector2d(37,-58), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(37,-60), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-4, -31), Math.toRadians(90)) // specimen 2
//                .splineToConstantHeading(new Vector2d(0,-36), Math.toRadians(315)) // specimen 3 pickup
//                .splineToConstantHeading(new Vector2d(37,-58), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(37,-60), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(-2, -31), Math.toRadians(90)) // specimen 3
//                .splineToConstantHeading(new Vector2d(0,-36), Math.toRadians(315)) // specimen 4 pickup
//                .splineToConstantHeading(new Vector2d(37,-58), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(37,-60), Math.toRadians(270))
//                .strafeToLinearHeading(new Vector2d(1, -31), Math.toRadians(90)) // specimen 4
                .build());












        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}