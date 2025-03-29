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
                .splineToLinearHeading(new Pose2d(37, -38, Math.toRadians(45)), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(39, -44), Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(43, -36), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(43, -44), Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(55.5, -37.5), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(45, -40), Math.toRadians(305))
                 .strafeToLinearHeading(new Vector2d(37, -45), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(37, -58.5))
                .strafeToLinearHeading(new Vector2d(-4, -30), Math.toRadians(90))


               //  .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90)) //
                        .splineToConstantHeading(new Vector2d(0,-36), Math.toRadians(330))
                        .splineToConstantHeading(new Vector2d(37,-55), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(37,-59.5), Math.toRadians(270))



                .strafeToLinearHeading(new Vector2d(-4, -30), Math.toRadians(90))
                 .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90)) //
                 .strafeToLinearHeading(new Vector2d(-2, -30), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.toRadians(90))//
                .strafeToLinearHeading(new Vector2d(1, -30), Math.toRadians(90))






        //   .splineToLinearHeading(new Pose2d(58, -34, Math.toRadians(45)), Math.toRadians(90))
            //    .splineToLinearHeading(new Pose2d(58, -34, Math.toRadians(300)), Math.toRadians(0))
                .build());










//                        .splineToConstantHeading(new Vector2d(47, -33), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(59, -59), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(59, -33), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(58, -59), Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(58, -33, Math.toRadians(45)), Math.toRadians(90))
//                        .splineToLinearHeading(new Pose2d(58, -59, Math.toRadians(90)), Math.toRadians(90))
//                        .splineToConstantHeading(new Vector2d(58, -65), Math.toRadians(90))



//                        .strafeToConstantHeading(new Vector2d(47, -33)) // pickup sample 1
//                        .strafeToConstantHeading(new Vector2d(59,-59)) // spit out sample 1
//                        .strafeToConstantHeading(new Vector2d(59, -33)) // pickup sample 2
//                        .strafeToConstantHeading(new Vector2d(58, -59)) // spit ou sample 2
//                        .strafeToLinearHeading(new Vector2d(58, -38), Math.toRadians(45)) // pickup sample 3
//                        .strafeToLinearHeading(new Vector2d(58, -59), Math.toRadians(90)) // spit out sample 3
//                        .strafeToConstantHeading(new Vector2d(58, -65)) // pickup sample 1





        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}