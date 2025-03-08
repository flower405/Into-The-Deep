package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Right0paths {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(17, -62.8, Math.toRadians(90))).splineToConstantHeading(new Vector2d(26, -51), Math.toRadians(0)) // first sample
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), Math.toRadians(90)) // push first sample
                .splineToLinearHeading(new Pose2d(37, -19, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -19, Math.toRadians(90)), Math.toRadians(270))
                .lineToY(-45)
                .splineToLinearHeading(new Pose2d(45, -19, Math.toRadians(90)), Math.toRadians(90)) // push second sample
                .splineToLinearHeading(new Pose2d(56.5, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(64, -19, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(64, -48), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(59, -19, Math.toRadians(90)), Math.toRadians(90)) // push third sample sample
                .splineToLinearHeading(new Pose2d(65, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(70, -19, Math.toRadians(90)), Math.toRadians(270))
                .lineToY(-45)
                .lineToY(-58)
                .splineToConstantHeading(new Vector2d(40, -40), Math.toRadians(180)) // place first Specimen
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup second speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place second specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup 3 speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place 3 specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup 4 speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place 4 specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup 4 speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place 4 specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
