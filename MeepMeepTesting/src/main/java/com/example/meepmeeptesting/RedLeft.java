package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.LinearHeadingPath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedLeft {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -52, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-45, -36), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-49,-51), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-56,-36), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-49,-51), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-51,-33), Math.toRadians(145))
                .strafeToLinearHeading(new Vector2d(-49,-51), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(-45,-11), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-24,-10), Math.toRadians(0))



                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

