package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftMeet3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-10, -60, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45)) // place first/preload sample in bucket
                .strafeToLinearHeading(new Vector2d(-48, -39), Math.toRadians(90)) // pick up second sample
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45)) // place second sample in bucket
                .strafeToLinearHeading(new Vector2d(-60,-39), Math.toRadians(90)) // pick up third sample
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45)) // place third sample in bucket
                .strafeToLinearHeading(new Vector2d(-52,-45), Math.toRadians(135)) // pick up fourth sample
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45)) // place fourth sample in bucket
              //  .splineToLinearHeading(new Vector2d())







                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}


