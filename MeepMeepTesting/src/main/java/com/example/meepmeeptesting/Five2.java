package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Five2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(17, -62.8, Math.toRadians(90))).splineToConstantHeading(new Vector2d(26, -51), Math.toRadians(0)) // first sample
                .splineToConstantHeading(new Vector2d(35, -40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -19), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -11), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -19), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(47, -47))
                .splineToConstantHeading(new Vector2d(43, -19), Math.toRadians(90)) // push second sample
                .splineToConstantHeading(new Vector2d(55, -9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -19), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(58, -47))
                .splineToConstantHeading(new Vector2d(53, -16), Math.toRadians(90)) // push third sample sample
                .splineToConstantHeading(new Vector2d(60.5, -8), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63.5, -16), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(63.5, -55))
                .strafeToConstantHeading(new Vector2d(63.5, -57))
                .splineToConstantHeading(new Vector2d(40, -40), Math.toRadians(180)) // place first Specimen
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup second speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place second specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90))



























                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
