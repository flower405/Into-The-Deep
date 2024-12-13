package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -52, Math.toRadians(90)))
                .lineToY(-36)
                .lineToY(-46)
               // .splineToConstantHeading(new Vector2d(30,-40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -10), Math.toRadians(90))
                .lineToY(-52)
                .lineToY(-60)
                // pick up second speciamn
                        .lineToY(-52)
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(90))
                // place second speciman
                .lineToY(-46)
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -10), Math.toRadians(90))
                .lineToY(-52)

                // pick up third speciman
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(90))
                // place third speciman
                .lineToY(-46)
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(90))
                .lineToY(-52)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

