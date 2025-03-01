package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -68, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(0,-32), Math.toRadians(90)) // placing first speciman
                .splineToConstantHeading(new Vector2d(0,-43), Math.toRadians(0)) // pushing samples in
                .splineToConstantHeading(new Vector2d(32, -43), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35, -34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36,-50, Math.toRadians(340)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(46,-34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(46,-50, Math.toRadians(340)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(56, -34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(54,-50, Math.toRadians(340)), Math.toRadians(100))
                .strafeToLinearHeading(new Vector2d(45,-37), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(40, -58)) // pick up second speciman
                .strafeToConstantHeading(new Vector2d(3,-32)) // place second speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0)) // pickup third speciman
                .strafeTo(new Vector2d(40,-58))
                .strafeToConstantHeading(new Vector2d(3,-32)) // place third speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0)) // pickup fourth speciman
                .strafeTo(new Vector2d(40,-58))
                .strafeToConstantHeading(new Vector2d(3,-32)) // place fourth speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0)) // pickup fifth speciman
                .strafeTo(new Vector2d(40,-58))
                .strafeToConstantHeading(new Vector2d(3,-32)) // place fifth speciman
                .strafeToConstantHeading(new Vector2d(40,-56)) // park
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}