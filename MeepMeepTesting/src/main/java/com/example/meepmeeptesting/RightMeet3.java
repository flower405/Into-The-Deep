package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RightMeet3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)), Math.toRadians(90)) // push first sample
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -15, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -16, Math.toRadians(90)), Math.toRadians(-90))



                        //  .strafeToLinearHeading(new Vector2d(33, -45), Math.toRadians(50)) // go to pick up first sample
                //  .turnTo(Math.toRadians(315)) // turn to put first sample in obersvation zone
              //  .turnTo(Math.toRadians(50)) // turn to pick up second sample
               // .turnTo(Math.toRadians(315)) // put second sample in obersavtion zone
               // .turnTo(Math.toRadians(50)) // turn to pick up third sample
               // .turnTo(315) // place third sample in obersavtion zone



                        // .strafeToLinearHeading(new Vector2d(40, -63), Math.toRadians(90)) // pick up second speciman
             //   .strafeToConstantHeading(new Vector2d(0,-32)) // place second speciman
             //   .strafeToConstantHeading(new Vector2d(40,-63)) // pick up third sepciman
              //  .strafeToConstantHeading(new Vector2d(0,-32)) // place third speciman




               // .strafeToConstantHeading(new Vector2d(40,-63)) // pick up fourth sepciman
              //  .strafeToConstantHeading(new Vector2d(0,-32)) // place fourth speciman
              //  .strafeToConstantHeading(new Vector2d(52,-56)) // park
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

