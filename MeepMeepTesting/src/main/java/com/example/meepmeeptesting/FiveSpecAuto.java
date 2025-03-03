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
                .splineToConstantHeading(new Vector2d(-5,-32), Math.toRadians(90)) // placing first speciman
                .splineToConstantHeading(new Vector2d(0,-43), Math.toRadians(0)) // pushing samples in
                .splineToLinearHeading(new Pose2d(31, -43, Math.toRadians(0)), Math.toRadians(0)) // first sample
                .splineToConstantHeading(new Vector2d(43, -10), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(43, -60))
                .splineToConstantHeading(new Vector2d(49, -10), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(49, -60))

               // .splineToSplineHeading(new Pose2d(31, -14, Math.toRadians(0)), Math.toRadians(90))
               // .splineToSplineHeading(new Pose2d(45, -14, Math.toRadians(0)), Math.toRadians(90))
             //   .splineToSplineHeading(new Pose2d(45, -60, Math.toRadians(0)), Math.toRadians(90))


               /* .splineToSplineHeading(new Pose2d(45, -20, Math.toRadians(0)), Math.toRadians(90)) // second sample
                .splineToSplineHeading(new Pose2d(54, -14, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(54, -60, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(57, -20, Math.toRadians(0)), Math.toRadians(90)) // third sample
                .splineToSplineHeading(new Pose2d(59, -14, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(59, -60, Math.toRadians(0)), Math.toRadians(90))
*/




               // .splineToSplineHeading(new Pose2d(32, -28, Math.toRadians(60)), Math.toRadians(90))
               // .splineToSplineHeading(new Pose2d(36,-62, Math.toRadians(60)), Math.toRadians(100))
              //  .splineToSplineHeading(new Pose2d(44,-34, Math.toRadians(50)), Math.toRadians(90))
              //  .splineToSplineHeading(new Pose2d(46,-62, Math.toRadians(50)), Math.toRadians(100))
              //  .splineToSplineHeading(new Pose2d(50, -34, Math.toRadians(50)), Math.toRadians(90))
              //  .splineToSplineHeading(new Pose2d(54,-62, Math.toRadians(50)), Math.toRadians(100))
               // .strafeToLinearHeading(new Vector2d(45,-37), Math.toRadians(90))
             //   .strafeToConstantHeading(new Vector2d(40, -62)); // pick up second speciman





//                .splineToSplineHeading(new Pose2d(32, -28, Math.toRadians(60)), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(36,-62, Math.toRadians(340)), Math.toRadians(100))
//                .splineToSplineHeading(new Pose2d(44,-34, Math.toRadians(50)), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(46,-62, Math.toRadians(340)), Math.toRadians(100))
//                .splineToSplineHeading(new Pose2d(50, -34, Math.toRadians(50)), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(54,-62, Math.toRadians(340)), Math.toRadians(100))
//                .strafeToLinearHeading(new Vector2d(45,-37), Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(38, -60)) // pick up second speciman
//                .strafeToConstantHeading(new Vector2d(-1,-32)) // place second speciman
//                .splineToSplineHeading(new Pose2d(25, -40, Math.toRadians(90)), Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(36, -60, Math.toRadians(90)), Math.toRadians(0))


             //   .splineToConstantHeading(new Vector2d(40,-58), Math.toRadians(30)) // pickup third speciman


              //  .strafeToConstantHeading(new Vector2d(2,-32)) // place third speciman
             //   .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0)) // pickup fourth speciman
             //   .strafeTo(new Vector2d(40,-58))
              //  .strafeToConstantHeading(new Vector2d(5,-32)) // place fourth speciman
             //   .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0)) // pickup fifth speciman
             //   .strafeTo(new Vector2d(40,-58))
              //  .strafeToConstantHeading(new Vector2d(7,-32)) // place fifth speciman
              //  .strafeToConstantHeading(new Vector2d(40,-56)) // park
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}