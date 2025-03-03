package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


@Autonomous
public class RightAsync extends LinearOpMode {
    private IMU imu = null;
    PidControl lift = new PidControl();
    public class SpecimanPickupReady implements Action {
        private double BeginTs = -1;

        public int liftHeight, storeLiftHeight = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double t;
            if (BeginTs < 0) {
                BeginTs = com.acmerobotics.roadrunner.Actions.now();
                t = 0;
            } else {
                t = com.acmerobotics.roadrunner.Actions.now() - BeginTs;
            }
            if (t > 2) {
                lift.WallPickup();
            }
            if (t > 3){
                return false;
            } else {
                return true;
            }
        }
    }





    public class AutoSlideWall implements Action {
        private double BeginTs = -1;

        public int liftHeight, storeLiftHeight = 0;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double t;
            if (BeginTs < 0) {
                BeginTs = com.acmerobotics.roadrunner.Actions.now();
                t = 0;
            } else {
                t = com.acmerobotics.roadrunner.Actions.now() - BeginTs;
            }
            if (t > 0.5) {
                lift.WallPickup();
            }
            if (t > 2) {
                liftHeight = SpecimanDropAuto;
            }
            if (t > 3){
                return false;
            } else {
                return true;
            }
        }
    }

    public class SpecimanPickup implements Action {
        private double BeginTs = -1;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double t;
            if (BeginTs < 0) {
                BeginTs = com.acmerobotics.roadrunner.Actions.now();
                t = 0;
            } else {
                t = com.acmerobotics.roadrunner.Actions.now() - BeginTs;
            }
            if (t > 0.3) {
                lift.OuttakePincherClose();
            }
            if (t > 0.6) {
                liftHeight = HighRung;
                lift.SpecimanDrop();
            }
            if (t > 0.5){
                return false;
            } else {
                return true;
            }
        }
    }
    public class SpecimanDrop implements Action {
        private double BeginTs = -1;
        @Override
        public boolean run (@NonNull TelemetryPacket telemetryPacket){
            double t;
            if (BeginTs < 0) {
                BeginTs = com.acmerobotics.roadrunner.Actions.now();
                t = 0;
            } else {
                t = com.acmerobotics.roadrunner.Actions.now() - BeginTs;
            }
            if (t > 0.2) {
                lift.OuttakePincherOpen();
            }
            if (t > 0.5) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class LiftLoop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lift.setHeight(liftHeight);
            return true;
        }

    }

    public int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();

    private DcMotor leftLift = null;
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(17, -62.8, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder SamplePush = drive.actionBuilder(initialPose) // pushing all the sample in
                //.splineToConstantHeading(new Vector2d(26, -51), Math.toRadians(0)) // first sample
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
                .strafeToConstantHeading(new Vector2d(63.5, -56.5));

        Action TrajectoryActionSamplePush = SamplePush.build();




        TrajectoryActionBuilder Speciman1 = SamplePush.endTrajectory().fresh() // place first speciman
                .splineToConstantHeading(new Vector2d(40, -40), Math.toRadians(180)) // place first Specimen
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpeciamn1 = Speciman1.build();

        TrajectoryActionBuilder Speciman2Pickup = Speciman1.endTrajectory().fresh() // push samples and Pickup Speciman 2
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0)) // pickup second speciman
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270));





        Action TrajectoryActionSpeciman2Pickup = Speciman2Pickup.build();

        TrajectoryActionBuilder Speciman2 = Speciman2Pickup.endTrajectory().fresh() // placing speicman 2
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place second specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));
        Action TrajectoryActionSpeciman2 = Speciman2.build();

        TrajectoryActionBuilder Speciman3Pickup = Speciman2.endTrajectory().fresh() // going to pickup third speciman
                .splineToSplineHeading(new Pose2d(25, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(39, -62, Math.toRadians(90)), Math.toRadians(0));


        Action TrajectoryActionSpeciman3Pickup = Speciman3Pickup.build();

        TrajectoryActionBuilder Speciman3 = Speciman3Pickup.endTrajectory().fresh() // placing third speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman3 = Speciman3.build();

        TrajectoryActionBuilder Speciman4Pickup = Speciman3.endTrajectory().fresh() // picking up fourth speciman
                .splineToSplineHeading(new Pose2d(25, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(39, -62, Math.toRadians(90)), Math.toRadians(0));
//                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-62));

        Action TrajectoryActionSpeciman4Pickup = Speciman4Pickup.build();

        TrajectoryActionBuilder Speciman4 = Speciman4Pickup.endTrajectory().fresh() // placing fourth speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman4 = Speciman4.build();

        TrajectoryActionBuilder Speicman5Pickup = Speciman4.endTrajectory().fresh() // picking up the fifth speciman
                .splineToSplineHeading(new Pose2d(25, -40, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(39, -62, Math.toRadians(90)), Math.toRadians(0));

//                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
//                .strafeTo(new Vector2d(40,-62));

        Action TrajectoryActionSpeciman5Pickup = Speicman5Pickup.build();

        TrajectoryActionBuilder Speciman5 = Speicman5Pickup.endTrajectory().fresh() // placing fifth speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman5 = Speciman5.build();

        TrajectoryActionBuilder Park = Speciman5.endTrajectory().fresh() // parking the robot
                .strafeToConstantHeading(new Vector2d(40,-56));

        Action TrajectoryActionPark = Park.build();


        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");





        lift.initTele(hardwareMap);
        lift.HSRetract();
        lift.OuttakePincherOpen();

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;






        Actions.runBlocking(
               new ParallelAction(
                        new LiftLoop(),
                       new SequentialAction(
                            new ParallelAction( // push samples in and got to first pickup position
                                    TrajectoryActionSamplePush,
                                    new InstantAction(() -> liftHeight = LiftSpickup),
                                    new SpecimanPickupReady()
                            ),
                           new ParallelAction( // pickup first sepciman
                                   new SpecimanPickup(),
                                   new WaitWall()
                           ),
                           new ParallelAction(
                                    TrajectoryActionSpeciamn1
                           ),
                           new ParallelAction(
                                   new WaitBar(),
                                   new SpecimanDrop(),
                                   new InstantAction(() -> liftHeight = SpecimanDrop)
                           ),
                           new ParallelAction(
                                       TrajectoryActionSpeciman2Pickup,
                                        new AutoSlideWall()
                           ),
                           new ParallelAction(
                                   new SpecimanPickup(),
                                   new WaitWall()
                           ),
                           new ParallelAction(
                                   TrajectoryActionSpeciman2
                           ),
                           new ParallelAction(
                                   new WaitBar(),
                                   new SpecimanDrop(),
                                   new InstantAction(() -> liftHeight = SpecimanDrop)
                           )



                ) // sequential loop for robots sequence

               ) // parallel loop for lift heigh and actionns
        ); // for actions run blocking

    } // for run opmode
} // for class



