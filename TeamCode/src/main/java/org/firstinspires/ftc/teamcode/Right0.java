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
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// goes straight to pushing samples
@Autonomous
public class Right0 extends LinearOpMode {
    private IMU imu = null;
    PidControl lift = new PidControl();
    public class SpecimenPickupReady implements Action {
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

    public class ArmIdle implements Action {
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
                lift.Idle();
            }
            if (t > 0.5) {
                liftHeight = liftRetracted;
            }
            if (t > 0.7) {
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
                liftHeight = SpecimenDropAuto;
            }
            if (t > 3){
                return false;
            } else {
                return true;
            }
        }
    }

    public class SpecimenPickup implements Action {
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
            }
            if (t > 0.9){
                return false;
            } else {
                return true;
            }
        }
    }
    public class SpecimenDrop implements Action {
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


        TrajectoryActionBuilder Sample1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(90)), Math.toRadians(90)) // push first sample
                .splineToLinearHeading(new Pose2d(37, -19, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -19, Math.toRadians(90)), Math.toRadians(-90))
                .lineToY(-47);

        Action TrajectoryActionSample1 = Sample1.build();

        TrajectoryActionBuilder Sample2 = Sample1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(49, -19, Math.toRadians(90)), Math.toRadians(90)) // push second sample
                .splineToLinearHeading(new Pose2d(50, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(59, -19, Math.toRadians(90)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(64, -48), Math.toRadians(270));

        Action TrajectoryActionSample2 = Sample2.build();


        TrajectoryActionBuilder Sample3 = Sample2.endTrajectory().fresh() // pushing all the sample in
                .splineToLinearHeading(new Pose2d(59, -19, Math.toRadians(90)), Math.toRadians(90)) // push third sample sample
                .splineToLinearHeading(new Pose2d(65, -8, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(67, -19, Math.toRadians(90)), Math.toRadians(-90))
                .lineToY(-45)
                .lineToY(-58);


        Action TrajectoryActionSample3 = Sample3.build();

        TrajectoryActionBuilder Specimen1 = Sample3.endTrajectory().fresh() // place first specimen
                .splineToConstantHeading(new Vector2d(40, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen1 = Specimen1.build();

        TrajectoryActionBuilder Specimen2Pickup = Specimen1.endTrajectory().fresh() // push samples and Pickup Specimen 2
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -58.5), Math.toRadians(270));


        Action TrajectoryActionSpecimen2Pickup = Specimen2Pickup.build();

        TrajectoryActionBuilder Specimen2 = Specimen2Pickup.endTrajectory().fresh() // placing specimen 2
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen2 = Specimen2.build();

        TrajectoryActionBuilder Specimen3Pickup = Specimen2.endTrajectory().fresh() // going to pickup third specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -58), Math.toRadians(270));


        Action TrajectoryActionSpecimen3Pickup = Specimen3Pickup.build();

        TrajectoryActionBuilder Specimen3 = Specimen3Pickup.endTrajectory().fresh() // placing third specimen
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen3 = Specimen3.build();

        TrajectoryActionBuilder Specimen4Pickup = Specimen3.endTrajectory().fresh() // picking up fourth specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270));


        Action TrajectoryActionSpecimen4Pickup = Specimen4Pickup.build();

        TrajectoryActionBuilder Specimen4 = Specimen4Pickup.endTrajectory().fresh() // placing fourth specimen
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen4 = Specimen4.build();

        TrajectoryActionBuilder Specimen5Pickup = Specimen4.endTrajectory().fresh() // picking up the fifth specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270));


        Action TrajectoryActionSpecimen5Pickup = Specimen5Pickup.build();

        TrajectoryActionBuilder Specimen5 = Specimen5Pickup.endTrajectory().fresh() // placing fifth specimen
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen5 = Specimen5.build();

        TrajectoryActionBuilder Park = Specimen5.endTrajectory().fresh() // parking the robot
                .strafeToConstantHeading(new Vector2d(38, -57));

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
                               TrajectoryActionSample1,
                                TrajectoryActionSample2,
                                TrajectoryActionSample3
//                            new ParallelAction( // push samples in and got to first pickup position
//                                TrajectoryActionSamplePush,
//                                new InstantAction(() -> liftHeight = LiftSpickup),
//                                new SpecimenPickupReady()
//                            ),
//                           new ParallelAction( // pickup first specimen
//                               new SpecimenPickup(),
//                               new WaitWall()
//                           ),
//                           new ParallelAction( // drive to bar 1
//                                TrajectoryActionSpecimen1,
//                               new InstantAction(() -> lift.SpecimanDrop())
//
//                           ),
//                           new ParallelAction( // drop 1
//                               new WaitBar(),
//                               new SpecimenDrop(),
//                               new InstantAction(() -> liftHeight = SpecimenDrop)
//                           ),
//                           new ParallelAction( // drive to pickup 2
//                               TrajectoryActionSpecimen2Pickup,
//                                new AutoSlideWall()
//                           ),
//                           new ParallelAction( // pickup 2
//                               new SpecimenPickup(),
//                               new WaitWall()
//                           ),
//                           new ParallelAction( // drive bar 2
//                               TrajectoryActionSpecimen2,
//                               new InstantAction(() -> lift.SpecimanDrop())
//
//                           ),
//                           new ParallelAction( // stop and drop 2
//                               new WaitBar(),
//                               new SpecimenDrop(),
//                               new InstantAction(() -> liftHeight = SpecimenDrop)
//                           ),
//                           new ParallelAction( // drive to pickup 3
//                               TrajectoryActionSpecimen3Pickup,
//                               new AutoSlideWall()
//                           ),
//                           new ParallelAction( // pickup 3
//                               new WaitWall(),
//                               new SamplePickup()
//                           ),
//                           new ParallelAction( // drive to bar 3
//                                TrajectoryActionSpecimen3,
//                               new InstantAction(() -> lift.SpecimanDrop())
//                           ),
//                           new ParallelAction( // stop and drop 3
//                                new WaitBar(),
//                                new InstantAction(() -> liftHeight = SpecimenDrop),
//                                new SpecimenDrop()
//                           ),
//                           new ParallelAction( // drive to pickup 4
//                                TrajectoryActionSpecimen4Pickup,
//                                new AutoSlideWall()
//                           ),
//                           new ParallelAction( // stop and pickup 4
//                                  new WaitWall(),
//                                  new SamplePickup()
//                           ),
//                           new ParallelAction( // drive to bar 4
//                               TrajectoryActionSpecimen4,
//                               new InstantAction(() -> lift.SpecimanDrop())
//                           ),
//                           new ParallelAction( // stop and drop 4
//                               new WaitBar(),
//                               new InstantAction(() -> liftHeight = SpecimenDrop),
//                               new SamplePickup()
//                           ),
//                           new ParallelAction( // drive to pickup 5
//                               TrajectoryActionSpecimen5Pickup,
//                               new AutoSlideWall()
//                           ),
//                           new ParallelAction( // stop and pickup 5
//                               new WaitWall(),
//                               new SpecimenPickup()
//                           ),
//                           new ParallelAction( // drive bar 5
//                                TrajectoryActionSpecimen5,
//                               new InstantAction(() -> lift.SpecimanDrop())
//                           ),
//                           new ParallelAction( // stop and drop 5
//                                new WaitBar(),
//                               new InstantAction(() -> liftHeight = SpecimenDrop),
//                               new SpecimenDrop()
//                           ),
//                           new ParallelAction( // park and arm idle
//                               TrajectoryActionPark,
//                               new ArmIdle()
//                           )



                ) // sequential loop for robots sequence

               ) // parallel loop for lift height and actions
        ); // for actions run blocking

    } // for run op mode
} // for class



