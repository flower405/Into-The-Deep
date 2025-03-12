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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// drops 2 specimen before pushing samples
@Disabled
@Autonomous
public class Right2 extends LinearOpMode {
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
    public class ArmRungToWall implements Action {
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
                liftHeight = LiftSpickup;
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


        TrajectoryActionBuilder Specimen1 = drive.actionBuilder(initialPose) // hang preload specimen
                .strafeToConstantHeading(new Vector2d(0, -27));

        Action TrajectoryActionSpecimen1 = Specimen1.build();

        TrajectoryActionBuilder Specimen2Pickup = Specimen1.endTrajectory().fresh() // Pickup second preload
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, -57.5), Math.toRadians(270));

        Action TrajectoryActionSpecimen2Pickup = Specimen2Pickup.build();

        TrajectoryActionBuilder Specimen2 = Specimen2Pickup.endTrajectory().fresh() // place second specimen
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place 2 specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen2 = Specimen2.build();

        TrajectoryActionBuilder Specimen3Pickup = Specimen2.endTrajectory().fresh() // push all the samples in and pickup 3rd specimen
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90)) // push first sample
                .splineToConstantHeading(new Vector2d(6, -45), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(15, -45), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(35,-45), Math.toRadians(-270))
                .splineToConstantHeading(new Vector2d(35, -17), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(47, -19), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(47, -47))
                .splineToConstantHeading(new Vector2d(43, -19), Math.toRadians(90)) // push second sample
                .splineToConstantHeading(new Vector2d(55, -9), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(58, -19), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(58, -47))
                .splineToConstantHeading(new Vector2d(53, -16), Math.toRadians(90)) // push third sample sample
                .splineToConstantHeading(new Vector2d(60.5, -8), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(63.5, -16), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(63.5, -55))
                .strafeToConstantHeading(new Vector2d(63.5, -56));

        Action TrajectoryActionSpecimen3Pickup = Specimen3Pickup.build();

        TrajectoryActionBuilder Specimen3 = Specimen3Pickup.endTrajectory().fresh() // hang third specimen
                .splineToConstantHeading(new Vector2d(40, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(10, -40), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -30), Math.toRadians(90));


        Action TrajectoryActionSpecimen3 = Specimen3.build();

        TrajectoryActionBuilder Specimen4Pickup = Specimen3.endTrajectory().fresh() // pickup 4th specimen
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
                .splineToConstantHeading(new Vector2d(40, -56.5), Math.toRadians(270));

        Action TrajectoryActionSpecimen5Pickup = Specimen5Pickup.build();

        TrajectoryActionBuilder Specimen5 = Specimen5Pickup.endTrajectory().fresh() // placing fifth specimen
                .splineToConstantHeading(new Vector2d(37, -45), Math.toRadians(180))//place fifth specimen
                .splineToConstantHeading(new Vector2d(12, -45), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(8, -30), Math.toRadians(90));

        Action TrajectoryActionSpecimen5 = Specimen5.build();

        TrajectoryActionBuilder Park = Specimen5.endTrajectory().fresh() // parking the robot
                .strafeToConstantHeading(new Vector2d(38, -56));

        Action TrajectoryActionPark = Park.build();


        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        lift.initTele(hardwareMap);
        lift.HSRetract();


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
                                new ParallelAction( // drive to place first preload
                                    TrajectoryActionSpecimen1,
                                    new InstantAction(() -> lift.SpecimanDrop()),
                                    new InstantAction(() -> lift.OuttakePincherClose()),
                                    new InstantAction(() -> liftHeight = HighRung)
                                ),
                                new ParallelAction( // stop and drop first specimen
                                    new WaitBar(),
                                    new SpecimenDrop(),
                                    new InstantAction(() -> liftHeight = SpecimenDrop)
                                ),
                                new ParallelAction( // drive to pickup second specimen lift ready
                                    TrajectoryActionSpecimen2Pickup,
                                    new ArmRungToWall()
                                ),
                                new ParallelAction( // stop at wall and pickup second specimen
                                    new WaitWall(),
                                    new SpecimenPickup()
                                ),
                                new ParallelAction( // drive to bar 2
                                    TrajectoryActionSpecimen2,
                                    new InstantAction(() -> lift.SpecimanDrop())
                                ),
                                new ParallelAction( // stop and drop specimen 2
                                    new WaitBar(),
                                    new SpecimenDrop(),
                                    new InstantAction(() -> liftHeight = SpecimenDrop)
                                ),
                                new ParallelAction( // push all the samples and pickup third specimen
                                    TrajectoryActionSpecimen3Pickup,
                                    new ArmRungToWall()
                                ),
                                new ParallelAction( // stop and pickup third specimen
                                    new WaitWall(),
                                    new SpecimenPickup()
                                ),
                                new ParallelAction( // drive to bar 3
                                    TrajectoryActionSpecimen3
                                ),
                                new ParallelAction( // stop and drop specimen 3
                                    new WaitBar(),
                                    new SpecimenDrop(),
                                    new InstantAction(() -> liftHeight = SpecimenDrop)
                                ),
                                new ParallelAction( // drive to pickup fourth specimen
                                    TrajectoryActionSpecimen4Pickup,
                                    new ArmRungToWall()
                                ),
                                new ParallelAction( // stop and pickup specimen 4
                                     new WaitWall(),
                                     new SpecimenPickup()
                                ),
                                new ParallelAction( // drive to drop specimen 4
                                    TrajectoryActionSpecimen4
                                ),
                                new ParallelAction( // stop and drop specimen 4
                                    new WaitBar(),
                                    new SpecimenDrop(),
                                    new InstantAction(() -> liftHeight = SpecimenDrop)
                                ),
                                new ParallelAction( // drive to pickup specimen 5
                                    TrajectoryActionSpecimen5Pickup,
                                    new ArmRungToWall()
                                ),
                                new ParallelAction( // stop and pickup 5
                                    new WaitWall(),
                                    new SpecimenPickup()
                                ),
                                new ParallelAction( // drive to bar 5
                                    TrajectoryActionSpecimen5
                                ),
                                new ParallelAction( // stop and drop 5
                                    new WaitBar(),
                                    new SpecimenDrop(),
                                    new InstantAction(() -> liftHeight = SpecimenDrop)
                                ),
                                new ParallelAction( // park
                                    TrajectoryActionPark,
                                    new ArmIdle()
                                )
                        ) // sequential loop for robots sequence

                ) // parallel loop for lift height and actions
        ); // for actions run blocking

    } // for run op mode
} // for class



