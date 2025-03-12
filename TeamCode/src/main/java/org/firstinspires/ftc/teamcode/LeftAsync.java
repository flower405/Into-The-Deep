package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// import org.firstinspires.ftc.teamcode.drive.DriveConstants;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous
public class LeftAsync extends LinearOpMode {

    private IMU imu = null;
    PidControl lift = new PidControl();
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

    private CRServo spinny1 = null;
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;

    public class SampleDrop implements Action {
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
                lift.OuttakePincherOpen();
            }
            if (t > 0.6) {
                liftHeight = liftRetracted;
                lift.Idle();
            }
            if (t > 0.9) {
                lift.IntakeDown();
                spinny1.setPower(1);
            }
            if (t > 1.1){
                return false;
            } else {
                return true;
            }
        }
    }

    public class AutoBucketToSample implements Action {
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
            if (t > 0.5) {
                lift.ArmRetract();
            }
            if (t > 1) {
                liftHeight = liftRetracted;
            }
            if (t > 1.5) {
                lift.IntakeDown();
            }
            if (t > 1.7){
                return false;
            } else {
                return true;
            }
        }
    }

    public class SamplePickup implements Action {
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
                lift.IntakePincherClose();
            }
            if (t > 0.6) {
                lift.IntakeUp();
                spinny1.setPower(0);
            }
            if (t > 0.9) {
                lift.ArmTransfer();
            }
            if ( t > 1.2) {
                lift.IntakePincherOpenAuto();
            }
            if (t > 1.3) {
                lift.OuttakePincherClose();
            }
            if (t > 1.6){
                return false;
            } else {
                return true;
            }
        }
    }

    public class BucketIdle implements Action {
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
                lift.OuttakePincherOpen();
            }
            if (t > 0.6) {
                liftHeight = liftRetracted;
                lift.Idle();
            }
            if (t > 0.9){
                return false;
            } else {
                return true;
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -60, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Bucket1 = drive.actionBuilder(initialPose) // bucket 1
                .strafeToLinearHeading(new Vector2d(-54,-53), Math.toRadians(45));

        Action TrajectoryActionBucket1 = Bucket1.build();

        TrajectoryActionBuilder Sample2Pickup = Bucket1.endTrajectory().fresh() // pickup 2
                .strafeToLinearHeading(new Vector2d(-44, -38), Math.toRadians(90));

        Action TrajectoryActionSample2Pickup = Sample2Pickup.build();

        TrajectoryActionBuilder Bucket2 = Sample2Pickup.endTrajectory().fresh() // bucket 2
                .strafeToLinearHeading(new Vector2d(-54,-50), Math.toRadians(45));

        Action TrajectoryActionBucket2 = Bucket2.build();

        TrajectoryActionBuilder Sample3Pickup = Bucket2.endTrajectory().fresh()  // pickup 3
                .strafeToLinearHeading(new Vector2d(-54,-36), Math.toRadians(90));

        Action TrajectoryActionSample3Pickup = Sample3Pickup.build();

        TrajectoryActionBuilder Bucket3 = Sample3Pickup.endTrajectory().fresh() // bucket 3
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45));

        Action TrajectoryActionBucket3 = Bucket3.build();

        TrajectoryActionBuilder Sample4Pickup = Bucket3.endTrajectory().fresh() // pickup 4
                .strafeToLinearHeading(new Vector2d(-51,-33), Math.toRadians(145));

        Action TrajectoryActionSample4Pickup = Sample4Pickup.build();

        TrajectoryActionBuilder Bucket4 = Sample4Pickup.endTrajectory().fresh()  // drop 4
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45));

        Action TrajectoryActionBucket4 = Bucket4.build();

        TrajectoryActionBuilder Park = Bucket4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionPark = Park.build();



        // lift init

        spinny1 = hardwareMap.get(CRServo.class, "spinny1");





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

        telemetry.addData("liftHeight", liftHeight);


        telemetry.update();


        Actions.runBlocking(
                new ParallelAction(
                        new LiftLoop(),
                        new SequentialAction(
                                new ParallelAction( // bucket 1
                                        TrajectoryActionBucket1,
                                        new InstantAction(() -> lift.OuttakePincherClose()),
                                        new InstantAction(() -> lift.Bucket()),
                                        new InstantAction(() -> liftHeight = HighBucketAuto),
                                        new InstantAction(() -> lift.IntakePincherOpenAuto())
                                ),
                                new ParallelAction( // drop 1
                                        new WaitBucket(),
                                        new SampleDrop()
                                ),
                               TrajectoryActionSample2Pickup,
                                new SamplePickup(),
                                TrajectoryActionBucket2,
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.4),
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new ParallelAction( // drop 2
                                        new WaitBucket(),
                                        new SampleDrop()
                                ),
                                TrajectoryActionSample3Pickup,
                                new SamplePickup(),
                                TrajectoryActionBucket3,
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.4),
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new ParallelAction( // drop 3
                                        new WaitBucket(),
                                        new SampleDrop()
                                ),
                                TrajectoryActionSample4Pickup,
                                new SamplePickup(),
                                TrajectoryActionBucket4,
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.4),
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new ParallelAction( // drop 4
                                     new WaitBucket(),
                                     new BucketIdle()
                                )

                        ) // sequential loop for robots sequence

                ) // parallel loop for lift heigh and actionns
        ); // for actions run blocking

    } // for run opmode
} // for class
