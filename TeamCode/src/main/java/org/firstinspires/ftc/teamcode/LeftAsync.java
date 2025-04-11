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

    PidControl2 Slide = new PidControl2();
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

    private int SlideOffset = 0;
    private int SlideHeight = 0;
    private int storeSlideHeight = 0;
    private boolean SlideIncrease = false;
    private boolean SlideDecrease = false;

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
            if (t > 0.1) {
                lift.OuttakePincherOpen();
            }
            if (t > 0.4) {
                liftHeight = liftRetracted;
                lift.Idle();
            }
            if (t > 0.6) {
                lift.IntakeDown();
                spinny1.setPower(1);
            }
            if (t > 0.8){
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
                SlideHeight = HAuto;
            }
            if (t > 1.5){
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

            if (t > 0.4) {
                lift.IntakePincherClose();
            }
            if (t > 0.5) {
                lift.IntakeUp();
                spinny1.setPower(0);
                SlideHeight = HRetract;
            }
            if (t > 0.7) {
                lift.IntakePincherOpenAuto();
            }
            if (t > 1) {
                lift.ArmTransfer();
            }
            if (t > 1.2) {
                lift.OuttakePincherClose();
            }
            if (t > 1.4) {
                liftHeight = HighBucketAuto;
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
            if (t > 0.1) {
                lift.OuttakePincherOpen();
            }
            if (t > 0.4) {
                liftHeight = liftRetracted;
                lift.Idle();
            }
            if (t > 0.6){
                return false;
            } else {
                return true;
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -59, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Bucket1 = drive.actionBuilder(initialPose) // bucket 1
                .strafeToLinearHeading(new Vector2d(-52,-52), Math.toRadians(45));

        Action TrajectoryActionBucket1 = Bucket1.build();

        TrajectoryActionBuilder Sample2Pickup = Bucket1.endTrajectory().fresh() // pickup 2
                .strafeToLinearHeading(new Vector2d(-45, -36), Math.toRadians(90));

        Action TrajectoryActionSample2Pickup = Sample2Pickup.build();

        TrajectoryActionBuilder Bucket2 = Sample2Pickup.endTrajectory().fresh() // bucket 2
                .strafeToLinearHeading(new Vector2d(-49,-51), Math.toRadians(45));

        Action TrajectoryActionBucket2 = Bucket2.build();

        TrajectoryActionBuilder Sample3Pickup = Bucket2.endTrajectory().fresh()  // pickup 3
                .strafeToLinearHeading(new Vector2d(-56,-36), Math.toRadians(90));

        Action TrajectoryActionSample3Pickup = Sample3Pickup.build();

        TrajectoryActionBuilder Bucket3 = Sample3Pickup.endTrajectory().fresh() // bucket 3
                .strafeToLinearHeading(new Vector2d(-49,-52), Math.toRadians(45));

        Action TrajectoryActionBucket3 = Bucket3.build();

        TrajectoryActionBuilder Sample4Pickup = Bucket3.endTrajectory().fresh() // pickup 4
                .strafeToLinearHeading(new Vector2d(-51,-33), Math.toRadians(145));

        Action TrajectoryActionSample4Pickup = Sample4Pickup.build();

        TrajectoryActionBuilder Bucket4 = Sample4Pickup.endTrajectory().fresh()  // drop 4
                .strafeToLinearHeading(new Vector2d(-49,-52), Math.toRadians(45));

        Action TrajectoryActionBucket4 = Bucket4.build();

        TrajectoryActionBuilder Park = Bucket4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-45,-7), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-21,-7), Math.toRadians(0));

        Action TrajectoryActionPark = Park.build();


        spinny1 = hardwareMap.get(CRServo.class, "spinny1");

        Slide.initTele(hardwareMap);
        lift.initTele(hardwareMap);
        lift.IntakeUp();
        lift.Idleint();
        SlideHeight = HRetract;

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
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.5),
                                new ParallelAction( // bucket 1
                                        TrajectoryActionBucket1,
                                        new InstantAction(() -> lift.OuttakePincherClose()),
                                        new InstantAction(() -> lift.Bucket()),
                                        new InstantAction(() -> lift.IntakePincherOpenAuto())
                                ),
                                new SampleDrop(), // drop 1
                                new ParallelAction(
                                   TrajectoryActionSample2Pickup,
                                   new AutoBucketToSample()
                               ),
                                new SamplePickup(),
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                TrajectoryActionBucket2,
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new SampleDrop(), // drop 2
                                TrajectoryActionSample3Pickup,
                                new AutoBucketToSample(),
                                new SamplePickup(),
                                TrajectoryActionBucket3,
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.2),
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new SampleDrop(), // drop 3
                                TrajectoryActionSample4Pickup,
                                new SamplePickup(),
                                TrajectoryActionBucket4,
                                new InstantAction(() -> liftHeight = HighBucketAuto),
                                new SleepAction(0.2),
                                new InstantAction(() -> lift.Bucket()),
                                new SleepAction(1.5),
                                new BucketIdle(), // drop 4
                                TrajectoryActionPark,
                                new InstantAction(() -> SlideHeight = HRetract)
                        ) // sequential loop for robots sequence

                ) // parallel loop for lift height and actions
        ); // for actions run blocking

    } // for run opmode
} // for class
