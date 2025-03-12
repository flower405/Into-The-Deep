package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class FiveAuto extends LinearOpMode {
    private IMU imu = null;
    PidControl lift = new PidControl();
    private CRServo spinny1 = null;
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
            if (t > 1) {
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
            if (t > 0) {
                lift.OuttakePincherClose();
            }
            if (t > 0.3) {
                liftHeight = HighRung;
            }
            if (t > 0.5){
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


        TrajectoryActionBuilder Sample1Pickup = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(39, -34, Math.toRadians(45)), Math.toRadians(90));


//                .strafeToConstantHeading(new Vector2d(47, -33),
//                null,
//                new ProfileAccelConstraint(-30, 80)
//        );
//                .splineToConstantHeading(new Vector2d(47, -33), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));

        Action TrajectoryActionSample1Pickup = Sample1Pickup.build();

        TrajectoryActionBuilder Sample1Out = Sample1Pickup.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(38, -47, Math.toRadians(300)), Math.toRadians(0));


//                .strafeToConstantHeading(new Vector2d(59,-59),
//               null,
//                new ProfileAccelConstraint(-30, 80));
//                .splineToConstantHeading(new Vector2d(59, -59), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));


        Action TrajectoryActionSample1Out = Sample1Out.build();

        TrajectoryActionBuilder Sample2Pickup = Sample1Out.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(50, -30, Math.toRadians(45)), Math.toRadians(90));


//                .strafeToConstantHeading(new Vector2d(59, -33),
//                null,
//                new ProfileAccelConstraint(-30, 80));
//                .splineToConstantHeading(new Vector2d(59, -33), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));


        Action TrajectoryActionSample2Pickup = Sample2Pickup.build();

        TrajectoryActionBuilder Sample2Out = Sample2Pickup.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(45, -34, Math.toRadians(300)), Math.toRadians(0));

//                .strafeToConstantHeading(new Vector2d(58, -59),
//               null,
//                        new ProfileAccelConstraint(-30,80));
//                .splineToConstantHeading(new Vector2d(58, -59), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));

        Action TrajectoryActionSample2Out = Sample2Out.build();

        TrajectoryActionBuilder Sample3Pickup = Sample2Out.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(58, -34, Math.toRadians(45)), Math.toRadians(90));


//                .strafeToLinearHeading(new Vector2d(58, -38), Math.toRadians(45),
//                   null,
//                        new ProfileAccelConstraint(-30, 80));
//                .splineToLinearHeading(new Pose2d(58, -33, Math.toRadians(45)), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));


        Action TrajectoryActionSample3Pickup = Sample3Pickup.build();

        TrajectoryActionBuilder Sample3Out = Sample3Pickup.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(58, -34, Math.toRadians(300)), Math.toRadians(0));


//                .strafeToLinearHeading(new Vector2d(58, -59), Math.toRadians(90),
//                       null,
//                        new ProfileAccelConstraint(-30, 80));
//                .splineToLinearHeading(new Pose2d(58, -59, Math.toRadians(90)), Math.toRadians(90),
//                        null,
//                        new ProfileAccelConstraint(-50, 90));

        Action TrajectoryActionSample3Out = Sample3Out.build();

        TrajectoryActionBuilder Specimen1Pickup = Sample3Out.endTrajectory().fresh()
//                .strafeToConstantHeading(new Vector2d(58, -65));
                .splineToConstantHeading(new Vector2d(58, -65), Math.toRadians(90));

        Action TrajectoryActionSpecimen1Pickup = Specimen1Pickup.build();

        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");

        lift.initTele(hardwareMap);
        lift.HSRetract();
        lift.OuttakePincherOpen();
        lift.Idle();

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
                        new SequentialAction( // pushing samples
                            new ParallelAction(
                                TrajectoryActionSample1Pickup,
                                new InstantAction(() -> lift.IntakeDown()),
                                new InstantAction(() -> spinny1.setPower(1))
                            ),
                            TrajectoryActionSample1Out,
                            new InstantAction(() -> spinny1.setPower(-1)),
                            new SleepAction(0.3),
                            new ParallelAction(
                                TrajectoryActionSample2Pickup,
                                 new InstantAction(() -> spinny1.setPower(1))
                            )
//                            TrajectoryActionSample2Out,
//                            new InstantAction(() -> spinny1.setPower(-1)),
//                                new SleepAction(0.2),
//                          new ParallelAction(
//                                  TrajectoryActionSample3Pickup,
//                                  new InstantAction(() -> spinny1.setPower(1))
//                          ),
//                            TrajectoryActionSample3Out,
//                         new InstantAction(() -> spinny1.setPower(-1)),
//                         new SleepAction(0.2)
                         //   TrajectoryActionSpecimen1Pickup

                        ) // sequential loop for robots sequence

                ) // parallel loop for lift height and actions
        ); // for actions run blocking




    } // for run op mode




} // for class



