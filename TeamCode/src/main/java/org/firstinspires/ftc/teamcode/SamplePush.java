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
public class SamplePush extends LinearOpMode {
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
            if (t > 0.5) {
                lift.WallPickup();
            }
            if (t > 0.7){
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
            if (t > 0.7) {
                liftHeight = LiftSpickupAuto;
            }

            if (t > 0.7){
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


    public class SampleIn implements Action {
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
                spinny1.setPower(1);
            }
            if (t > 0.8) {
                spinny1.setPower(0);
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
        Pose2d initialPose = new Pose2d(15.5, -64, Math.toRadians(90));
        //  Pose2d initialPose = new Pose2d(17, -62.8, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Sample1In = drive.actionBuilder(initialPose)
                .splineToLinearHeading(new Pose2d(37, -38, Math.toRadians(45)), Math.toRadians(45),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSample1In = Sample1In.build();

        TrajectoryActionBuilder Sample1Out = Sample1In.endTrajectory().fresh()
                .afterTime(0.5, new InstantAction(() -> spinny1.setPower(-1)))
                .strafeToLinearHeading(new Vector2d(39, -44), Math.toRadians(315),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSample1Out = Sample1Out.build();

        TrajectoryActionBuilder Sample2In = Sample1Out.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(43, -36), Math.toRadians(45),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSample2In = Sample2In.build();

        TrajectoryActionBuilder Sample2Out = Sample2In.endTrajectory().fresh()
                .afterTime(0.7, new InstantAction(() -> spinny1.setPower(-1)))
                .strafeToLinearHeading(new Vector2d(43, -44), Math.toRadians(315),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSample2Out = Sample2Out.build();


        TrajectoryActionBuilder Sample3In = Sample2Out.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(55.5, -37.5), Math.toRadians(45),
                new TranslationalVelConstraint(90));

        Action TrajectoryActionSample3In = Sample3In.build();

        TrajectoryActionBuilder Sample3Out = Sample3In.endTrajectory().fresh()
                .afterTime(0.9, new InstantAction(() -> spinny1.setPower(-1)))
                .strafeToLinearHeading(new Vector2d(45, -40), Math.toRadians(305),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSample3Out = Sample3Out.build();


        TrajectoryActionBuilder Specimen1Position = Sample3Out.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(37, -45), Math.toRadians(90),
                        new TranslationalVelConstraint(90))
                .strafeToConstantHeading(new Vector2d(37, -58.5),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen1Position = Specimen1Position.build();

        TrajectoryActionBuilder Specimen = Specimen1Position.endTrajectory().fresh()
                .afterDisp(53, new InstantAction(() -> liftHeight = SpecimenDrop))
                .strafeToLinearHeading(new Vector2d(-8, -30), Math.toRadians(90),
                        new TranslationalVelConstraint(90))
                ;

        Action TrajectoryActionSpecimen = Specimen.build();


        TrajectoryActionBuilder Specimen2Pickup = Specimen.endTrajectory().fresh()
                .afterDisp(49, new InstantAction(() -> lift.OuttakePincherClose()))
                .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen2Pickup = Specimen2Pickup.build();

        TrajectoryActionBuilder Specimen2 = Specimen2Pickup.endTrajectory().fresh()
                .afterDisp(46, new InstantAction(() -> liftHeight = SpecimenDrop))
                .strafeToLinearHeading(new Vector2d(-6, -30), Math.toRadians(90),
                        new TranslationalVelConstraint(90));


        Action TrajectoryActionSpecimen2 = Specimen2.build();

        TrajectoryActionBuilder Specimen3Pickup = Specimen2.endTrajectory().fresh()
                .afterDisp(49, new InstantAction(() -> lift.OuttakePincherClose()))
                .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90),
                        new TranslationalVelConstraint(90));


        Action TrajectoryActionSpecimen3Pickup = Specimen3Pickup.build();

        TrajectoryActionBuilder Specimen3 = Specimen3Pickup.endTrajectory().fresh()
                .afterDisp(46, new InstantAction(() -> liftHeight = SpecimenDrop))
                .strafeToLinearHeading(new Vector2d(-4, -30), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen3 = Specimen3.build();

        TrajectoryActionBuilder Specimen4Pickup = Specimen3.endTrajectory().fresh()
                .afterDisp(48, new InstantAction(() -> lift.OuttakePincherClose()))
                .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen4Pickup = Specimen4Pickup.build();

        TrajectoryActionBuilder Specimen4 = Specimen4Pickup.endTrajectory().fresh()
                .afterDisp(46, new InstantAction(() -> liftHeight = SpecimenDrop))
                .strafeToLinearHeading(new Vector2d(-2, -30), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen4 = Specimen4.build();


        TrajectoryActionBuilder Specimen5Pickup = Specimen4.endTrajectory().fresh()
                .afterDisp(48, new InstantAction(() -> lift.OuttakePincherClose()))
                .strafeToLinearHeading(new Vector2d(37, -59.5), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen5Pickup = Specimen5Pickup.build();

        TrajectoryActionBuilder Specimen5 = Specimen5Pickup.endTrajectory().fresh()
                .afterDisp(46, new InstantAction(() -> liftHeight = SpecimenDrop))
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(90),
                        new TranslationalVelConstraint(90));

        Action TrajectoryActionSpecimen5 = Specimen5.build();











        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");

        lift.initTele(hardwareMap);
        lift.HSRetract();
        lift.OuttakePincherOpen();
        lift.IntakeUp();
        lift.Idle();
        liftHeight = liftRetracted;


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
                                    TrajectoryActionSample1In,
                                    new InstantAction(() -> lift.IntakeDown()),
                                    new InstantAction(() -> lift.HSLow()),
                                    new InstantAction(() -> spinny1.setPower(1))
                            ),
                                TrajectoryActionSample1Out,
                            new SleepAction(0.4),
                            new ParallelAction(
                                TrajectoryActionSample2In,
                                new InstantAction(() -> spinny1.setPower(1))
                            ),
                            new SleepAction(0.2),
                            new ParallelAction(
                                TrajectoryActionSample2Out,
                                new InstantAction(() -> lift.HSRetract())
                            ),
                            new SleepAction(0.5),
                            new ParallelAction(
                                TrajectoryActionSample3In,
                                new InstantAction(() -> spinny1.setPower(1))
                            ),
                            new SleepAction(0.2),
                            new ParallelAction(
                                TrajectoryActionSample3Out,
                                new InstantAction(() -> liftHeight = LiftSpickupAuto),
                                new SpecimenPickupReady()

                            ),
                           new SleepAction(0.5),
                           new ParallelAction(
                               TrajectoryActionSpecimen1Position,
                                   new InstantAction(() -> lift.IntakeUp()),
                                   new InstantAction(() -> spinny1.setPower(0))
                           ),
                                new InstantAction(() -> lift.OuttakePincherClose()),
                                new SleepAction(0.1)
//                                new InstantAction(() -> liftHeight = HighRung),
//                                new SleepAction(0.05),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen,
//                                        new InstantAction(() -> lift.SpecimanDrop())
//                                        )
//                                new InstantAction(() -> lift.OuttakePincherOpen()),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen2Pickup,
//                                        new AutoSlideWall()
//                                ),
//                                new InstantAction(() -> liftHeight = HighRung),
//                                new SleepAction(0.05),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen2,
//                                        new InstantAction(() -> lift.SpecimenDropAuto())
//                                ),
//                                new InstantAction(() -> lift.OuttakePincherOpen()),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen3Pickup,
//                                        new AutoSlideWall()
//                                ),
//                                new InstantAction(() -> liftHeight = HighRung),
//                                new SleepAction(0.05),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen3,
//                                        new InstantAction(() -> lift.SpecimenDropAuto())
//                                ),
//                                new InstantAction(() -> lift.OuttakePincherOpen()),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen4Pickup,
//                                        new AutoSlideWall()
//                                ),
//                                new InstantAction(() -> liftHeight = HighRung),
//                                new SleepAction(0.05),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen4,
//                                        new InstantAction(() -> lift.SpecimenDropAuto())
//                                ),
//                                new InstantAction(() -> lift.OuttakePincherOpen()),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen5Pickup,
//                                        new AutoSlideWall()
//                                ),
//                                new InstantAction(() -> liftHeight = HighRung),
//                                new SleepAction(0.05),
//                                new ParallelAction(
//                                        TrajectoryActionSpecimen5,
//                                        new InstantAction(() -> lift.SpecimenDropAuto())
//                                )

                        ) // sequential loop for robots sequence
                ) // parallel loop for lift height and actions
        ); // for actions run blocking




    } // for run op mode




} // for class