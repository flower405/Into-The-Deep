package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
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

@Disabled
@Autonomous
public class NewAuto extends LinearOpMode {
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
            if (t > 1) {
                lift.WallPickup();
            }
            if (t > 3) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class ArmIdle implements Action {
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
                liftHeight = LiftSpickupAuto;
            }
            if (t > 3) {
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
            if (t > 0.1) {
                lift.OuttakePincherClose();
            }
            if (t > 0.3) {
                liftHeight = HighRung;
            }
            if (t > 0.5) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class SpecimenDrop implements Action {
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

    TrajectoryActionBuilder Sample1 = drive.actionBuilder(initialPose) //push first
            .splineToConstantHeading(new Vector2d(30,-45), Math.toRadians(0))//s to move
            .splineToConstantHeading(new Vector2d(37,-40), Math.toRadians(90))
            .splineToConstantHeading(new Vector2d(37,-16), Math.toRadians(90))//forwards
            .splineToConstantHeading(new Vector2d(45,-14), Math.toRadians(0))//semicircle
            .splineToConstantHeading(new Vector2d(47,-16), Math.toRadians(270))
            .strafeToConstantHeading(new Vector2d(47,-45));//back




            Action TrajectoryActionBuilderSample1 = Sample1.build();





        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");





        lift.initTele(hardwareMap);

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
                        TrajectoryActionBuilderSample1
                )











                )
        );




    }
}
