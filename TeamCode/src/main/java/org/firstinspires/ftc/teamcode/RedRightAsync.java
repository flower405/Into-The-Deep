package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// import org.firstinspires.ftc.teamcode.drive.DriveConstants;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class RedRightAsync extends LinearOpMode {

    private IMU imu = null;
    PidControl2 lift = new PidControl2();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime SpecimanStartTimer = new ElapsedTime();
    ElapsedTime SpecimanTimer = new ElapsedTime();

    ElapsedTime SpecimanTransferTimer = new ElapsedTime();
    private Servo LeftArm, RightArm,  OuttakePincher = null;
    private DcMotor leftLift = null;
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;

    private enum specimanstart {
        SPECIMAN_READY,
        SPECIMAN_DROP,
        LIFT_IDLE
    }


    private enum SPECIMANSEQUENCE {
        PICKUP_READY,
        CLOSE_PINCHER,
        TRANSFER,
        DROP,
        RETRACT
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -52, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation

        TrajectoryActionBuilder Speciman1 = drive.actionBuilder(initialPose) // place first speciman
                .afterTime(0, telemetryPacket -> {
                    liftHeight = LiftConstants.HighRung;
                    return false;
                })
                .splineToConstantHeading(new Vector2d(0, -30), Math.toRadians(90));

        Action TrajectoryActionSpeciamn1 = Speciman1.build();

        TrajectoryActionBuilder Sample1 = Speciman1.endTrajectory().fresh() // pick up first sample
                .lineToY(-46) // push first sample into area
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -10), Math.toRadians(90))
                .lineToY(-63);

        Action TrajectoryActionSample1 = Sample1.build();

        TrajectoryActionBuilder Speciman2 = Sample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(3,-30), Math.toRadians(90));

        Action TrajectoryActionSpeciman2 = Speciman2.build();

        TrajectoryActionBuilder Speciamn3pickup = Speciman2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(40, -63), Math.toRadians(90));

        Action TrajectoryActionSpeciamn3pickup = Speciamn3pickup.build();

        TrajectoryActionBuilder Speciman3 = Speciamn3pickup.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6, -30), Math.toRadians(90));

        Action TrajectoryActionSpeciman3 = Speciman3.build();

        TrajectoryActionBuilder Park = Speciman3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(45, -52), Math.toRadians(90));

        Action TrajectoryActionPark = Park.build();


        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");

        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");


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
                new SequentialAction(
                        TrajectoryActionSpeciamn1
                )
        );









    }



}





















