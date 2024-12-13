package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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


@Autonomous
public class RedRightAsync extends LinearOpMode {

    private IMU imu = null;
    PidControl2 lift = new PidControl2();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();

    ElapsedTime liftTimer = new ElapsedTime();
    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher = null;
    private DcMotor leftLift = null;
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -52, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        Action TrajectoryActionBar1 = drive.actionBuilder(initialPose)
                .lineToY(-43)
                .build();
        // this is going to second bar

        Action TrajectoryActionSample1 = drive.actionBuilder(drive.pose)
                .lineToY(-46)
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -10), Math.toRadians(90))
                .lineToY(-52)
                .build();
        // this goes from bar to pushing sample into the park area, after this pick up second speciman

        Action TrajectoryActionSpeciman2 = drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(90))
                .build();
        // this is just to bar now go push second sample

        Action TrajectoryActionSample2 = drive.actionBuilder(drive.pose)
                .lineToY(-46)
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(54, -10), Math.toRadians(90))
                .lineToY(-52)
                .build();
        // this is then pushing second sample into park area and park

        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        ClawRotate = hardwareMap.get(Servo.class, "Claw_Rotate");
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
                new ParallelAction(
                        TrajectoryActionBar1


                ));





    }
}





















