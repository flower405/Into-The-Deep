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
    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher, ClawElbow, ClawWrist, SlideServoLeft, SlideServoRight = null;
    private DcMotor leftLift = null;
    private CRServo spinny1 = null;
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -68, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Bucket1 = drive.actionBuilder(initialPose) // place first speciman
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(90))
                //   .afterTime(0, new InstantAction(() -> liftHeight = HighRung));
                .afterTime(0, new InstantAction(() -> lift.SpecimanDrop()));

        Action TrajectoryActionBucket1 = Bucket1.build();

        TrajectoryActionBuilder Sample2Pickup = Bucket1.endTrajectory().fresh() // push samples and Pickup Speciman 2
                .splineToConstantHeading(new Vector2d(0,-43), Math.toRadians(0))
                .afterTime(0, new InstantAction(() -> lift.WallPickup()))
                .splineToConstantHeading(new Vector2d(32, -43), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(35, -34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(36,-50, Math.toRadians(340)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(46,-34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(46,-50, Math.toRadians(340)), Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(56, -34, Math.toRadians(60)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(54,-50, Math.toRadians(340)), Math.toRadians(100))
                .strafeToLinearHeading(new Vector2d(45,-37), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(40, -58));

        Action TrajectoryActionSample2Pickup = Sample2Pickup.build();

        TrajectoryActionBuilder Bucket2 = Sample2Pickup.endTrajectory().fresh() // placing speicman 2
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionBucket2 = Bucket2.build();

        TrajectoryActionBuilder Sample3Pickup = Bucket2.endTrajectory().fresh() // going to pickup third speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionSample3Pickup = Sample3Pickup.build();

        TrajectoryActionBuilder Bucket3 = Sample3Pickup.endTrajectory().fresh() // placing third speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionBucket3 = Bucket3.build();

        TrajectoryActionBuilder Sample4Pickup = Bucket3.endTrajectory().fresh() // picking up fourth speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionSample4Pickup = Sample4Pickup.build();

        TrajectoryActionBuilder Bucket4 = Sample4Pickup.endTrajectory().fresh() // placing fourth speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionBucket4 = Bucket4.build();

        TrajectoryActionBuilder Park = Bucket4.endTrajectory().fresh() // picking up the fifth speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionPark = Park.build();



        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");

        RightArm.setDirection(Servo.Direction.REVERSE);
        SlideServoLeft.setDirection(Servo.Direction.REVERSE);



        lift.initAuto(hardwareMap);
        SlideServoLeft.setPosition(0);
        SlideServoRight.setPosition(0);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("Position", leftLift.getCurrentPosition());
        telemetry.addData("liftHeight", liftHeight);
        telemetry.addData("Outtake", OuttakePincher.getPosition());

        telemetry.update();


        Actions.runBlocking(
                new ParallelAction(
                        new LiftLoop(),
                        new SequentialAction(
                                new ParallelAction( // drive to drop S1 and lift slides and put arm up
                                        TrajectoryActionBucket1,
                                        new org.firstinspires.ftc.teamcode.AutoBucket1()
                                ),
                                new ParallelAction(
                                        new WaitBucket(),
                                        new SampleDrop()
                                ),
                                new ParallelAction(
                                   TrajectoryActionSample2Pickup,
                                    new AutoBucketToSample()

                                ),
                                new ParallelAction(
                                        new WaitSample(),
                                        new InstantAction(() -> spinny1.setPower(1)),
                                        new SamplePickup()
                                ),
                                new ParallelAction(
                                        TrajectoryActionBucket2

                                )


                        ) // sequential loop for robots sequence

                ) // parallel loop for lift heigh and actionns
        ); // for actions run blocking

    } // for run opmode
} // for class
