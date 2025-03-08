package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
public class RedRight1 extends LinearOpMode {

    private IMU imu = null;
    PidControl lift = new PidControl();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();


    ElapsedTime liftTimer = new ElapsedTime();
    LiftState liftState = LiftState.CLOSE_PINCHER1;


    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher, ClawElbow, ClawWrist, SlideServoLeft, SlideServoRight = null;
    private DcMotor leftLift = null;

    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    private enum LiftState {
        CLOSE_PINCHER1,
        DRIVE_BAR_1,
        SPECIMAN_DROP1,
        DRIVE_BEFORE_SAMPLE_PUSH,
        LIFT_READY_SPECIMAN2,
        PUSH_SAMPLE1,
        PICKUP_SPECIMAN2,
        DRIVE_BAR2,
        DROP_SPECIMAN2,
        DRIVE_WALL3,
        PICKUP_SPECIMAN3,
        DRIVE_BAR3,
        DROP3,
        PARK,
        DONE
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -68, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Speciman1 = drive.actionBuilder(initialPose) // place first speciman
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(90));

        Action TrajectoryActionSpeciamn1 = Speciman1.build();

        TrajectoryActionBuilder Sample1beforePush = Speciman1.endTrajectory().fresh() // pick up first sample
                .lineToY(-46) // push first sample into area
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(49, -10), Math.toRadians(90));

        Action TrajectoryActionSample1beforePush = Sample1beforePush.build();

        TrajectoryActionBuilder Sample1Push = Sample1beforePush.endTrajectory().fresh() // pick up first sample
                .lineToY(-57);

        Action TrajectoryActionSample1Push = Sample1Push.build();


        TrajectoryActionBuilder Speciman2 = Sample1Push.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(3, -25), Math.toRadians(90));

        Action TrajectoryActionSpeciman2 = Speciman2.build();


        TrajectoryActionBuilder Speciamn3pickup = Speciman2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(46, -54), Math.toRadians(90));

        Action TrajectoryActionSpeciamn3pickup = Speciamn3pickup.build();

        TrajectoryActionBuilder Speciman3 = Speciamn3pickup.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(6, -24),Math.toRadians(90));

        Action TrajectoryActionSpeciman3 = Speciman3.build();

        TrajectoryActionBuilder Park = Speciman3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(49,-57),Math.toRadians(90));

        Action TrajectoryActionPark = Park.build();


        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
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
        telemetry.addData("lift State", liftState);
        telemetry.addData("Outtake", OuttakePincher.getPosition());

        telemetry.update();


        while (liftState != LiftState.DONE) {
            switch (liftState) {
                case CLOSE_PINCHER1:
                    SlideServoLeft.setPosition(0);
                    SlideServoRight.setPosition(0);
                    liftTimer.reset();
                    LeftArm.setPosition(0.52);
                    RightArm.setPosition(0.52);
                    ClawElbow.setPosition(0.15);
                    ClawWrist.setPosition(0.67);
                    OuttakePincher.setPosition(0.5);
                    telemetry.update();
                    liftHeight = LiftConstants.BarAuto;
                    liftState = LiftState.DRIVE_BAR_1;
                    break;
                case DRIVE_BAR_1:
                    if (liftTimer.seconds() > 1.2) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionSpeciamn1
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.SPECIMAN_DROP1;
                        telemetry.update();
                    }
                    break;
                case SPECIMAN_DROP1:
                    if (liftTimer.seconds() > 0.1) {
                        liftHeight = LiftConstants.SpecimenDropAuto;
                        ClawElbow.setPosition(0.4);
                    }
                    if (liftTimer.seconds() > 0.4) {
                        OuttakePincher.setPosition(0.1);
                    }
                    if (liftTimer.seconds() > 0.8) {
                        LeftArm.setPosition(0.14);
                        RightArm.setPosition(0.14);
                        ClawElbow.setPosition(0.65);
                        ClawWrist.setPosition(0);
                    }
                    if (liftTimer.seconds() > 1.3) {
                        liftHeight = LiftConstants.LiftSpickup;
                        liftState = LiftState.DRIVE_BEFORE_SAMPLE_PUSH;
                        liftTimer.reset();
                    }

                    break;
                case DRIVE_BEFORE_SAMPLE_PUSH:
                    if (liftTimer.seconds() > 0.5) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionSample1beforePush,
                                        TrajectoryActionSample1Push
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.PICKUP_SPECIMAN2;
                    }
                    break;
                case PICKUP_SPECIMAN2:
                    if (liftTimer.seconds() > 0.2) {
                        OuttakePincher.setPosition(0.5);
                    }
                    if (liftTimer.seconds() > 0.6) {
                        liftHeight = LiftConstants.HighRung;
                    }
                    if (liftTimer.seconds() > 0.9) {
                        ClawElbow.setPosition(0);
                        LeftArm.setPosition(0.5);
                        RightArm.setPosition(0.5);
                        ClawWrist.setPosition(0.67);
                        liftState = LiftState.DRIVE_BAR2;
                        liftTimer.reset();
                    }
                    break;
                case DRIVE_BAR2:
                    if (liftTimer.seconds() > 0.5) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionSpeciman2
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.DROP_SPECIMAN2;
                    }
                    break;
                case DROP_SPECIMAN2:
                    if (liftTimer.seconds() > 0.1) {
                        liftHeight = LiftConstants.SpecimenDropAuto;
                        ClawElbow.setPosition(0.4);
                    }
                    if (liftTimer.seconds() > 0.4) {
                        OuttakePincher.setPosition(0.1);
                    }
                    if (liftTimer.seconds() > 0.8) {
                        LeftArm.setPosition(0.14);
                        RightArm.setPosition(0.14);
                        ClawElbow.setPosition(0.65);
                        ClawWrist.setPosition(0);
                    }
                    if (liftTimer.seconds() > 1.3) {
                        liftHeight = LiftConstants.LiftSpickup;
                        liftState = LiftState.DRIVE_WALL3;
                        liftTimer.reset();
                    }
                    break;
                case DRIVE_WALL3:
                    if (liftTimer.seconds() > 0.5) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionSpeciamn3pickup
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.PICKUP_SPECIMAN3;
                    }
                    break;
                case PICKUP_SPECIMAN3:
                    if (liftTimer.seconds() > 0.2) {
                        OuttakePincher.setPosition(0.5);
                        telemetry.update();
                    }
                    if (liftTimer.seconds() > 0.6) {
                        liftHeight = LiftConstants.HighRung;
                    }
                    if (liftTimer.seconds() > 0.9) {
                        ClawElbow.setPosition(0);
                        LeftArm.setPosition(0.5);
                        RightArm.setPosition(0.5);
                        ClawWrist.setPosition(0.67);
                        liftState = LiftState.DRIVE_BAR3;
                        liftTimer.reset();
                    }
                    break;
                case DRIVE_BAR3:
                    if (liftTimer.seconds() > 0.5) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionSpeciman3
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.DROP3;
                    }
                    break;
                case DROP3:
                    if (liftTimer.seconds() > 0.1) {
                        liftHeight = LiftConstants.SpecimenDropAuto;ClawElbow.setPosition(0.4);
                    }
                    if (liftTimer.seconds() > 0.4) {
                        OuttakePincher.setPosition(0.1);
                    }
                    if (liftTimer.seconds() > 0.8) {
                        LeftArm.setPosition(0.5);
                        RightArm.setPosition(0.5);
                        ClawElbow.setPosition(0.7);
                        ClawWrist.setPosition(0);
                    }
                    if (liftTimer.seconds() > 1.3) {
                        liftHeight = LiftConstants.liftRetracted;
                        liftState = LiftState.PARK;
                        liftTimer.reset();
                    }
                    break;
                case PARK:
                    if (liftTimer.seconds() > 0.5) {
                        Actions.runBlocking(
                                new SequentialAction(
                                        TrajectoryActionPark
                                )
                        );
                        liftTimer.reset();
                        liftState = LiftState.DONE;
                    }
                    break;
            }
            lift.setHeight(liftHeight);

        }
    }

}