package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Disabled
@Autonomous
public class Left extends LinearOpMode {

    private IMU imu = null;
    PidControl lift = new PidControl();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();
    ElapsedTime liftTimer = new ElapsedTime();
   // LiftState liftState = LiftState.SAMPLE1;
    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher, ClawElbow, ClawWrist, SlideServoLeft, SlideServoRight, IntakeFlip, IntakePincher = null;
    private DcMotor leftLift = null;
    private CRServo spinny1 = null;

    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-34, -60, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Sample1 = drive.actionBuilder(initialPose) // place first speciman
                .strafeToLinearHeading(new Vector2d(-50,-46), Math.toRadians(45));

        Action TrajectoryActionSample1 = Sample1.build();

        TrajectoryActionBuilder PickupSample2 = Sample1.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-48, -39), Math.toRadians(90));

        Action TrajectoryActionPickupSample2 = PickupSample2.build();

        TrajectoryActionBuilder Sample2 = PickupSample2.endTrajectory().fresh()
                ;

        Action TrajectoryActionSample2 = Sample2.build();

        TrajectoryActionBuilder PickupSample3 = Sample2.endTrajectory().fresh()
                ;

        Action TrajectoryActionPickupSample3 = PickupSample3.build();

        TrajectoryActionBuilder Sample3 = PickupSample3.endTrajectory().fresh()
                ;

        Action TrajectoryActionSample3 = Sample3.build();

        TrajectoryActionBuilder PickupSample4 = Sample3.endTrajectory().fresh()
                ;
        Action TrajectoryActionPickupSample4 = PickupSample4.build();




        // lift init
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");
        IntakePincher = hardwareMap.get(Servo.class, "IntakePincher");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");

        RightArm.setDirection(Servo.Direction.REVERSE);
        SlideServoLeft.setDirection(Servo.Direction.REVERSE);
        IntakeFlip.setDirection(Servo.Direction.REVERSE);


        lift.initAuto(hardwareMap);
       OuttakePincher.setPosition(0.5);
       SlideServoRight.setPosition(0);
        SlideServoLeft.setPosition(0);

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
     //   telemetry.addData("lift State", liftState);
        telemetry.update();


//        while (liftState != LiftState.DONE) {
//            switch (liftState) {
//                case SAMPLE1:
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    TrajectoryActionSample1
//                            )
//                    );
//                    liftTimer.reset();
//                    liftState = LiftState.DROP_SAMPLE1;
//                    break;
//                case DROP_SAMPLE1:
//                    if (liftTimer.seconds() > 0) {
//                        SlideServoLeft.setPosition(0);
//                        SlideServoRight.setPosition(0);
//                        liftHeight = LiftConstants.HighBucket;
//                    }
//                    if (liftTimer.seconds() > 0.6) {
//                        RightArm.setPosition(0.9);
//                        LeftArm.setPosition(0.9);
//                        ClawWrist.setPosition(0);
//                        ClawElbow.setPosition(0);
//                    }
//                    if (liftTimer.seconds() > 1.3) {
//                        OuttakePincher.setPosition(0.1);
//                    }
//                    if (liftTimer.seconds() > 1.7) {
//                        LeftArm.setPosition(0.5);
//                        RightArm.setPosition(0.5);
//                        ClawElbow.setPosition(0.7);
//                    }
//                    if (liftTimer.seconds() > 2) {
//                        liftHeight = LiftConstants.liftRetracted;
//                    }
//                    if (liftTimer.seconds() > 2.4) {
//                        IntakeFlip.setPosition(0);
//                        spinny1.setPower(1);
//
//                    }
//                    break;
//                case PICKUP_SAMPLE2:
//                   if (liftTimer.seconds() > 0.5)
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    TrajectoryActionPickupSample2
//                            )
//                    );
//                    liftTimer.reset();
//                    liftState = LiftState.DONE;
//                    break;
//            }
//            lift.setHeight(liftHeight);
//
        }
  }
//
