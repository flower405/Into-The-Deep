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
public class RightAsync extends LinearOpMode {

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
    private boolean ReadyToClose = false;
    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -68, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Speciman1 = drive.actionBuilder(initialPose) // place first speciman
                .splineToConstantHeading(new Vector2d(0, -40), Math.toRadians(90))
             //   .afterTime(0, new InstantAction(() -> liftHeight = HighRung));
          .afterTime(0, new InstantAction(() -> lift.SpecimanDrop()));

        Action TrajectoryActionSpeciamn1 = Speciman1.build();

        TrajectoryActionBuilder Speciman2Pickup = Speciman1.endTrajectory().fresh() // push samples and Pickup Speciman 2
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

        Action TrajectoryActionSpeciman2Pickup = Speciman2Pickup.build();

        TrajectoryActionBuilder Speciman2 = Speciman2Pickup.endTrajectory().fresh() // placing speicman 2
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman2 = Speciman2.build();

        TrajectoryActionBuilder Speciman3Pickup = Speciman2.endTrajectory().fresh() // going to pickup third speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionSpeciman3Pickup = Speciman3Pickup.build();

        TrajectoryActionBuilder Speciman3 = Speciman3Pickup.endTrajectory().fresh() // placing third speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman3 = Speciman3.build();

        TrajectoryActionBuilder Speciman4Pickup = Speciman3.endTrajectory().fresh() // picking up fourth speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionSpeciman4Pickup = Speciman4Pickup.build();

        TrajectoryActionBuilder Speciman4 = Speciman4Pickup.endTrajectory().fresh() // placing fourth speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman4 = Speciman4.build();

        TrajectoryActionBuilder Speicman5Pickup = Speciman4.endTrajectory().fresh() // picking up the fifth speciman
                .splineToConstantHeading(new Vector2d(40,-50), Math.toRadians(0))
                .strafeTo(new Vector2d(40,-58));

        Action TrajectoryActionSpeciman5Pickup = Speicman5Pickup.build();

        TrajectoryActionBuilder Speciman5 = Speicman5Pickup.endTrajectory().fresh() // placing fifth speciman
                .strafeToConstantHeading(new Vector2d(3,-32));

        Action TrajectoryActionSpeciman5 = Speciman5.build();

        TrajectoryActionBuilder Park = Speciman5.endTrajectory().fresh() // parking the robot
                .strafeToConstantHeading(new Vector2d(40,-56));

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
        telemetry.addData("Outtake", OuttakePincher.getPosition());

        telemetry.update();


        Actions.runBlocking(
               new ParallelAction(
                        new LiftLoop(),
                       new SequentialAction(
                       new ParallelAction( // drive to drop S1 and lift slides and put arm up
                               TrajectoryActionSpeciamn1,
                               new InstantAction(() -> liftHeight = HighRung),
                               new InstantAction(() -> lift.OuttakePincherClose()),
                               new InstantAction(() -> lift.SpecimanDrop())
                       ),
                        new ParallelAction( // Stop Driving and drop slides down then let go of S
                                new WaitBar(),
                                new InstantAction(() -> liftHeight = SpecimanDrop),
                                new SpecimanDrop()
                        ),
                       new ParallelAction( // push samples in and go to pickup speciman 2 while getting slides ready
                               TrajectoryActionSpeciman2Pickup,
                               new AutoSlideWall()
                       ),
                       new ParallelAction( // wait at wall to pickup second speciman
                               new WaitWall(),
                               new SpecimanPickup()
                       ),
                       new ParallelAction( // drive to bar two and slides
                               TrajectoryActionSpeciman2,
                               new InstantAction(() -> liftHeight = HighRung)
                       ),
                       new ParallelAction( // stop and drop 2
                               new WaitBar(),
                               new InstantAction(() -> liftHeight = SpecimanDrop),
                               new SpecimanDrop()
                       ),
                       new ParallelAction( // Pickup 3 and slides ready
                               TrajectoryActionSpeciman3Pickup,
                               new AutoSlideWall()
                       ),
                       new ParallelAction( // stop and pickup 3
                               new WaitWall(),
                               new SpecimanPickup()
                       ),
                       new ParallelAction( // drive to bar 3
                               TrajectoryActionSpeciman3,
                               new InstantAction(() -> liftHeight = HighRung)
                       ),
                       new ParallelAction( // stop and drop 3
                               new WaitBar(),
                               new InstantAction(() -> liftHeight = SpecimanDrop),
                               new SpecimanDrop()
                       ),
                       new ParallelAction( // drive to wall pickup 4
                         TrajectoryActionSpeciman4Pickup,
                         new AutoSlideWall()
                       ),
                       new ParallelAction( // stop and pickup 4
                               new WaitWall(),
                               new SpecimanPickup()
                       ),
                       new ParallelAction( // drive to bar 4 and lift slides
                               TrajectoryActionSpeciman4,
                               new InstantAction(() -> liftHeight = HighRung)
                       ),
                       new ParallelAction( // stop and drop 4
                               new WaitBar(),
                               new InstantAction(() -> liftHeight = SpecimanDrop),
                               new SpecimanDrop()
                       ),
                       new ParallelAction( // drive to pickup 5
                               TrajectoryActionSpeciman5Pickup,
                               new AutoSlideWall()
                       ),
                       new ParallelAction( // stop and pcikup 5
                               new WaitWall(),
                               new SpecimanPickup()
                       ),
                       new ParallelAction( // drive to bar 5
                               TrajectoryActionSpeciman5,
                               new InstantAction(() -> liftHeight = HighRung)
                       ),
                       new ParallelAction( // stop and drop 5
                               new WaitBar(),
                               new InstantAction(() -> liftHeight = SpecimanDrop),
                               new SpecimanDrop()
                       ),
                       new ParallelAction( // drive to park and slides idle
                             TrajectoryActionPark,
                             new AutoSlideIdle()
                       )

                ) // sequential loop for robots sequence

               ) // parallel loop for lift heigh and actionns
        ); // for actions run blocking

    } // for run opmode
} // for class



