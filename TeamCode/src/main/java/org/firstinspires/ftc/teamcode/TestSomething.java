package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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


@Disabled
@Autonomous
public class TestSomething extends LinearOpMode {

    private IMU imu = null;
    PidControl2 lift = new PidControl2();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();




    ElapsedTime liftTimer = new ElapsedTime();


    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher = null;
    private DcMotor leftLift = null;

    private boolean ReadyToClose = false;

    private boolean ReadyToDrop = false;
    private int liftOffset = 0;



    private enum SPECIMANSTART {
        SPECIMAN_READY,
        SPECIMAN_DROP,
        LIFT_IDLE
    }



    private enum SPECIMANDROP {
        SPECIMAN_PICKUP,
        CLOSE_OUTTAKE_PINCHER,
        DROP_READY,
        LIFT_RETURN_IDLE,

    }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -52, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        Action TrajectoryActionBar1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -25), Math.toRadians(90))
                .build();
        // this is going to second bar

        Action TrajectorySpeciman2Ready = drive.actionBuilder(drive.pose)
                .lineToY(-40)
                .splineToConstantHeading(new Vector2d(50, -50), Math.toRadians(90))
                .build();
        // move to position to put claw up

        Action TrajectoryPickUpSpeciman = drive.actionBuilder(drive.pose)
                .lineToY(-60)
                .build();
        //   pick up speciman

        Action TrajectoryArmFlipS2 = drive.actionBuilder(drive.pose)
                .lineToY(-50)
                .splineTo(new Vector2d(0, -52), Math.toRadians(270))
                .build();

        Action TrajectoryBar2 = drive.actionBuilder(drive.pose)
                .lineToY(-25)
                .build();
        // place on bar

        Action TrajectoryPark = drive.actionBuilder(drive.pose)
                .lineToY(-43)
                .splineTo(new Vector2d(50, -52), Math.toRadians(90))
                .build();







        // these are bad

       Action TrajectoryActionBar1test = drive.actionBuilder(initialPose)
               .lineToY(-25)
               .build();





        Action TrajectoryActionSample1 = drive.actionBuilder(drive.pose)
                .lineToY(-46)
                .splineToConstantHeading(new Vector2d(35, -38), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(42, -10), Math.toRadians(90))
                .lineToY(-52)
                .build();
        // this goes from bar to pushing sample into the park area, after this pick up second speciman

        Action TrajectoryActionSpecimanPickup1 = drive.actionBuilder(drive.pose)
                .lineToY(-60)
                .build();
        Action TrajectoryActionSpeciman2 = drive.actionBuilder(drive.pose)
                .lineToY(-52)
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

        LeftArm.setDirection(Servo.Direction.REVERSE);
        ClawRotate.setDirection(Servo.Direction.REVERSE);

        lift.initAuto(hardwareMap);







        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(RevOrientation));
        imu.resetYaw();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;
        telemetry.addData("Position",leftLift.getCurrentPosition());
        telemetry.addData("liftHeight",liftHeight);

        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(
                        TrajectoryActionBar1
                     //   TrajectorySpeciman2Ready
                )
        );








    }

}
