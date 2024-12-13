package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class RedRight extends LinearOpMode {

    private IMU imu = null;
    PidControl2 lift = new PidControl2();
    private int liftHeight, storeLiftHeight = 0;
    ElapsedTime imuTimer = new ElapsedTime();




    ElapsedTime liftTimer = new ElapsedTime();
    LiftState liftState = LiftState.CLOSE_PINCHER1;


    private Servo LeftArm, RightArm, ClawRotate, OuttakePincher = null;
    private DcMotor leftLift = null;

    private boolean ReadyToClose = false;

    private boolean ReadyToDrop = false;
    private int liftOffset = 0;


    private enum LiftState {
       CLOSE_PINCHER1,
       LIFT_EXTEND1,
       DRIVE_BAR_1,
       SPECIMAN_DROP1,
       LIFT_RETRACT1,
       DRIVE_SAMPLE1,
       SPECIMAN_PICKUP1,
       ARM_FLIP1,
       DRIVE_SPECIMAN_PICKUP,
       CLOSE_PINCHER2,
       TRANSFER1,
       DRIVE_BAR2,
       SPECIAN_DROP2,
       LIFT_RETRACT2,
       DRIVE_SAMPLE2


   }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -52, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        Action TrajectoryActionBar1 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0, -23), Math.toRadians(90))
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
        telemetry.addData("lift State",liftState);
        telemetry.update();

        while(liftState != LiftState.DRIVE_SAMPLE2){
        switch (liftState) {
            case CLOSE_PINCHER1:
                liftTimer.reset();
                LeftArm.setPosition(0.35);
                RightArm.setPosition(0.35);
                ClawRotate.setPosition(0.35);
                OuttakePincher.setPosition(0.4);
                telemetry.update();
                    liftState = LiftState.LIFT_EXTEND1;


                break;
            case LIFT_EXTEND1:
                if (liftTimer.seconds() > 0.2) {
                    liftHeight = LiftConstants.BarAuto;
                    liftTimer.reset();
                    telemetry.update();
                    liftState = liftState.DRIVE_BAR_1;
                }
                break;
            case DRIVE_BAR_1:
              if (liftTimer.seconds() > 1) {
                  Actions.runBlocking(
                          new SequentialAction(
                                  TrajectoryActionBar1
                          )
                  );
                  liftTimer.reset();
                  liftState = LiftState.SPECIMAN_DROP1;
              }
              break;
            case SPECIMAN_DROP1:
                if (liftTimer.seconds() > 0.2) {
                    ClawRotate.setPosition(0.6);
                }
                if (liftTimer.seconds() > 0.5) {
                    liftHeight = LiftConstants.SpecimanDropAuto1;
                } if (liftTimer.seconds() > 0.8) {
                liftHeight = LiftConstants.SpecimanDropAuto2;
            } if (liftTimer.seconds() > 1.1) {
                liftHeight = LiftConstants.SpecimanDropAuto3;
            //    } if (liftTimer.seconds() > 1.4) {
               // liftHeight = LiftConstants.SpecimanDropAuto4;
            }
                if (liftTimer.seconds() > 2) {
                    OuttakePincher.setPosition(0);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_RETRACT1;
                }
                break;
            case LIFT_RETRACT1:
                if (liftTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.35);
                    RightArm.setPosition(0.35);
                    ClawRotate.setPosition(0.35);
                }
                if (liftTimer.seconds() > 0.7) {
                    liftHeight = LiftConstants.liftRetracted;
                 //   liftState = LiftState.DRIVE_SAMPLE1;
                }
                break;
            case DRIVE_SAMPLE1:
                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryActionSample1
                        )
                );
                liftTimer.reset();
                liftState = LiftState.SPECIMAN_PICKUP1;
                break;
            case SPECIMAN_PICKUP1:
                liftHeight = LiftConstants.LiftSpickup;
                OuttakePincher.setPosition(0.5); //closes claw when going by strings
                liftTimer.reset();
                liftState = LiftState.ARM_FLIP1;
                break;
            case ARM_FLIP1:
                if (liftTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.7);
                    RightArm.setPosition(0.7);
                    ClawRotate.setPosition(0.3);
                    OuttakePincher.setPosition(0);
                    liftTimer.reset();
                    liftState = LiftState.DRIVE_SPECIMAN_PICKUP;
                }
                break;
            case DRIVE_SPECIMAN_PICKUP:
                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryActionSpecimanPickup1
                        )
                );
                liftTimer.reset();
                liftState = LiftState.CLOSE_PINCHER2;
                break;
            case CLOSE_PINCHER2:
                if (liftTimer.seconds() > 1) {
                    OuttakePincher.setPosition(0.4);
                    liftTimer.reset();
                    liftState = LiftState.TRANSFER1;
                }
                break;
            case TRANSFER1:
                if (liftTimer.seconds() > 0.2) {
                    liftHeight = LiftConstants.HighRung;
                }
                if (liftTimer.seconds() > 1.5) {
                    LeftArm.setPosition(0.4); // what are these value?
                    RightArm.setPosition(0.4);
                    ClawRotate.setPosition(0.9);
                    liftTimer.reset();
                    liftState = LiftState.DRIVE_BAR2;
                }
                break;
            case DRIVE_BAR2:
                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryActionSpeciman2
                        )
                );
                liftTimer.reset();
                liftState = LiftState.SPECIAN_DROP2;
                break;
            case SPECIAN_DROP2:
                if (liftTimer.seconds() > 1) {
                    liftHeight = LiftConstants.SpecimanDrop;
                }
                if (liftTimer.seconds() > 3) {
                    OuttakePincher.setPosition(0);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_RETRACT2;
                }
                break;
            case LIFT_RETRACT2:
                if (liftTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.55);
                    RightArm.setPosition(0.55);
                    ClawRotate.setPosition(0.3);
                }
                if (liftTimer.seconds() > 0.7) {
                    liftHeight = LiftConstants.liftRetracted;
                    liftState = LiftState.DRIVE_SAMPLE2;
                }
            case DRIVE_SAMPLE2:
                Actions.runBlocking(
                        new SequentialAction(
                                TrajectoryActionSample2
                        )
                );
                break;
        }
          lift.setHeight(liftHeight);
        }
    }

}
