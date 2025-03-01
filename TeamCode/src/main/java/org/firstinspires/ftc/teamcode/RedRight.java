package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// import org.firstinspires.ftc.teamcode.drive.DriveConstants;
// import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled

@Autonomous
public class RedRight extends LinearOpMode {

    private IMU imu = null;
    PidControl lift = new PidControl();
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
        DRIVE_ARM_READY,
        ARM_READY,
        DRIVE_PICKUP_SPECIMAN2,
        PICKUP_SPECIMAN2,
        LIFT_EXTEND2,
        DRIVE_ARM_TRANSFER,
        ARM_TRANSFER2,
        DRIVE_BAR2,
        SPECIMAN_DROP2,
        BACK_UP,
        LIFT_RETRACT2,
        PARK


   }





    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10,-60, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        //Define robots starting position and orientation


        TrajectoryActionBuilder Bar1 = drive.actionBuilder(initialPose) // drop prload cube
         .splineToConstantHeading(new Vector2d(0, -32), Math.toRadians(90));

        Action TrajectoryActionBar1 = Bar1.build();

        TrajectoryActionBuilder Speciman2Ready = Bar1.endTrajectory().fresh() // go to pick up second specian
                .strafeTo(new Vector2d(40, -50));

        Action TrajectoryActionSpeciman2Ready = Speciman2Ready.build();
        // go to positon to put up arm

        TrajectoryActionBuilder PickUpSpeciman = Speciman2Ready.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -63));

        Action TrajectoryActionPickUpSpeciman = PickUpSpeciman.build();

        TrajectoryActionBuilder ArmFlipS2 = PickUpSpeciman.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(3, -45), Math.toRadians(270));

        Action TrajectoryArmFlipS2 = ArmFlipS2.build();

     TrajectoryActionBuilder Bar2 = ArmFlipS2.endTrajectory().fresh()
             .strafeTo(new Vector2d(5, -30));

     Action TrajectoryBar2 = Bar2.build();

     TrajectoryActionBuilder BackUp = Bar2.endTrajectory().fresh()
             .strafeTo(new Vector2d(5, -40))
             .afterTime(2, telemetryPacket -> {
                 liftState = LiftState.LIFT_EXTEND1;
                 return false;
             })
            .strafeToLinearHeading(new Vector2d(45, -52), Math.toRadians(90));

     Action TrajectoryBackUp = BackUp.build();



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

        Actions.runBlocking(
              new ParallelAction(
                      TrajectoryActionBar1

              )
        );







        while(liftState != LiftState.PARK){
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
                  telemetry.update();
              }
              break;
            case SPECIMAN_DROP1:
                if (liftTimer.seconds() > 1) {
                    ClawRotate.setPosition(0.6);
                }
                if (liftTimer.seconds() > 1.2) {
                    liftHeight = LiftConstants.SpecimanDropAuto;
                } if (liftTimer.seconds() > 1.5) {
                liftHeight = LiftConstants.SpecimanDropAuto2;
            } if (liftTimer.seconds() > 1.8) {
                liftHeight = LiftConstants.SpecimanDropAuto3;
            //    } if (liftTimer.seconds() > 1.4) {
               // liftHeight = LiftConstants.SpecimanDropAuto4;
            }
                if (liftTimer.seconds() > 2) {
                    OuttakePincher.setPosition(0);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_RETRACT1;
                    telemetry.update();
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
                    liftTimer.reset();
                    liftState = LiftState.DRIVE_ARM_READY;
                    telemetry.update();
                }
                break;
            case DRIVE_ARM_READY:
                if (liftTimer.seconds() > 0.5) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    TrajectoryActionSpeciman2Ready
                            )
                    );
                liftTimer.reset();
                liftState = LiftState.ARM_READY;
                    telemetry.update();
                }
                break;
            case ARM_READY:
                if (liftTimer.seconds() > 0.2) {
                    liftHeight = LiftConstants.LiftSpickup;
                  //  OuttakePincher.setPosition(0.5);
                } if (liftTimer.seconds() > 0.5) {
                LeftArm.setPosition(0.7);
                RightArm.setPosition(0.7);
                ClawRotate.setPosition(0.65);
                OuttakePincher.setPosition(0);
                liftTimer.reset();
                liftState = LiftState.DRIVE_PICKUP_SPECIMAN2;
                telemetry.update();
            }
                break;
            case DRIVE_PICKUP_SPECIMAN2:
                Actions.runBlocking(
                        new SequentialAction(
                              TrajectoryActionPickUpSpeciman
                        )
                );
                liftTimer.reset();
                liftState = LiftState.PICKUP_SPECIMAN2;
                telemetry.update();
                break;
            case PICKUP_SPECIMAN2:
                if (liftTimer.seconds() > 0.2) {
                    OuttakePincher.setPosition(0.4);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_EXTEND2;
                    telemetry.update();
                }
                break;
            case LIFT_EXTEND2:
               if (liftTimer.seconds() > 0.7) {
                   liftHeight = LiftConstants.BarAuto2;
                   liftTimer.reset();
                   telemetry.update();
                   liftState = LiftState.DRIVE_ARM_TRANSFER;
                   telemetry.update();
               }
                break;
            case DRIVE_ARM_TRANSFER:
               if (liftTimer.seconds() > 1) {
                   Actions.runBlocking(
                           new SequentialAction(
                                   TrajectoryArmFlipS2
                           )
                   );
                   liftTimer.reset();
                   liftState = LiftState.DRIVE_BAR2;
                   telemetry.update();
               }
                break;
           case DRIVE_BAR2:
               Actions.runBlocking(
                            new SequentialAction(
                                    TrajectoryBar2
                            )
                    );
                    liftTimer.reset();
                    liftState = LiftState.SPECIMAN_DROP2;
               telemetry.update();

                break;
            case SPECIMAN_DROP2:
               // if (liftTimer.seconds() > 0.2) {
               //     ClawRotate.setPosition(0.6);
              //  }
                if (liftTimer.seconds() > 0.5) {
                    liftHeight = LiftConstants.SpecimanDropAuto;
                }
                if (liftTimer.seconds() > 0.8) {
                liftHeight = LiftConstants.SpecimanDropAuto2;
            }
                //if (liftTimer.seconds() > 1.1) {
              //  liftHeight = LiftConstants.SpecimanDropAuto3;
              //  }
                if (liftTimer.seconds() > 2) {
                    OuttakePincher.setPosition(0);
                    liftTimer.reset();
                    liftState = LiftState.BACK_UP;
                    telemetry.update();
                }
                break;
            case BACK_UP:
                if (liftTimer.seconds() > 1) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    TrajectoryBackUp
                            )
                    );
            liftTimer.reset();
            liftState = LiftState.LIFT_RETRACT2;
                    telemetry.update();
                }
            case LIFT_RETRACT2:
                if (liftTimer.seconds() > 1) {
                    LeftArm.setPosition(0.35);
                    RightArm.setPosition(0.35);
                    ClawRotate.setPosition(0.35);
                    telemetry.update();
                }
                if (liftTimer.seconds() > 3) {
                    liftHeight = LiftConstants.liftRetracted;
                    liftTimer.reset();
                   // liftState = LiftState.PARK;
                    telemetry.update();
                }
                break;
        }
          lift.setHeight(liftHeight);
        }
    }

}
