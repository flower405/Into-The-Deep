package org.firstinspires.ftc.teamcode;


import android.app.usage.NetworkStats;
import android.transition.Slide;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import java.util.Arrays;
import java.util.stream.IntStream;


@TeleOp
public class Driving extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftLift = null;
    private CRServo spinny1 = null;
    private Servo IntakePincher = null;
    private Servo SlideServoLeft = null;
    private Servo SlideServoRight = null;
    private Servo IntakeFlip = null;
   private Servo LeftArm = null;
    private Servo RightArm = null;
    private Servo ClawWrist = null;
    private Servo ClawElbow = null;
    private Servo OuttakePincher = null;
    private int liftOffset = 0;
    private int liftHeight = 0;
    private int storeLiftHeight = 0;
    private boolean liftIncrease = false;
    private boolean liftDecrease = false;
    private DigitalChannel breakBeam = null;
    PidControl2 lift = new PidControl2();

    private enum BucketState {
        IDLE,
        HORIZONTAL_EXTEND,
        INTAKE,
        INTAKE_PINCHER_CLOSE,
        HORIZONTAL_RETRACT,
        SAMPLE_TRANSFER,
        SAMPLE_PINCH,
        LIFT_EXTEND,
        CLAW_READY,
        CLAW_READY_FRONT,

        SAMPLE_DUMP,
        LIFT_RETRACT,
        ARM_RETRACT,
        WALL_PICKUP,
        ARM_FLIP,
        CUBE_PICKUP,
        FRONT_BACK_TRANSFER,
        BAR_HANG,
        SPECIMAN_DROP,
        SLIFT_RETRACT,
        EMERGENCY

    }


    BucketState bucketState = BucketState.IDLE;
    ElapsedTime BucketTimer = new ElapsedTime();

    ElapsedTime SpecimanTimer = new ElapsedTime();
    int heightAdjust = 0;


    @Override
    public void init() {


        //Declare variables for phone to recognise//


        //names on the config


        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");

        spinny1 = hardwareMap.get(CRServo.class, "spinny1");
        IntakePincher = hardwareMap.get(Servo.class, "IntakePincher");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        breakBeam = hardwareMap.get(DigitalChannel.class, "break_beam");





        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        SlideServoLeft.setDirection(Servo.Direction.REVERSE);
        RightArm.setDirection(Servo.Direction.REVERSE);
        IntakeFlip.setDirection(Servo.Direction.REVERSE);





        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("status", "Initialized");

        lift.initTele(hardwareMap);




    }


    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        leftFrontPower = Range.clip(drive + turn + strafe, -1, 1);
        rightFrontPower = Range.clip(drive - turn - strafe, -1, 1);
        leftBackPower = Range.clip(drive + turn - strafe, -1, 1);
        rightBackPower = Range.clip(drive - turn + strafe, -1, 1);
        //Driving Slow Mode
        if (gamepad1.right_bumper) {
            leftFrontPower /= 2;
            leftBackPower /= 2;
            rightFrontPower /= 2;
            rightBackPower /= 2;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Position",leftLift.getCurrentPosition());
        telemetry.addData("liftHeight",liftHeight);
        lift.setHeight(liftHeight +liftOffset);
        telemetry.addData("lift State",bucketState);
        telemetry.addData("ClawRotate", ClawWrist.getPosition());
        telemetry.addData("breakBeam", breakBeam.getState());
        telemetry.addData("ClawElbow", ClawElbow.getPosition());
        telemetry.addData("LeftArm", LeftArm.getPosition());
        telemetry.addData("RightArm", RightArm.getPosition());



        if (!breakBeam.getState()) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }



        switch (bucketState) {
            case IDLE:  // start positions
                LeftArm.setPosition(0.5);
                RightArm.setPosition(0.5);
                ClawElbow.setPosition(0.7);
                ClawWrist.setPosition(0);
                SlideServoLeft.setPosition(0);
                SlideServoRight.setPosition(0);
                OuttakePincher.setPosition(0.1);
                IntakeFlip.setPosition(0.6);
                IntakePincher.setPosition(0); // find this postion
                liftHeight = LiftConstants.liftRetracted;
                bucketState = BucketState.HORIZONTAL_EXTEND;
                break;
            case HORIZONTAL_EXTEND: // extending horiztonal lifts and flipping intake out
                if (gamepad1.left_bumper) {
                    SlideServoLeft.setPosition(0.2);
                    SlideServoRight.setPosition(0.2);
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.5);
                    SlideServoRight.setPosition(0.5);
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.75);
                    SlideServoRight.setPosition(0.75);
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad2.dpad_left) {
                    bucketState = BucketState.WALL_PICKUP;
                }
                break;
            case INTAKE:
                if (BucketTimer.seconds() > 0.2) {
                    spinny1.setPower(-1);
                    bucketState = BucketState.INTAKE_PINCHER_CLOSE;
                }
                break;
            case INTAKE_PINCHER_CLOSE:
                if (gamepad1.a) {
                    IntakeFlip.setPosition(0); // position
                }
                if (gamepad1.b) {
                    IntakeFlip.setPosition(0.5); // positon
                }
               if (gamepad2.b) {
                   spinny1.setPower(-1);
               }
               if (gamepad2.y) {
                   spinny1.setPower(1);
               }
               if (gamepad2.x) {
                   spinny1.setPower(0);
                   IntakePincher.setPosition(0.3); // close position
                   BucketTimer.reset();
                   bucketState = BucketState.HORIZONTAL_RETRACT;
               }
               if (gamepad1.left_bumper) {
                   SlideServoLeft.setPosition(0.2);
                   SlideServoRight.setPosition(0.2);
               }
               if (gamepad1.left_trigger > 0.9) {
                   SlideServoLeft.setPosition(0.5);
                   SlideServoRight.setPosition(0.5);
               }
               if (gamepad1.right_trigger > 0.9) {
                   SlideServoLeft.setPosition(0.75);
                   SlideServoRight.setPosition(0.75);
               }
               break;
               case HORIZONTAL_RETRACT: // retract lift and flip back intake
               if (BucketTimer.seconds() > 0.5) {
                   IntakeFlip.setPosition(0.5); // up position
                   SlideServoLeft.setPosition(0);
                   SlideServoRight.setPosition(0);
                   OuttakePincher.setPosition(0.1);
                   liftHeight = LiftConstants.liftRetracted;
                   BucketTimer.reset();
                   bucketState = BucketState.SAMPLE_TRANSFER;
               }
                break;
            case SAMPLE_TRANSFER: // arm goes from idle to pick up cube and go back to idle
               if (BucketTimer.seconds()> 0.4) {
                   ClawElbow.setPosition(0.65);
                   IntakePincher.setPosition(0);// open claw
               }
               if (BucketTimer.seconds() > 0.6) {
                   LeftArm.setPosition(0.44);
                   RightArm.setPosition(0.44);
               }
               if (BucketTimer.seconds() > 1.1) {
                   OuttakePincher.setPosition(0.6);
                   bucketState = BucketState.LIFT_EXTEND;
               }
                break;
            case LIFT_EXTEND: // extending vertical slides
                if (gamepad2.left_bumper) {
                    bucketState = BucketState.CLAW_READY;
                    liftHeight = LiftConstants.LowBucket;
                    BucketTimer.reset();
                }
                if (gamepad2.right_bumper) {
                    bucketState = BucketState.CLAW_READY;
                    liftHeight = LiftConstants.HighBucket;
                    BucketTimer.reset();

                }
                if (gamepad2.left_trigger > 0.9) {
                    liftHeight = LiftConstants.HighBucket;
                    bucketState = BucketState.CLAW_READY_FRONT;
                    BucketTimer.reset();
                }
                if (gamepad1.left_bumper) { // so that you can go back to extending the horizontal slides
                    SlideServoLeft.setPosition(0.2);
                    SlideServoRight.setPosition(0.2);
                    IntakeFlip.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.5);
                    SlideServoRight.setPosition(0.5);
                    IntakeFlip.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.75);
                    SlideServoRight.setPosition(0.75);
                    IntakeFlip.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }

                if (gamepad2.dpad_up) { //to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case CLAW_READY:
                if (BucketTimer.seconds() > 0.3) {
                    RightArm.setPosition(0.9);
                    LeftArm.setPosition(0.9);
                }
                if (BucketTimer.seconds() > 0.8) {
                    ClawWrist.setPosition(0);
                    ClawElbow.setPosition(0);
                    bucketState = BucketState.SAMPLE_DUMP;
                }
                if (gamepad2.dpad_up) { // to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case CLAW_READY_FRONT:
                if (BucketTimer.seconds() > 0.3) {
                    LeftArm.setPosition(0.6);
                    RightArm.setPosition(0.6);
                }
                if (BucketTimer.seconds() > 0.8) {
                    ClawWrist.setPosition(0);
                    ClawElbow.setPosition(0.7);
                    bucketState = BucketState.SAMPLE_DUMP;
                }
                if (gamepad2.dpad_up) {
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case SAMPLE_DUMP: // adjusting height
                if (gamepad2.dpad_down) {
                    OuttakePincher.setPosition(0.1);
                    BucketTimer.reset();
                    bucketState = BucketState.ARM_RETRACT;
                }
                if (gamepad2.left_bumper) {
                    liftHeight = LiftConstants.LowBucket;
                }
                if (gamepad2.right_bumper) {
                    liftHeight = LiftConstants.HighBucket;
                }
                if (gamepad2.dpad_up) { //to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case ARM_RETRACT:
                if (BucketTimer.seconds() > 0.8) {
                    RightArm.setPosition(0.5);
                    LeftArm.setPosition(0.5);
                    ClawWrist.setPosition(0); // find this position
                    ClawElbow.setPosition(0.7); // find this position
                    BucketTimer.reset();
                    bucketState = BucketState.LIFT_RETRACT;
                }
                break;
            case LIFT_RETRACT:
                if (BucketTimer.seconds() > 0.5) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            case WALL_PICKUP:
                if (gamepad2.dpad_left) {
                    liftHeight = LiftConstants.LiftSpickup;
                    OuttakePincher.setPosition(0.5); //closes claw when going by strings
                    bucketState = BucketState.ARM_FLIP;
                    SpecimanTimer.reset();
                }
                break;
            case ARM_FLIP:
                if (SpecimanTimer.seconds() > 0.4) {
                    RightArm.setPosition(0.1);
                    LeftArm.setPosition(0.1);
                    ClawWrist.setPosition(0); // find this position
                    OuttakePincher.setPosition(0.1);
                    ClawElbow.setPosition(0.65); // find this position
                    SpecimanTimer.reset();
                    bucketState = BucketState.CUBE_PICKUP;
                }
                break;
            case CUBE_PICKUP: // pickup cube and swing arm around
                if (gamepad2.a) {
                    OuttakePincher.setPosition(0.5);
                    SpecimanTimer.reset();
                    bucketState = BucketState.FRONT_BACK_TRANSFER;
                }
                break;
            case FRONT_BACK_TRANSFER:
                if (SpecimanTimer.seconds() > 0.6) {
                    liftHeight = LiftConstants.HighRung;
                }
                if (SpecimanTimer.seconds() > 0.7) {
                   LeftArm.setPosition(0.45);
                   RightArm.setPosition(0.45);
                    ClawWrist.setPosition(0.67); // find this position
                    ClawElbow.setPosition(0.0); // find this position
                    SpecimanTimer.reset();
                    bucketState = BucketState.BAR_HANG;
                }
                break;
            case BAR_HANG:
                if (gamepad2.y) {
                    bucketState = BucketState.SPECIMAN_DROP;
                    BucketTimer.reset();
                }

                break;
            case SPECIMAN_DROP: // dropping the cube off
//                if (gamepad2.y) {
//                    OuttakePincher.setPosition(0.1);
//                    bucketState = BucketState.SLIFT_RETRACT;
//                    SpecimanTimer.reset();
//                }
                if (BucketTimer.seconds() > 0.1) {
                    liftHeight = LiftConstants.SpecimanDrop;
                    ClawElbow.setPosition(0.3);
                }
                if (BucketTimer.seconds() > 0.4) {
                    OuttakePincher.setPosition(0.1);
                    BucketTimer.reset();
                    bucketState = BucketState.SLIFT_RETRACT;
                }
                break;
            case SLIFT_RETRACT:
                if (BucketTimer.seconds() > 0.5) {
                    RightArm.setPosition(0.6);
                    ClawWrist.setPosition(0.6); // find this position
                    ClawElbow.setPosition(0.7); // find this position
                    ClawWrist.setPosition(0);
                }
                if (BucketTimer.seconds() > 1) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            case EMERGENCY: //to reset just in case
                RightArm.setPosition(0.5);
                LeftArm.setPosition(0.5);
                ClawWrist.setPosition(0.5); // find this position
                ClawElbow.setPosition(0.7); // find this position
                ClawWrist.setPosition(0);
                if (BucketTimer.seconds() > 1) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            default:
                bucketState = BucketState.IDLE;
        }
    }

    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}