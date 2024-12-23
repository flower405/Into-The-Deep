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

    // private Limelight3A limelight;
    private CRServo Intake = null;


    //  SparkFunOTOS myOtos;
    private Servo SlideServoLeft = null;
    private Servo SlideServoRight = null;
    private Servo IntakeFlip = null;

    private Servo LeftArm = null;
      private Servo RightArm = null;

    private Servo ClawRotate = null;

    private Servo OuttakePincher = null;

    private Servo IntakePincher = null;


    private int liftOffset = 0;

    private int liftHeight = 0;
    private int storeLiftHeight = 0;

    private boolean liftIncrease = false;

    private boolean liftDecrease = false;

    PidControl2 lift = new PidControl2();

    //funny little comment
    // for dumping the cubey things in the bucket
    private enum BucketState {
        IDLE,
        HORIZONTAL_EXTEND,
        INTAKE,
        INTAKE_PINCHER_CLOSE,
        HORIZONTAL_RETRACT,
        SAMPLE_TRANSFER,
        LIFT_EXTEND,
        CLAW_READY,
        VERTICAL_ADJUSMENT,
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
        //  limelight = hardwareMap.get(Limelight3A.class, "limelight");

        Intake = hardwareMap.get(CRServo.class, "Intake");
        //    myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        ClawRotate = hardwareMap.get(Servo.class, "Claw_Rotate");
        IntakePincher = hardwareMap.get(Servo.class, "Intake_Pincher");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");



        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        SlideServoLeft.setDirection(Servo.Direction.REVERSE);
        LeftArm.setDirection(Servo.Direction.REVERSE);
        ClawRotate.setDirection(Servo.Direction.REVERSE);
        IntakePincher.setDirection(Servo.Direction.REVERSE);



        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("status", "Initialized");

        lift.initTele(hardwareMap);

        // limelight set up
        //  telemetry.setMsTransmissionInterval(11);


        //   limelight.pipelineSwitch(0);


        //   limelight.start();


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
        telemetry.addData("ClawRotate", ClawRotate.getPosition());



// see I use comments not for anything useful but I am using it


        switch (bucketState) {
            case IDLE:  // start positions
                LeftArm.setPosition(0.35); //I adjusted the idle position so we can flip intake down without hitting the arm
                RightArm.setPosition(0.35);
                SlideServoLeft.setPosition(0);
                SlideServoRight.setPosition(0);
                ClawRotate.setPosition(0.3);
                IntakePincher.setPosition(0.3);
                OuttakePincher.setPosition(0);
                IntakeFlip.setPosition(0);
                storeLiftHeight = LiftConstants.liftRetracted;
                bucketState = BucketState.HORIZONTAL_EXTEND;
                break;
            case HORIZONTAL_EXTEND: // extending horiztonal lifts and flipping intake out
                if (gamepad1.left_bumper) {
                    LeftArm.setPosition(0.35);
                    RightArm.setPosition(0.35);
                    SlideServoLeft.setPosition(0.2);
                    SlideServoRight.setPosition(0.2);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.left_trigger > 0.9) {
                    LeftArm.setPosition(0.35);
                    RightArm.setPosition(0.35);
                    SlideServoLeft.setPosition(0.5);
                    SlideServoRight.setPosition(0.5);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.right_trigger > 0.9) {
                    LeftArm.setPosition(0.35);
                    RightArm.setPosition(0.35);
                    SlideServoLeft.setPosition(0.75);
                    SlideServoRight.setPosition(0.75);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                } if (gamepad2.dpad_left) {
                    bucketState = BucketState.WALL_PICKUP;
            }
                break;
            case INTAKE: // starting the roller an rejecting
                if (BucketTimer.seconds() > 0.5) {
                    //IntakeFlip.setPosition(0.52);
                    bucketState = BucketState.INTAKE_PINCHER_CLOSE;
                    Intake.setPower(1);
                }
                break;
            case INTAKE_PINCHER_CLOSE:  // close pincher on sample and stop the roller

                if (gamepad1.a) {
                    IntakeFlip.setPosition(0.53);
                }
                if (gamepad1.b) {
                    IntakeFlip.setPosition(0);
                }

                if (gamepad2.b) {
                    Intake.setPower(-1);
                }
                if (gamepad2.y) {
                    Intake.setPower(1);
                }


                if (gamepad2.x) {
                    Intake.setPower(0);
                    IntakePincher.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.HORIZONTAL_RETRACT;
                }
                if (gamepad1.left_bumper) { // so that you can switch between HS slide positions
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
                if (BucketTimer.seconds() > 0.6) {
                    IntakeFlip.setPosition(0); // change to actual position
                    SlideServoLeft.setPosition(0);
                    SlideServoRight.setPosition(0); //I changed the transfer position to better grab sample
                    OuttakePincher.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.SAMPLE_TRANSFER;
                }
                break;
            case SAMPLE_TRANSFER: // arm goes from idle to pick up cube and go back to idle
               if (BucketTimer.seconds()> 0.5) {
                   IntakePincher.setPosition(0.3);
                   ClawRotate.setPosition(0.6);
               }
               if (BucketTimer.seconds() > 1.2) {
                    LeftArm.setPosition(0.4);
                    RightArm.setPosition(0.4);
               }
                if (BucketTimer.seconds() > 1.8) {
                    OuttakePincher.setPosition(0.4);
                }
                if (BucketTimer.seconds() > 2) {
                    LeftArm.setPosition(0.2);
                    RightArm.setPosition(0.2);
                    bucketState = BucketState.LIFT_EXTEND;
                } if (BucketTimer.seconds() > 2.4) {
                ClawRotate.setPosition(0.3);
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
                if (gamepad1.left_bumper) { // so that you can go back to extending the horizontal slides
                    SlideServoLeft.setPosition(0.2);
                    SlideServoRight.setPosition(0.2);
                    IntakeFlip.setPosition(0.6);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.5);
                    SlideServoRight.setPosition(0.5);
                    IntakeFlip.setPosition(0.6);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideServoLeft.setPosition(0.75);
                    SlideServoRight.setPosition(0.75);
                    IntakeFlip.setPosition(0.6);
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE;
                }

                if (gamepad2.dpad_up) { //to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case CLAW_READY:
                if (BucketTimer.seconds() > 0.8) {
                    LeftArm.setPosition(0);
                    RightArm.setPosition(0);
                    ClawRotate.setPosition(0);
                    bucketState = BucketState.SAMPLE_DUMP;
                }
                if (gamepad2.dpad_up) { //to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case SAMPLE_DUMP: // adjusting height
                if (gamepad2.dpad_down) {
                    OuttakePincher.setPosition(0);
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
                if (BucketTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.3);
                    RightArm.setPosition(0.3);
                    ClawRotate.setPosition(0.3);
                    BucketTimer.reset();
                    bucketState = BucketState.LIFT_RETRACT;
                }
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
                if (SpecimanTimer.seconds() > 0) {
                    LeftArm.setPosition(0.7);
                    RightArm.setPosition(0.7);
                    ClawRotate.setPosition(0.65);
                    OuttakePincher.setPosition(0);
                    SpecimanTimer.reset();
                    bucketState = BucketState.CUBE_PICKUP;
                }
                break;
            case CUBE_PICKUP: // pickup cube and swing arm around
                if (gamepad2.a) {
                    OuttakePincher.setPosition(0.4);
                    SpecimanTimer.reset();
                    bucketState = BucketState.FRONT_BACK_TRANSFER;
                }
                break;
            case FRONT_BACK_TRANSFER:
                if (SpecimanTimer.seconds() > 0.2) {
                    liftHeight = LiftConstants.HighRung;
                }
                if (SpecimanTimer.seconds() > 1.5) {
                  //  LeftArm.setPosition(0.2);
                 //   RightArm.setPosition(0.2);
                    ClawRotate.setPosition(0.65);
                    SpecimanTimer.reset();
                    bucketState = BucketState.BAR_HANG;
                }
                break;
            case BAR_HANG:
                if (gamepad2.right_stick_y < -0.6 && !liftIncrease) {
                    liftHeight += 125;
                    liftIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    liftIncrease = false;
                }

                if (gamepad2.right_stick_y > 0.6 && !liftDecrease) {
                liftHeight -= 125;
                liftDecrease = true;
            } else if (gamepad2.right_stick_y < 0.6) {
                    liftDecrease = false;
                }

//                if (gamepad1.dpad_left) {
//                    liftHeight = LiftConstants.SpecimanDrop;
//                    LeftArm.setPosition(0.1);
//                    RightArm.setPosition(0.1);
//                    ClawRotate.setPosition(0.1);
//                }

                //  liftHeight = LiftConstants.SpecimanDrop;
                     // LeftArm.setPosition(0.85);
                    telemetry.addData("We made it to Bar hang case", "");
                   //  RightArm.setPosition(0.85);
                    SpecimanTimer.reset();
                   // bucketState = BucketState.SPECIMAN_DROP;
                if (gamepad2.y) {
                     bucketState = BucketState.SPECIMAN_DROP;
                }

                break;
            case SPECIMAN_DROP: // dropping the cube off
                if (gamepad2.y) {
                    OuttakePincher.setPosition(0);
                    bucketState = BucketState.SLIFT_RETRACT;
                    SpecimanTimer.reset();
                }
                break;
            case SLIFT_RETRACT:
                if (SpecimanTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.55);
                    RightArm.setPosition(0.55);
                    ClawRotate.setPosition(0.3);
                }
                if (SpecimanTimer.seconds() > 0.7) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            case EMERGENCY: //to reset just in case
                RightArm.setPosition(0.3);
                LeftArm.setPosition(0.3);
                ClawRotate.setPosition(0.3);
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