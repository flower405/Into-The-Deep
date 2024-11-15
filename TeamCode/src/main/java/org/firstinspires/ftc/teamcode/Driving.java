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

   private CRServo Reject = null;

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

  }

//  private enum SpecimanState {
//      IDLE,
//      WALL_PICKUP,
//      CUBE_PICKUP,
//
//
//
//  }



   // SpecimanState specimanState = SpecimanState.IDLE;
    BucketState bucketState = BucketState.IDLE;
    ElapsedTime BucketTimer = new ElapsedTime();



    @Override
    public void init() {




        //Declare variables for phone to recognise//


        //names on the config


        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        //  limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        //    myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        ClawRotate = hardwareMap.get(Servo.class, "Claw_Rotate");
        IntakePincher = hardwareMap.get(Servo.class, "Intake_Pincher");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        Reject = hardwareMap.get(CRServo.class, "Reject");




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




// see I use comments not for anything useful but I am using it






        switch (bucketState) {
            case IDLE:  // start positions
        LeftArm.setPosition(0);
        SlideServoLeft.setPosition(0);
        SlideServoRight.setPosition(0);
        RightArm.setPosition(0);
        ClawRotate.setPosition(0.3);
        IntakePincher.setPosition(0.72);
        OuttakePincher.setPosition(0);
        lift.setHeight(LiftConstants.liftRetracted);
        bucketState = BucketState.HORIZONTAL_EXTEND;
        break;
        case HORIZONTAL_EXTEND: // extending horiztonal lifts and flipping intake out
    if (gamepad1.left_bumper) {
      SlideServoLeft.setPosition(0.2);
      SlideServoRight.setPosition(0.2);
     IntakeFlip.setPosition(0.6);
      BucketTimer.reset();
      bucketState = BucketState.INTAKE;
    } if (gamepad1.left_trigger > 0.9) {
        SlideServoLeft.setPosition(0.5);
        SlideServoRight.setPosition(0.5);
        IntakeFlip.setPosition(0.6);
        BucketTimer.reset();
        bucketState = BucketState.INTAKE;
    } if (gamepad1.right_trigger > 0.9) {
        SlideServoLeft.setPosition(0.75);
        SlideServoRight.setPosition(0.75);
        IntakeFlip.setPosition(0.6);
        BucketTimer.reset();
        bucketState = BucketState.INTAKE;
    }
    break;
    case INTAKE:
                Intake.setPower(1);
        bucketState = BucketState.INTAKE_PINCHER_CLOSE;
                if (gamepad2.b) {
                    Reject.setPower(-1);
                    bucketState = BucketState.INTAKE_PINCHER_CLOSE;

                }
                break;
    case INTAKE_PINCHER_CLOSE:  // close pincher on sample
                if (gamepad2.x) {
                     Intake.setPower(0);
                    Reject.setPower(0);
                    IntakePincher.setPosition(0.6);
                    BucketTimer.reset();
                     bucketState = BucketState.HORIZONTAL_RETRACT;
                    }
        if (gamepad1.left_bumper) {
            SlideServoLeft.setPosition(0.2);
            SlideServoRight.setPosition(0.2);
            bucketState = BucketState.INTAKE;
        } if (gamepad1.right_bumper) {
        SlideServoLeft.setPosition(0.5);
        SlideServoRight.setPosition(0.5);
        bucketState = BucketState.INTAKE;
    } if (gamepad1.right_trigger > 0.9) {
        SlideServoLeft.setPosition(0.75);
        SlideServoRight.setPosition(0.75);
        bucketState = BucketState.INTAKE;
    }
     break;
     case HORIZONTAL_RETRACT: // retract lift and flip back intake
         if (BucketTimer.seconds() > 0.5) {
             IntakeFlip.setPosition(0); // change to actual position
             SlideServoLeft.setPosition(0);
             SlideServoRight.setPosition(0);
     }
         if (BucketTimer.seconds() > 1.5) { // open up the claw
             IntakePincher.setPosition(0.72);
           //  bucketState = BucketState.SAMPLE_TRANSFER;
             BucketTimer.reset();
         }
         break;
            case SAMPLE_TRANSFER:
                if (BucketTimer.seconds() > 0.3) {
                    LeftArm.setPosition(0.5);
                    RightArm.setPosition(0.5);
                } if (BucketTimer.seconds() > 0.6) {
                    OuttakePincher.setPosition(0.3);
            } if (BucketTimer.seconds() > 1.2) {
                    LeftArm.setPosition(0.3);
                    RightArm.setPosition(0.3);
                    ClawRotate.setPosition(0.1);
                    bucketState = BucketState.LIFT_EXTEND;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad1.left_bumper) {
                 lift.setHeight(LiftConstants.LowBucket);
                    bucketState = BucketState.LIFT_EXTEND;
                } if (gamepad1.right_bumper) {
                    lift.setHeight(LiftConstants.HighBucket);
                     bucketState = BucketState.LIFT_EXTEND;
            }  if (gamepad2.left_bumper) {
                SlideServoLeft.setPosition(0.2);
                SlideServoRight.setPosition(0.2);
                IntakeFlip.setPosition(0.55);
                BucketTimer.reset();
                bucketState = BucketState.INTAKE;
            } if (gamepad2.right_bumper) {
                SlideServoLeft.setPosition(0.5);
                SlideServoRight.setPosition(0.5);
                IntakeFlip.setPosition(0.55);
                BucketTimer.reset();
                bucketState = BucketState.INTAKE;
            } if (gamepad2.right_trigger > 0.9) {
                SlideServoLeft.setPosition(0.75);
                SlideServoRight.setPosition(0.75);
                IntakeFlip.setPosition(0.55);
                BucketTimer.reset();
                bucketState = BucketState.INTAKE;
            }
            break;
            case CLAW_READY:
                if (leftLift.getCurrentPosition() > 600) {
                 LeftArm.setPosition(0.7);
                 RightArm.setPosition(0.7);
                 ClawRotate.setPosition(0.9);
                 bucketState = BucketState.VERTICAL_ADJUSMENT;
                }
                break;
                case SAMPLE_DUMP: // adjusting height
                if (gamepad1.dpad_down) {
                    OuttakePincher.setPosition(0);
                    BucketTimer.reset();
                    bucketState = BucketState.ARM_RETRACT;
                } if (gamepad1.left_bumper) {
                    lift.setHeight(LiftConstants.LowBucket);
                    bucketState = BucketState.ARM_RETRACT;
                } if (gamepad1.right_bumper) {
                    lift.setHeight(LiftConstants.HighBucket);
                    bucketState = BucketState.ARM_RETRACT;
                }
                break;
            case ARM_RETRACT:
                if (BucketTimer.seconds() > 0.5) {
                    LeftArm.setPosition(0.3);
                    RightArm.setPosition(0.3);
                    ClawRotate.setPosition(0.1);
                    BucketTimer.reset();
                    bucketState = BucketState.LIFT_RETRACT;
                }
                case LIFT_RETRACT:
                if (BucketTimer.seconds() > 0.7) {
                    lift.setHeight(LiftConstants.liftRetracted);
                    bucketState = BucketState.IDLE;
                }
                break;
                default:
                //Should never happen but just in case
                bucketState = BucketState.IDLE;
        }


//        switch (specimanState)  {
//            case IDLE:
//                LeftArm.setPosition(0);
//                SlideServoLeft.setPosition(LiftConstants.HSretracted);
//                SlideServoRight.setPosition(0.2);
//                RightArm.setPosition(0);
//                ClawRotate.setPosition(0.3);
//                IntakePincher.setPosition(0.72);
//                OuttakePincher.setPosition(0);
//                lift.setHeight(LiftConstants.liftRetracted);
//                break;
//            case WALL_PICKUP:
//                if (gamepad1.dpad_left) {
//                    LeftArm.setPosition(0);
//                    RightArm.setPosition(0);
//                    ClawRotate.setPosition(0);
//                }
//                break;
//            case CUBE_PICKUP:
//                if (gamepad1.a) {
//                    LeftArm.setPosition(0);
//                    RightArm.setPosition(0);
//                    ClawRotate.setPosition(0);
//                }
//        }




















    }
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}