package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class Driving extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor HorizontalSlide = null;
    private DcMotor leftLift = null;
    private CRServo spinny1 = null;

    private Servo IntakeFlip = null;
    private Servo OuttakePincher = null;
    private int liftOffset = 0;
    private int liftHeight = 0;
    private int storeLiftHeight = 0;
    private boolean liftIncrease = false;
    private boolean liftDecrease = false;

    private int SlideOffset = 0;
    private int SlideHeight = 0;
    private int storeSlideHeight = 0;
    private boolean SlideIncrease = false;
    private boolean SlideDecrease = false;
    private DigitalChannel breakBeam = null;
    PidControl lift = new PidControl();

    PidControl2 Slide = new PidControl2();


    // Rune was here
    // I love Rune Katchur

    private enum BucketState {
        IDLE,
        HORIZONTAL_EXTEND,
        INTAKE_DOWN,
        INTAKE,
        INTAKE_PINCHER_CLOSE,
        SAMPLE_YEET,
        SAMPLE_BACK,
        DROP_IDLE,
        HORIZONTAL_RETRACT,
        HORIZONTAL_RETRACT2,
        SAMPLE_TRANSFER,
        SAMPLE_PINCH,
        LIFT_EXTEND,
        CLAW_READY,
        SAMPLE_DUMP,
        ARM_RETRACT,
        WALL_PICKUP,
        ARM_FLIP,
        CUBE_PICKUP,
        FRONT_BACK_TRANSFER,
        BAR_HANG,
        SPECIMEN_DROP,
        SLIFT_RETRACT,
        EMERGENCY
    }

    private enum HangState {
        LIFT_START,
        LIFT_EXTEND,
        BOX_EXTEND,
        BOX_RETRACT,
        LIFT_RETRACT,
        LIFT_RETRACTED
    }

    BucketState bucketState = BucketState.IDLE;
    HangState hangState = HangState.LIFT_START;
    ElapsedTime BucketTimer = new ElapsedTime();

    ElapsedTime SpecimenTimer = new ElapsedTime();
    int heightAdjust = 0;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontal_slide");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");
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
        spinny1.setDirection(DcMotorSimple.Direction.REVERSE);
        HorizontalSlide.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "Initialized");

        Slide.initTele(hardwareMap);
        lift.initTele(hardwareMap);
        lift.OuttakePincherOpen();
        lift.IntakePincherOpen();
        liftHeight = LiftConstants.liftRetracted;
        SlideHeight = LiftConstants.HRetract;
        lift.Idleint();
        lift.IntakeUp();


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

        telemetry.addData("Position", leftLift.getCurrentPosition());
        telemetry.addData("liftHeight", liftHeight);
        lift.setHeight(liftHeight + liftOffset);
        Slide.setHeight(SlideHeight + SlideOffset);
        telemetry.addData("lift State", bucketState);
        telemetry.addData("PositionSlide", Slide.HorizontalSlide.getCurrentPosition());
        telemetry.addData("SlideHeight", SlideHeight);

        telemetry.addData("breakBeam", breakBeam.getState());


        if (!breakBeam.getState()) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }





        switch (bucketState) {
            case IDLE:  // start positions
                lift.Idle();
                OuttakePincher.setPosition(0);
                lift.IntakePincherOpen();
                liftHeight = LiftConstants.liftRetracted;
                bucketState = BucketState.HORIZONTAL_EXTEND;
                SlideHeight = LiftConstants.HRetract;
                break;
            case HORIZONTAL_EXTEND: // extending horizontal lifts and flipping intake out
                if (gamepad1.left_bumper) {
                    SlideHeight = LiftConstants.Hlow;
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hmedium;
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hhigh;
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad2.dpad_left) {
                    bucketState = BucketState.WALL_PICKUP;
                }
                if (gamepad2.right_stick_y < -0.6 && !SlideIncrease && SlideHeight < 1200) {
                    SlideHeight += 100;
                    SlideIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    SlideIncrease = false;
                }

                if (gamepad2.right_stick_y > 0.6 && !SlideDecrease && SlideHeight > 0) {
                    SlideHeight -= 100;
                    SlideDecrease = true;
                } else if (gamepad2.right_stick_y < 0.6) {
                    SlideDecrease = false;
                }

                if (gamepad2.left_stick_y < -0.6 && !liftIncrease) {
                    liftHeight += 100;
                    liftIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    liftIncrease = false;
                }

                if (gamepad2.left_stick_y > 0.6 && !liftDecrease) {
                    liftHeight -= 100;
                    liftDecrease = true;
                } else if (gamepad2.right_stick_y < 0.6) {
                    liftDecrease = false;
                }
                break;
            case INTAKE_DOWN:
                if (BucketTimer.seconds() > 0.4) {
                    lift.IntakeDown();
                }
            case INTAKE:
                if (BucketTimer.seconds() > 0.2) {
                    spinny1.setPower(-1);
                    bucketState = BucketState.INTAKE_PINCHER_CLOSE;
                }
                break;
            case INTAKE_PINCHER_CLOSE:
                if (gamepad1.a) {
                    lift.IntakeDown();
                }
                if (gamepad1.b) {
                    lift.IntakeUp();
                }
                if (gamepad2.b) {
                    spinny1.setPower(1);
                }
                if (gamepad2.y) {
                    spinny1.setPower(-1);
                }
                if (gamepad2.right_trigger > 0.9) {
                    spinny1.setPower(0);
                    lift.IntakePincherClose(); // close position
                    BucketTimer.reset();
                    bucketState = BucketState.HORIZONTAL_RETRACT;
                }
                if (gamepad1.left_bumper) {
                    SlideHeight = LiftConstants.Hlow;
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hmedium;
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hhigh;
                }
                if (gamepad2.right_stick_y < -0.6 && !SlideIncrease && SlideHeight < 1300) {
                    SlideHeight += 100;
                    SlideIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    SlideIncrease = false;
                }

                if (gamepad2.right_stick_y > 0.6 && !SlideDecrease && SlideHeight > 0) {
                    SlideHeight -= 100;
                    SlideDecrease = true;
                } else if (gamepad2.right_stick_y < 0.6) {
                    SlideDecrease = false;
                }
                break;
            case HORIZONTAL_RETRACT: // retract lift and flip back intake
                if (BucketTimer.seconds() > 0.2) {
                    lift.IntakeUp();
                }
                if (BucketTimer.seconds() > 0.5) {
                    SlideHeight = LiftConstants.HRetract;
                    OuttakePincher.setPosition(0);
                    liftHeight = LiftConstants.liftRetracted;
                    BucketTimer.reset();
                    bucketState = BucketState.SAMPLE_TRANSFER;
                }
                break;
            case HORIZONTAL_RETRACT2:
                if (BucketTimer.seconds() > 0.4) {
                    lift.IntakeUp();
                }
                if (BucketTimer.seconds() > 0.7) {
                    SlideHeight = LiftConstants.HRetract;
                    OuttakePincher.setPosition(0);
                    liftHeight = LiftConstants.liftRetracted;
                    BucketTimer.reset();
                    bucketState = BucketState.SAMPLE_YEET;
                }
            case SAMPLE_TRANSFER: // arm goes from idle to pick up cube and go back to idle
                if (BucketTimer.seconds() > 0.4) {
                    lift.ElbowTransfer();
                    lift.IntakePincherOpen(); // open claw
                }
                if (BucketTimer.seconds() > 0.6) {
                    lift.ArmTransfer();
                }
                if (BucketTimer.seconds() > 1) {
                    OuttakePincher.setPosition(0.5);
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
                    bucketState = BucketState.WALL_PICKUP;
                    BucketTimer.reset();
                }
                if (gamepad1.left_bumper) { // so that you can go back to extending the horizontal slides
                    SlideHeight = LiftConstants.Hlow;
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
                }
                if (gamepad1.left_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hmedium;
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
                }
                if (gamepad1.right_trigger > 0.9) {
                    SlideHeight = LiftConstants.Hhigh;
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
                }

                if (gamepad2.right_stick_y < -0.6 && !SlideIncrease && SlideHeight < 1200) {
                    SlideHeight += 100;
                    SlideIncrease = true;
                } else if (gamepad2.right_stick_y > -0.6 ) {
                    SlideIncrease = false;
                }

                if (gamepad2.right_stick_y > 0.6 && !SlideDecrease && SlideHeight > 0) {
                    SlideHeight -= 100;
                    SlideDecrease = true;
                } else if (gamepad2.right_stick_y < 0.6) {
                    SlideDecrease = false;
                }

                if (gamepad2.dpad_up) { //to reset in emergency
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case CLAW_READY:
                if (BucketTimer.seconds() > 0.3) {
                    lift.Bucket();
                    bucketState = BucketState.SAMPLE_DUMP;
                }
                if (gamepad2.dpad_up) { // to reset in emergency
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
                    lift.ArmRetract();
                }
                if (BucketTimer.seconds() > 1.2) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.HORIZONTAL_EXTEND;
                }
                break;
            case SAMPLE_BACK:
                if (BucketTimer.seconds() > 0) {
                    liftHeight = LiftConstants.LiftSpickup;
                }
                if (BucketTimer.seconds() > 0.6) {
                    lift.WallPickup();
                    bucketState = BucketState.SAMPLE_YEET;
                    BucketTimer.reset();
                }
            case SAMPLE_YEET:
                if (gamepad2.dpad_left) {
                    lift.OuttakePincherOpen();
                    bucketState = BucketState.WALL_PICKUP;
                    BucketTimer.reset();
                }
                if (gamepad2.dpad_up) {
                    bucketState = BucketState.DROP_IDLE;
                    BucketTimer.reset();
                }
            case DROP_IDLE:
                if (BucketTimer.seconds() > 0) {
                    lift.OuttakePincherOpen();
                }
                if (BucketTimer.seconds() > 0.3) {
                    lift.Idle();
                }
                if (BucketTimer.seconds() > 0.6) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                    BucketTimer.reset();
                }
            case WALL_PICKUP:
                if (gamepad2.dpad_left) {
                    liftHeight = LiftConstants.SampleYeet;
                    bucketState = BucketState.ARM_FLIP;
                    BucketTimer.reset();
                    SpecimenTimer.reset();

                }
                break;
            case ARM_FLIP:
                if (BucketTimer.seconds() > 0.6) {
                    lift.WallPickup();
                    BucketTimer.reset();

                }
                if (SpecimenTimer.seconds() > 1) {
                    OuttakePincher.setPosition(0);
                    liftHeight = LiftConstants.LiftSpickup;
                    bucketState = BucketState.CUBE_PICKUP;
                }
                break;
            case CUBE_PICKUP: // pickup cube and swing arm around
                if (gamepad2.a) {
                    OuttakePincher.setPosition(0.5);
                    BucketTimer.reset();
                    bucketState = BucketState.FRONT_BACK_TRANSFER;
                }
                if (gamepad2.dpad_up) {
                    BucketTimer.reset();
                    bucketState = BucketState.EMERGENCY;
                }
                break;
            case FRONT_BACK_TRANSFER:
                if (BucketTimer.seconds() > 0.6) {
                    liftHeight = LiftConstants.HighRung;
                }
                if (BucketTimer.seconds() > 0.9) {
                    lift.SpecimanDrop();
                    BucketTimer.reset();
                    bucketState = BucketState.BAR_HANG;
                }
                break;
            case BAR_HANG:
                if (gamepad2.y) {
                    bucketState = BucketState.SPECIMEN_DROP;
                    BucketTimer.reset();
                }

                break;
            case SPECIMEN_DROP: // dropping the cube off
                if (BucketTimer.seconds() > 0.1) {
                    liftHeight = LiftConstants.SpecimenDrop;
                }
                if (BucketTimer.seconds() > 0.4) {
                    OuttakePincher.setPosition(LiftConstants.OuttakePincherOpen);
                    BucketTimer.reset();
                    bucketState = BucketState.SLIFT_RETRACT;
                }

                break;
            case SLIFT_RETRACT:
                if (BucketTimer.seconds() > 0.5) {
                    lift.ArmRetract();
                }
                if (BucketTimer.seconds() > 1) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            case EMERGENCY: //to reset just in case
                lift.ArmRetract();
                if (BucketTimer.seconds() > 1) {
                    liftHeight = LiftConstants.liftRetracted;
                    bucketState = BucketState.IDLE;
                }
                break;
            default:
                bucketState = BucketState.IDLE;
        }


        switch (hangState) {
            case LIFT_START:
                //Driver 2 Dpad starts sequence
                if (gamepad2.touchpad) {
                    hangState = HangState.LIFT_EXTEND;
                    //Extend lift
                    liftHeight = LiftConstants.liftHang2;
                }
                break;
            case LIFT_EXTEND:
                if (gamepad2.dpad_down) {                    hangState = HangState.LIFT_RETRACT;
        }

        break;
        case LIFT_RETRACT:
        // Wait for dpad_up to retract lift and hang
        //Goes straight back to start in case it gets stuck and can't retract all the way
        liftHeight = LiftConstants.liftHang;
//        if (!gamepad2.dpad_up) {
//            hangState = HangState.LIFT_START;
//            //Retract Lift
//        }
        if (gamepad2.right_stick_button) {
            liftHeight = LiftConstants.liftbackup;
        }
        if (gamepad2.left_stick_button) {
            leftLift.setPower(0);
        }
            if (gamepad2.left_stick_y < -0.6 && !liftIncrease) {
                liftHeight += 100;
                liftIncrease = true;
            } else if (gamepad2.right_stick_y > -0.6 ) {
                liftIncrease = false;
            }

            if (gamepad2.left_stick_y > 0.6 && !liftDecrease) {
                liftHeight -= 100;
                liftDecrease = true;
            } else if (gamepad2.right_stick_y < 0.6) {
                liftDecrease = false;
            }
        break;
        default:
        //Should never happen but just in case
        hangState = hangState.LIFT_START;
    }

}
        @Override
        public void stop () {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);


        }
    }


