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
    private DcMotor leftLift = null;
    private CRServo spinny1 = null;

    private Servo IntakeFlip = null;
    private Servo OuttakePincher = null;
    private int liftOffset = 0;
    private int liftHeight = 0;
    private int storeLiftHeight = 0;
    private boolean liftIncrease = false;
    private boolean liftDecrease = false;
    private DigitalChannel breakBeam = null;
    PidControl lift = new PidControl();

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
    BucketState bucketState = BucketState.IDLE;
    ElapsedTime BucketTimer = new ElapsedTime();
    int heightAdjust = 0;
    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");

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

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("status", "Initialized");

        lift.initTele(hardwareMap);
        lift.OuttakePincherOpen();
       lift.IntakePincherOpen();
        liftHeight = LiftConstants.liftRetracted;




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

        telemetry.addData("breakBeam", breakBeam.getState());




        if (!breakBeam.getState()) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }



        switch (bucketState) {
            case IDLE:  // start positions
                lift.Idle();
                OuttakePincher.setPosition(0.1);
               lift.IntakePincherOpen();
                liftHeight = LiftConstants.liftRetracted;
                bucketState = BucketState.HORIZONTAL_EXTEND;
                break;
            case HORIZONTAL_EXTEND: // extending horizontal lifts and flipping intake out
                if (gamepad1.left_bumper) {
                    lift.HSLow();
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.left_trigger > 0.9) {
                    lift.HSMedium();
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad1.right_trigger > 0.9) {
                    lift.HSHigh();
                    bucketState = BucketState.INTAKE;
                    BucketTimer.reset();
                }
                if (gamepad2.dpad_left) {
                    bucketState = BucketState.WALL_PICKUP;
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
                   lift.HSLow();
                }
                if (gamepad1.left_trigger > 0.9) {
                    lift.HSMedium();
                }
                if (gamepad1.right_trigger > 0.9) {
                    lift.HSHigh();
                }
                break;
            case HORIZONTAL_RETRACT: // retract lift and flip back intake
                if (BucketTimer.seconds() > 0.4) {
                    lift.IntakeUp();
                }
                if (BucketTimer.seconds() > 0.7) {
                   lift.HSRetract();
                   OuttakePincher.setPosition(0.1);
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
                    lift.HSRetract();
                    OuttakePincher.setPosition(0.1);
                    liftHeight = LiftConstants.liftRetracted;
                    BucketTimer.reset();
                    bucketState = BucketState.SAMPLE_YEET;
                }
            case SAMPLE_TRANSFER: // arm goes from idle to pick up cube and go back to idle
                if (BucketTimer.seconds()> 0.4) {
                    lift.ElbowTransfer();
                   lift.IntakePincherOpen(); // open claw
                }
                if (BucketTimer.seconds() > 0.6) {
                    lift.ArmTransfer();
                }
                if (BucketTimer.seconds() > 0.8) {
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
                    bucketState = BucketState.SAMPLE_YEET;
                    BucketTimer.reset();
                }
                if (gamepad1.left_bumper) { // so that you can go back to extending the horizontal slides
                    lift.HSLow();
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
                }
                if (gamepad1.left_trigger > 0.9) {
                    lift.HSMedium();
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
                }
                if (gamepad1.right_trigger > 0.9) {
                    lift.HSHigh();
                    BucketTimer.reset();
                    bucketState = BucketState.INTAKE_DOWN;
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
                    liftHeight = LiftConstants.LiftSpickup;
                    bucketState = BucketState.ARM_FLIP;
                    BucketTimer.reset();
                }
                break;
            case ARM_FLIP:
                if (BucketTimer.seconds() > 0.6) {
                    lift.WallPickup();
                    OuttakePincher.setPosition(0.1);
                    BucketTimer.reset();
                    bucketState = BucketState.CUBE_PICKUP;
                }
                break;
            case CUBE_PICKUP: // pickup cube and swing arm around
                if (gamepad2.a) {
                    OuttakePincher.setPosition(0.5);
                   BucketTimer.reset();
                    bucketState = BucketState.FRONT_BACK_TRANSFER;
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
    }
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }

}