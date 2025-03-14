package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
public class PidControl {
    DcMotorEx leftLift;
    DcMotorEx rightLift;

    private Servo SlideServoLeft = null;
    private Servo SlideServoRight = null;
    private Servo IntakeFlipLeft = null;
    private Servo IntakeFlipRight = null;
    private Servo LeftArm = null;
    private Servo RightArm = null;
    private Servo ClawWrist = null;
    private Servo ClawElbow = null;
    private Servo OuttakePincher = null;
    private Servo IntakePincher = null;


    double integralSum = 0;
    double Kp = 0.007; // 0.045
    double Ki = 0;
    double Kd = 0.000001; // 0.0000038

    //0.000001
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public void initAuto(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // rightServo.setDirection(Servo.Direction.REVERSE);

    }

    public void initTele(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlipLeft = hardwareMap.get(Servo.class, "Intake_Flip_Left");
        IntakeFlipRight = hardwareMap.get(Servo.class, "Intake_Flip_Right");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        IntakePincher = hardwareMap.get(Servo.class, "IntakePincher");


        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
      SlideServoLeft.setDirection(Servo.Direction.REVERSE);
      RightArm.setDirection(Servo.Direction.REVERSE);
      IntakeFlipLeft.setDirection(Servo.Direction.REVERSE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public void setHeight(double height) {
        double power = PIDControl(height, leftLift.getCurrentPosition());
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void disableMotors() {
        leftLift.setPower(0);
        rightLift.setPower(0);
    }

    public void SpecimanDrop() {
        LeftArm.setPosition(LiftConstants.ArmRung);
        RightArm.setPosition(LiftConstants.ArmRung);
        ClawElbow.setPosition(LiftConstants.ElbowRung);
        ClawWrist.setPosition(LiftConstants.WristSpeciman);
    }

    public void SpecimenDropAuto() {
        LeftArm.setPosition(LiftConstants.ArmRung);
        RightArm.setPosition(LiftConstants.ArmRung);
        ClawElbow.setPosition(LiftConstants.ElbowRungAuto);
        ClawWrist.setPosition(LiftConstants.WristSpeciman);
    }



    public void Idle() {
        LeftArm.setPosition(LiftConstants.ArmIdle);
        RightArm.setPosition(LiftConstants.ArmIdle);
        ClawElbow.setPosition(LiftConstants.ElbowIdle);
        ClawWrist.setPosition(LiftConstants.WristIdle);
    }

    public void HSLow() {
        SlideServoLeft.setPosition(LiftConstants.HSLow);
        SlideServoRight.setPosition(LiftConstants.HSLow);
    }

    public void HSMedium() {
        SlideServoLeft.setPosition(LiftConstants.HSMedium);
        SlideServoRight.setPosition(LiftConstants.HSMedium);
    }

    public void HSHigh() {
        SlideServoLeft.setPosition(LiftConstants.HSHigh);
        SlideServoRight.setPosition(LiftConstants.HSHigh);
    }

    public void IntakeUp() {
        IntakeFlipLeft.setPosition(LiftConstants.IntakeUp);
        IntakeFlipRight.setPosition(LiftConstants.IntakeUp);
    }

    public void IntakeDown() {
        IntakeFlipLeft.setPosition(LiftConstants.IntakeDown);
        IntakeFlipRight.setPosition(LiftConstants.IntakeDown);
    }

    public void HSRetract() {
        SlideServoLeft.setPosition(LiftConstants.HSIdle);
        SlideServoRight.setPosition(LiftConstants.HSIdle);
    }

    public void ElbowTransfer() {
        ClawElbow.setPosition(LiftConstants.ElbowTransfer);
    }

    public void ArmTransfer() {
        LeftArm.setPosition(LiftConstants.ArmTransfer);
        RightArm.setPosition(LiftConstants.ArmTransfer);
    }

    public void ArmRetract() {
        LeftArm.setPosition(LiftConstants.ArmIdle);
        RightArm.setPosition(LiftConstants.ArmIdle);
        ClawElbow.setPosition(LiftConstants.ElbowIdle);
        ClawWrist.setPosition(LiftConstants.WristIdle);
    }

    public void WallPickup() {
        LeftArm.setPosition(LiftConstants.ArmWall);
        RightArm.setPosition(LiftConstants.ArmWall);
        ClawElbow.setPosition(LiftConstants.ElbowWall);
        ClawWrist.setPosition(LiftConstants.WristIdle);
    }

    public void Bucket() {
        LeftArm.setPosition(LiftConstants.ArmBucket);
        RightArm.setPosition(LiftConstants.ArmBucket);
        ClawElbow.setPosition(LiftConstants.ElbowBucket);
        ClawWrist.setPosition(LiftConstants.WristIdle);
    }
    public void OuttakePincherClose() {
        OuttakePincher.setPosition(LiftConstants.OuttakePincherClose);
    }
    public void OuttakePincherOpen() {
       OuttakePincher.setPosition(LiftConstants.OuttakePincherOpen);
    }

    public void IntakePincherOpen() {
        IntakePincher.setPosition(LiftConstants.InttakePincherOpen);
    }

    public void IntakePincherClose() {
        IntakePincher.setPosition(LiftConstants.InttakePincherClose);
    }

    public void IntakePincherOpenAuto() {
        IntakePincher.setPosition(LiftConstants.IntakePincherOpenAuto);
    }

    //Rotates the wrist counter-clockwise to the next wrist position



}
