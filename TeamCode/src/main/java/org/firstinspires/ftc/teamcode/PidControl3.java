package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidControl3 {
    DcMotorEx leftLift;
    DcMotorEx rightLift;

    private Servo SlideServoLeft, SlideServoRight = null;
    private Servo LeftArm, RightArm = null;
    private Servo ClawWrist = null;
    private Servo ClawElbow = null;
    private Servo IntakeFlip = null;



    double integralSum =0;
    double Kp =0.007; // 0.045
    double Ki =0;
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
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");


        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        SlideServoLeft.setDirection(Servo.Direction.REVERSE);
        RightArm.setDirection(Servo.Direction.REVERSE);
        IntakeFlip.setDirection(Servo.Direction.REVERSE);


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

    public void Idle() {
        SlideServoLeft.setPosition(LiftConstants.HSIdle);
        SlideServoRight.setPosition(LiftConstants.HSIdle);
        LeftArm.setPosition(LiftConstants.ArmIdle);
        RightArm.setPosition(LiftConstants.ArmIdle);
        ClawWrist.setPosition(LiftConstants.WristIdle);
        ClawElbow.setPosition(LiftConstants.ElbowIdle);
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

    public void HSRetract () {
        SlideServoLeft.setPosition(LiftConstants.HSIdle);
        SlideServoRight.setPosition(LiftConstants.HSIdle);
        IntakeFlip.setPosition(LiftConstants.IntakeUp);
    }
    public void ArmTransfer() {
        LeftArm.setPosition(LiftConstants.ArmTransfer);
        RightArm.setPosition(LiftConstants.ArmTransfer);
    }

    public void WallPickup() {
        LeftArm.setPosition(LiftConstants.ArmWall);
        RightArm.setPosition(LiftConstants.ArmWall);
        ClawElbow.setPosition(LiftConstants.ElbowWall);
        ClawWrist.setPosition(LiftConstants.WristIdle);
    }

    public void SpecimanDrop() {
        LeftArm.setPosition(LiftConstants.ArmRung);
        RightArm.setPosition(LiftConstants.ArmRung);
        ClawWrist.setPosition(LiftConstants.WristSpeciman);
        ClawElbow.setPosition(LiftConstants.ElbowRung);
    }



}
