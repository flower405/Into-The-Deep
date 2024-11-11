package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidControl2 {
    DcMotorEx leftLift;
    DcMotorEx rightLift;

    double integralSum =0;
    double Kp =0.007; // 0.045
    double Ki =0;
    double Kd = 0.000001; // 0.0000038

    //0.000001
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private Servo IntakePincherServo, OuttakePincherServo, IntakeExtendServo, LeftExtendServo, RightExtendServo = null;

    public void initAuto(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakePincherServo = hardwareMap.get(Servo.class, "Intake_Pincher");
        OuttakePincherServo = hardwareMap.get(Servo.class, "Left_outtake");
        IntakeExtendServo = hardwareMap.get(Servo.class, "Intake_extend");
        LeftExtendServo = hardwareMap.get(Servo.class, "Left_extend");
        RightExtendServo = hardwareMap.get(Servo.class, "Right_extend");


       // rightServo.setDirection(Servo.Direction.REVERSE);

    }

    public void initTele(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        leftLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        IntakePincherServo = hardwareMap.get(Servo.class, "Intake_Pincher");
        OuttakePincherServo = hardwareMap.get(Servo.class, "Left_outtake");
        IntakeExtendServo = hardwareMap.get(Servo.class, "Intake_extend");
        LeftExtendServo = hardwareMap.get(Servo.class, "Left_extend");
        RightExtendServo = hardwareMap.get(Servo.class, "Right_extend");


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




    //Rotates the wrist counter-clockwise to the next wrist position



}
