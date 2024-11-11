package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;

    //PID constants
    double integralSum =0;
    double Kp =0.04;
    double Ki =0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public void init(HardwareMap hardwareMap){

        leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);


        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void setHeight(double height){
        double ticks_per_mm = (384.5 / 112);
        double value = height * ticks_per_mm;
        int target = (int)value;
//        leftLift.setTargetPosition(target);
        rightLift.setTargetPosition(target);
        leftLift.setVelocity(2785);
        rightLift.setVelocity(2785);
    }

    // PID Controller takes reference and current state ctrlaltftc.com
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double getHeight(){return leftLift.getCurrentPosition() / (384.5/112);}
//    public double getRightHeight(){return rightLift.getCurrentPosition() / (384.5/112);}



}