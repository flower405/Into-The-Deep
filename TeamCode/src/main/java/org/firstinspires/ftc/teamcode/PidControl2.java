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
public class PidControl2 {
    DcMotorEx HorizontalSlide;





    double integralSum = 0;
    double Kp = 0.007; // 0.045
    double Ki = 0;
    double Kd = 0.000001; // 0.0000038

    //0.000001
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public void initAuto(HardwareMap hardwareMap) {
        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontal_slide");


        HorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        HorizontalSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        HorizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);


        HorizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // rightServo.setDirection(Servo.Direction.REVERSE);

    }

    public void initTele(HardwareMap hardwareMap) {
        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontal_slide");


        HorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        HorizontalSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);



        HorizontalSlide.setDirection(DcMotorSimple.Direction.FORWARD);


        HorizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





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
        double power = PIDControl(height, HorizontalSlide.getCurrentPosition());
        HorizontalSlide.setPower(power);

    }

    public void disableMotors() {
        HorizontalSlide.setPower(0);

    }
















}

