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
public class ServoTest extends OpMode {





    private Servo IntakeFlipLeft = null;

    private Servo IntakeFlipRight = null;


    private Servo ClawElbow = null;
    private Servo ClawWrist = null;

    private Servo LeftArm = null;
    private Servo RightArm = null;

    private DcMotor HorizontalSlide = null;

    private Servo OuttakePincher = null;

    private Servo IntakePincher = null;

   private CRServo spinny1 = null;



    //funny little comment
    // for dumping the cubey things in the bucket







    @Override
    public void init() {




        //Declare variables for phone to recognise//


        //names on the config




        //  limelight = hardwareMap.get(Limelight3A.class, "limelight");


        IntakeFlipLeft = hardwareMap.get(Servo.class, "Intake_Flip_Left");
        IntakeFlipRight = hardwareMap.get(Servo.class, "Intake_Flip_Right");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        HorizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontal_slide");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        ClawElbow = hardwareMap.get(Servo.class, "Claw_Elbow");
        ClawWrist = hardwareMap.get(Servo.class, "Claw_Wrist");
        spinny1 = hardwareMap.get(CRServo.class, "spinny1");


        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        IntakePincher = hardwareMap.get(Servo.class, "IntakePincher");





        RightArm.setDirection(Servo.Direction.REVERSE);
      IntakeFlipLeft.setDirection(Servo.Direction.REVERSE);
        HorizontalSlide.setDirection(DcMotor.Direction.REVERSE);










        telemetry.addData("status", "Initialized");

        HorizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // limelight set up
        //  telemetry.setMsTransmissionInterval(11);


        //   limelight.pipelineSwitch(0);


        //   limelight.start();






    }


    @Override
    public void loop() {

telemetry.addData("Position", HorizontalSlide.getCurrentPosition());

if (gamepad2.left_bumper) {
    IntakeFlipLeft.setPosition(0);
    IntakeFlipRight.setPosition(0);
}

if (gamepad2.right_bumper) { // idle / rung
IntakeFlipRight.setPosition(0.55);
IntakeFlipLeft.setPosition(0.55);

}

if (gamepad2.left_trigger > 0.9) {

}

if (gamepad2.right_trigger > 0.9) {

  }

if (gamepad2.a) {

}

if (gamepad2.b) {

}






// see I use comments not for anything useful but I am using it























    }
    @Override
    public void stop() {


    }

}