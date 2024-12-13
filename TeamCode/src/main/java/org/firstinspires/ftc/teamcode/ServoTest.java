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



    private CRServo Intake = null;
    private Servo SlideServoLeft = null;
    private Servo SlideServoRight = null;
    private Servo IntakeFlip = null;

    private Servo LeftArm = null;
    private Servo RightArm = null;

    private Servo ClawRotate = null;

    private Servo OuttakePincher = null;

    private Servo IntakePincher = null;

    private CRServo Reject = null;



    //funny little comment
    // for dumping the cubey things in the bucket







    @Override
    public void init() {




        //Declare variables for phone to recognise//


        //names on the config




        //  limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        SlideServoLeft = hardwareMap.get(Servo.class, "SlideServoLeft");
        SlideServoRight = hardwareMap.get(Servo.class, "SlideServoRight");
        IntakeFlip = hardwareMap.get(Servo.class, "Intake_Flip");
        LeftArm = hardwareMap.get(Servo.class, "Left_Arm");
        RightArm = hardwareMap.get(Servo.class, "Right_Arm");
        ClawRotate = hardwareMap.get(Servo.class, "Claw_Rotate");
        IntakePincher = hardwareMap.get(Servo.class, "Intake_Pincher");
        OuttakePincher = hardwareMap.get(Servo.class, "Outtake_Pincher");
        Reject = hardwareMap.get(CRServo.class, "Reject");



        SlideServoLeft.setDirection(Servo.Direction.REVERSE);
        LeftArm.setDirection(Servo.Direction.REVERSE);
        IntakePincher.setDirection(Servo.Direction.REVERSE);
        ClawRotate.setDirection(Servo.Direction.REVERSE);







        telemetry.addData("status", "Initialized");


        // limelight set up
        //  telemetry.setMsTransmissionInterval(11);


        //   limelight.pipelineSwitch(0);


        //   limelight.start();






    }


    @Override
    public void loop() {


if (gamepad1.left_bumper) {
   LeftArm.setPosition(0);
    RightArm.setPosition(0);
} if (gamepad1.right_bumper) {
            LeftArm.setPosition(0.);
            RightArm.setPosition(0.5);
        } if (gamepad1.right_trigger > 0.9) {
            LeftArm.setPosition(0.4);
         RightArm.setPosition(0.4);
        } if (gamepad1.left_trigger > 0.9) {
            LeftArm.setPosition(0.7);
           RightArm.setPosition(0.7);
        }


//      if (gamepad1.left_bumper) {
//          LeftArm.setPosition(0.1);
//          RightArm.setPosition(0.1);
//ClawRotate.setPosition(0.1);
//      } if (gamepad1.right_bumper) {
//            LeftArm.setPosition(1);
//            RightArm.setPosition(1);
//            ClawRotate.setPosition(0.35);
//        } if (gamepad1.left_trigger > 0.9) {
//                SlideServoLeft.setPosition(0.1);
//                SlideServoRight.setPosition(0.1);
//              LeftArm.setPosition(0.45);
//                RightArm.setPosition(0.45);
//            ClawRotate.setPosition(0.0);
//        } if (gamepad1.a) {
//            LeftArm.setPosition(0.73);
//            RightArm.setPosition(0.73);
//            ClawRotate.setPosition(0.2);
//        }


// picking up speciman = 0
        // arm idle = 0.55
        // pciking up cube from intake = 0.47
        // dropping off bucket = 1
        // dropping off specima = 0.73

// see I use comments not for anything useful but I am using it























    }
    @Override
    public void stop() {


    }

}