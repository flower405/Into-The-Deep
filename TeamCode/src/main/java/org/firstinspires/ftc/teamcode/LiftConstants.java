package org.firstinspires.ftc.teamcode;

//Parameters for lift
public class LiftConstants {
    //Lift heights in ticks
    public static int LowBucket = 800;
    public static int HighBucket = 1750;
    public static int LowRung = 600;
    public static int HighRung = 1000;
    public static int liftRetracted =0;

    public static int liftIdle = 140;
    public static int liftHang = 400;

    public static int[] liftFlat = {1550, 1500, 1450, 1400};

    public static int LiftSpickup = 250;

    public static int BarAuto = 1000;
    public static int BarAuto2 = 1670;

    public static int SpecimanDrop = 500;
    public static int SpecimanDropAuto= 450;
    public static int SpecimanDropAuto2= 750;
    public static int SpecimanDropAuto3 = 600;

    public static int SpecimanDrop2Auto1= 1570;
    public static int SpecimanDrop2Auto2= 1470;
    public static int SpecimanDrop2Auto3 = 200;
    public static int SpecimanDropAuto4 = 550;

    // arm positions
    public static double ArmIdle = 0.5;
    public static double ArmTransfer = 0.41;
    public static double ArmBucket = 0.9;
    public static double ArmWall = 0.14;
    public static double ArmRung = 0.5;

    // elbow positions
    public static double ElbowIdle = 0.7;
    public static double ElbowTransfer = 0.65;
    public static double ElbowBucket = 0;
    public static double ElbowWall = 0.65;
    public static double ElbowRung = 0.3;

    // wrist postion
    public static double WristIdle = 0;
    public static double WristSpeciman = 0.67;

    // Horizontal slide postions
    public static double HSIdle = 0;
    public static double HSLow = 0.2;
    public static double HSMedium = 0.5;
    public static double HSHigh = 0.75;

    // Intake flip
    public static double IntakeDown = 0;
    public static double IntakeUp = 0.6;





}
