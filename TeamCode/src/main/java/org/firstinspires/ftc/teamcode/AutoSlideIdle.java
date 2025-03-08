package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LiftConstants.SpecimenDropAuto;
import static org.firstinspires.ftc.teamcode.LiftConstants.liftRetracted;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;

public class AutoSlideIdle implements Action {
    private double BeginTs = -1;
    PidControl lift = new PidControl();
    public int liftHeight, storeLiftHeight = 0;
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double t;
        if (BeginTs < 0) {
            BeginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - BeginTs;
        }
        if (t > 1) {
            lift.ArmRetract();
        }
        if (t > 2) {
            liftHeight = liftRetracted;
        }
        if (t > 3){
            return false;
        } else {
            return true;
        }
    }
}
