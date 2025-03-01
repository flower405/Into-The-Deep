package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;




public class WaitBucket implements Action {
    private double BeginTs = -1;
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double t;
        if (BeginTs < 0) {
            BeginTs = Actions.now();
            t = 0;
        } else {
            t = Actions.now() - BeginTs;
        }
        if (t > 2) {
            return false;
        } else {
            return true;
        }
    }
}
