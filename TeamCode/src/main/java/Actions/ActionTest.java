package Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.ActionHardware;


public class ActionTest implements Action {


    private ActionHardware pivot;


    public ActionTest(ActionHardware pivot) {
        this.pivot = pivot;
    }


    @Override
    public boolean run(TelemetryPacket p) {
        pivot.setWallPosition();
        return true;
    }
}

