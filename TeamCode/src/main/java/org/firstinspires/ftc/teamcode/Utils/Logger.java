package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;

import java.util.TreeMap;

@Config
public class Logger extends SubsystemBase {
    FtcDashboard dashboard;
    TelemetryPacket packet;
    TreeMap<String, Object> stack = new TreeMap<String, Object>();

    public static double logTime = 2;

    public Logger(FtcDashboard dashboard_) {
        dashboard = dashboard_;
        packet = new TelemetryPacket();
    }

    public void log(String label, Object data) {
        stack.put(label, data);
//        System.out.println(label + ": " + data);
    }

    void clear() {
        packet = new TelemetryPacket();
    }

    int counter = 0;

    @Override
    public void periodic() {
        if(counter %logTime == 0) {
            packet.putAll(stack);
            dashboard.sendTelemetryPacket(packet);
        }
        counter++;
    }
}
