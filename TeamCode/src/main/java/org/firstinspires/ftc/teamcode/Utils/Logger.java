package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class Logger extends SubsystemBase {
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public Logger(FtcDashboard dashboard_) {
        dashboard = dashboard_;
        packet = new TelemetryPacket();
    }

    public void log(String label, Object data) {
        packet.put(label, data);
        dashboard.sendTelemetryPacket(packet);
        System.out.println(label + ": " + data);
    }

//    @Override
//    public void periodic() {
//        dashboard.sendTelemetryPacket(packet);
//    }
}
