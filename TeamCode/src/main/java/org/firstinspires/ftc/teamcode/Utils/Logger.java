package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/*
utility class to log data to the FTC Lib dashboard
 */
public class Logger implements Runnable {
    Thread thread;
    FtcDashboard dashboard;
    TelemetryPacket logs;

    public Logger() {
        dashboard = FtcDashboard.getInstance();
        logs = new TelemetryPacket();
    }

    @Override
    public void run() {
        while(!thread.isInterrupted()) {
            try {
                dashboard.sendTelemetryPacket(getLogs());
            } catch (Exception e) {
                System.err.println(e);
            }

        }
    }

    public void start() {
        thread = new Thread(this, "logger");
        thread.start();
    }

    public synchronized void log(String lable, Object data) {
        logs.put(lable, data);
    }

    private synchronized TelemetryPacket getLogs() {
        return logs;
    }

    public void stop() {
        thread.interrupt();
    }

}
