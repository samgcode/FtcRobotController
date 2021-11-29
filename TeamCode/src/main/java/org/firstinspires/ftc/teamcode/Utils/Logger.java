package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Logger implements Runnable {
    Thread thread;
    FtcDashboard dashboard;
    TelemetryPacket logs;

    public static long logTime = 100;

    public Logger() {
        dashboard = FtcDashboard.getInstance();
        logs = new TelemetryPacket();
    }

    @Override
    public void run() {
        while(!thread.isInterrupted()) {
            try {
                dashboard.sendTelemetryPacket(getLogs());
//                Thread.sleep(logTime);
//                System.out.println("test");
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
