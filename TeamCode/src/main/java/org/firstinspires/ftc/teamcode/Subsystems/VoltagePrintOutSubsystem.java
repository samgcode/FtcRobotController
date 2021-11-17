package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Logger;

public class VoltagePrintOutSubsystem extends SubsystemBase {
    Logger logger;
    ModernRoboticsUsbDcMotorController hub;

    public VoltagePrintOutSubsystem(Logger logger_, HardwareMap hardwareMap_) {
        HardwareMap hardwareMap = hardwareMap_;
        logger = logger_;
        hub = hardwareMap.get(ModernRoboticsUsbDcMotorController.class, "Control Hub");
    }

    @Override
    public void periodic() {
         logger.log("voltage", hub.getVoltage());
    }
}
