package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
public class robot extends Robot {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final robotHw robotHw;
    public Drive drive;
    public static enum OpModeType {TELEOP, AUTO}
    public static OpModeType opModeType = OpModeType.TELEOP;

    public robot(Telemetry telemetry, HardwareMap hardwareMap, OpModeType opModeType) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.hardwareMap = hardwareMap;
        this.opModeType = opModeType;

        robotHw = new robotHw(
                telemetry,
                hardwareMap.getAll(LynxModule.class),
                hardwareMap.voltageSensor
        );

        init();
    }

    public void init(){
        // add subsystem constructors
        drive = new Drive(
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(DcMotorEx.class, "leftFront"),
                hardwareMap.get(GoBildaPinpointDriver.class, "pinPoint"),
                telemetry,
                this
                );
    }

    public void read(){
        robotHw.read();
    }

    public void loop(){
        drive.loop();
        robotHw.loop();

    }

    public void write(){
        drive.write();
        robotHw.write(); //always last
        telemetry.update();
    }
}
