package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.LLSub;

@TeleOp(name = "Turret Recenter Test", group = "Test")
public class TurretRecenterTest extends LinearOpMode {

    private LLSub llSub;
    private double centerVoltage = 0;
    private boolean isRecentering = false; // State flag

    @Override
    public void runOpMode() {
        llSub = new LLSub(this);

        telemetry.addLine("Initialized. Reading Center Voltage...");
        telemetry.update();

        // Assume where it starts is 'center' for this test
        centerVoltage = llSub.getTurretVoltage();

        telemetry.addData("Captured Center Voltage", "%.3f V", centerVoltage);
        telemetry.addLine("Press START.");
        telemetry.addLine("Controls: Stick X to move, 'A' to recenter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Trigger recentering on button press
            if (gamepad1.square) {
                isRecentering = true;
            }

            // Calculate manual power
            double manualPower = -gamepad1.left_stick_x * 0.4;

            // If joystick is moved significantly, cancel recentering
            if (Math.abs(manualPower) > 0.05) {
                isRecentering = false;
            }

            if (isRecentering) {
                // Active recentering
                llSub.recenterTurret(centerVoltage);
                telemetry.addData("Mode", "RECENTERING (Touch Stick to Cancel)");

                // Optional: Check if done to auto-disable flag, though leaving it true
                // just keeps the P-loop active holding position which is usually good.
                if (Math.abs(centerVoltage - llSub.getTurretVoltage()) < 0.05) {
                     telemetry.addData("Status", "CENTERED");
                }
            } else {
                // Manual override / Idle
                if (Math.abs(manualPower) > 0.05) {
                     llSub.setTurretPower(manualPower);
                     telemetry.addData("Mode", "MANUAL");
                } else {
                     llSub.setTurretPower(0);
                     telemetry.addData("Mode", "IDLE");
                }
            }

            telemetry.addData("Current Voltage", "%.3f V", llSub.getTurretVoltage());
            telemetry.addData("Target Voltage", "%.3f V", centerVoltage);
            telemetry.addData("Error", "%.3f V", centerVoltage - llSub.getTurretVoltage());
            telemetry.update();
        }

        llSub.stop();
    }
}
