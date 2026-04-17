package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "Spindexer Ball Count Lights", group = "Test")
public class SpindexerBallCountLights extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the Prism LED Driver
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "fikerled");
        prism.setStripLength(36);

        // Initialize the 3 Color Sensors from the Spindexer
        NormalizedColorSensor cs0 = hardwareMap.get(NormalizedColorSensor.class, "cs0");
        NormalizedColorSensor cs1 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        NormalizedColorSensor cs2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");

        // Set the gain to match the Spindexer subsystem configuration
        cs0.setGain(12.0f);
        cs1.setGain(12.0f);
        cs2.setGain(12.0f);

        // Initialize Motors for Manual Control
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor spindexer = hardwareMap.get(DcMotor.class, "spindexer");

        telemetry.addLine("Spindexer Ball Count Lights Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("  RB: Intake In");
        telemetry.addLine("  LB: Intake Out");
        telemetry.addLine("  Right Trigger: Spindexer CW");
        telemetry.addLine("  Left Trigger: Spindexer CCW");
        telemetry.update();

        waitForStart();

        int lastCount = -1;
        
        // Initial clear to ensure a clean state
        prism.clearAllAnimations();

        while (opModeIsActive()) {
            // Manual Controls
            if (gamepad1.right_bumper) intake.setPower(-0.9);
            else if (gamepad1.left_bumper) intake.setPower(0.7);
            else intake.setPower(0);

            if (gamepad1.right_trigger > 0.1) spindexer.setPower(gamepad1.right_trigger);
            else if (gamepad1.left_trigger > 0.1) spindexer.setPower(-gamepad1.left_trigger);
            else spindexer.setPower(0);

            // Ball Detection Logic (Strict: Green or Purple only)
            int count = 0;
            if (isBallPresent(cs0)) count++;
            if (isBallPresent(cs1)) count++;
            if (isBallPresent(cs2)) count++;

            // Update LEDs if the count changes
            if (count != lastCount) {
                // Instead of relying on clearAllAnimations() which might be unreliable in rapid succession,
                // we explicitly update every layer. Setting a layer to Color(0,0,0) effectively turns it off.
                
                // Layer 0: LEDs 0-5 (1st Ball)
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_0,
                        new PrismAnimations.Solid(count >= 1 ? new Color(255, 0, 0) : new Color(0, 0, 0), 0, 5)
                );

                // Layer 1: LEDs 6-15 (2nd Ball)
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_1,
                        new PrismAnimations.Solid(count >= 2 ? new Color(255, 0, 0) : new Color(0, 0, 0), 6, 15)
                );

                // Layer 2: LEDs 18-23 (3rd Ball)
                prism.insertAndUpdateAnimation(
                        GoBildaPrismDriver.LayerHeight.LAYER_2,
                        new PrismAnimations.Solid(count >= 3 ? new Color(255, 0, 0) : new Color(0, 0, 0), 18, 23)
                );

                lastCount = count;
            }

            telemetry.addData("Balls Detected", count);
            telemetry.addData("CS0", getDetectedColorName(cs0));
            telemetry.addData("CS1", getDetectedColorName(cs1));
            telemetry.addData("CS2", getDetectedColorName(cs2));
            telemetry.update();
            
            idle();
        }
    }

    /**
     * Checks if a valid ball (Green or Purple) is present.
     */
    private boolean isBallPresent(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        
        // Proximity check (alpha < 0.15 is empty)
        if (colors.alpha < 0.15) return false;

        // Color dominance check (from Spindexer.kt)
        boolean isGreen = (colors.green > colors.red && colors.green > colors.blue);
        boolean isPurple = (colors.blue > colors.red && colors.blue > colors.green);
        
        return isGreen || isPurple;
    }

    private String getDetectedColorName(NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors.alpha < 0.15) return "EMPTY";
        if (colors.green > colors.red && colors.green > colors.blue) return "GREEN";
        if (colors.blue > colors.red && colors.blue > colors.green) return "PURPLE";
        return "OTHER";
    }
}
