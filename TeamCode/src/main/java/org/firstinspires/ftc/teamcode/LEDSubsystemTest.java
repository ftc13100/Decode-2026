package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;

@TeleOp(name = "LED Subsystem Test", group = "Test")
public class LEDSubsystemTest extends LinearOpMode {
    private static final int TOTAL_TESTS = 9;
    private int testIndex = 0;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() throws InterruptedException {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "fikerled");
        LEDSubsystem leds = new LEDSubsystem(prism);

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        applyCurrentTest(leds, testIndex);

        while (opModeIsActive()) {
            boolean bPressed = gamepad1.b;
            boolean xPressed = gamepad1.x;
            boolean yPressed = gamepad1.y;

            if (bPressed && !prevB) {
                testIndex = (testIndex + 1) % TOTAL_TESTS;
                applyCurrentTest(leds, testIndex);
            }

            if (xPressed && !prevX) {
                testIndex = (testIndex - 1 + TOTAL_TESTS) % TOTAL_TESTS;
                applyCurrentTest(leds, testIndex);
            }

            if (yPressed && !prevY) {
                leds.clear();
            }

            prevB = bPressed;
            prevX = xPressed;
            prevY = yPressed;

            telemetry.addData("Test", "%d / %d", testIndex, TOTAL_TESTS - 1);
            telemetry.addData("Call", getTestName(testIndex));
            telemetry.addLine("B: next test");
            telemetry.addLine("X: previous test");
            telemetry.addLine("Y: clear all");
            telemetry.update();

            idle();
        }
    }

    private void applyCurrentTest(LEDSubsystem leds, int index) {
        switch (index) {
            case 0:
                leds.setLights(0);
                break;
            case 1:
                leds.setLights(1);
                break;
            case 2:
                leds.setLights(2);
                break;
            case 3:
                leds.setLights(3);
                break;
            case 4:
                leds.setDecorative();
                break;
            case 5:
                applyTripleColorTest(leds, 1, 1, 1); // PPP
                break;
            case 6:
                applyTripleColorTest(leds, 1, 1, 2); // PPG
                break;
            case 7:
                applyTripleColorTest(leds, 1, 2, 2); // PGG
                break;
            case 8:
                applyTripleColorTest(leds, 2, 2, 2); // GGG
                break;
            default:
                break;
        }
    }

    private void applyTripleColorTest(LEDSubsystem leds, int color0, int color1, int color2) {
        leds.clear();
        leds.setLights(0, color0);
        leds.setLights(1, color1);
        leds.setLights(2, color2);
    }

    private String getTestName(int index) {
        switch (index) {
            case 0:
                return "setLights(0)";
            case 1:
                return "setLights(1)";
            case 2:
                return "setLights(2)";
            case 3:
                return "setLights(3)";
            case 4:
                return "setDecorative()";
            case 5:
                return "PPP";
            case 6:
                return "PPG";
            case 7:
                return "PGG";
            case 8:
                return "GGG";
            default:
                return "unknown";
        }
    }
}

