package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "Brightness Test", group = "Test")
public class PrismPhotoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "fikerled");
        
        // Ensure the strip length is set
        prism.setStripLength(36);

        telemetry.addLine("Brightness Test Initialized");
        telemetry.update();

        waitForStart();

        // Clear any existing animations first
        prism.clearAllAnimations();

        // 0-5: White at 100% brightness
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Solid(new Color(255, 255, 255), 0, 5)
        );

        // 6-17: Red at 75% brightness (~191)
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_1,
                new PrismAnimations.Solid(new Color(191, 0, 0), 6, 17)
        );

        // 18-23: White at 50% brightness (~127)
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_2,
                new PrismAnimations.Solid(new Color(127, 127, 127), 18, 23)
        );

        // 24-35: Red at 5% brightness (~13)
        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_3,
                new PrismAnimations.Solid(new Color(13, 0, 0), 24, 35)
        );

        while (opModeIsActive()) {
            telemetry.addLine("Brightness Test Running");
            telemetry.addData("0-5", "White 100%");
            telemetry.addData("6-17", "Red 75%");
            telemetry.addData("18-23", "White 50%");
            telemetry.addData("24-35", "Red 5%");
            telemetry.update();
            idle();
        }
    }
}
