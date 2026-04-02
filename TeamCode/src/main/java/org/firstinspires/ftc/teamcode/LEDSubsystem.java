package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDSubsystem {
    private final GoBildaPrismDriver prism;

    public LEDSubsystem(GoBildaPrismDriver prism) {
        this.prism = prism;
    }

    public void setLights(int mode) {
        if (mode == 0) {
            prism.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_0,
                    new PrismAnimations.Solid(Color.WHITE)
            );
        } else if (mode == 1) {
            prism.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_0,
                    new PrismAnimations.Rainbow()
            );
        } else if (mode == 2) {
            prism.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_0,
                    new PrismAnimations.Blink()
            );
        } else if (mode == 3) {
            prism.insertAndUpdateAnimation(
                    GoBildaPrismDriver.LayerHeight.LAYER_0,
                    new PrismAnimations.Pulse()
            );
        }
        prism.updateAllAnimations();
    }

    public void setLights(int index, int colorCode) {
        int start;
        int end;

        if (index == 0) {
            start = 0;
            end = 3;
        } else if (index == 1) {
            start = 4;
            end = 7;
        } else if (index == 2) {
            start = 8;
            end = 11;
        } else {
            return;
        }

        Color color;
        if (colorCode == 0) {
            color = Color.TRANSPARENT;
        } else if (colorCode == 1) {
            color = Color.PURPLE;
        } else if (colorCode == 2) {
            color = Color.GREEN;
        } else {
            return;
        }

        prism.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Solid(color, start, end)
        );
        prism.updateAllAnimations();
    }
}

