package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDSubsystem {
    private final GoBildaPrismDriver prism;
    private static final Color BLACK = new Color(0, 0, 0);
    private static final int STRIP_1_START = 0;
    private static final int STRIP_1_LENGTH = 12;
    private static final int STRIP_2_START = 12;
    private static final int STRIP_2_LENGTH = 6;
    private static final int STRIP_3_START = 18;
    private static final int STRIP_3_LENGTH = 6;
    private static final int STRIP_4_START = 24;
    private static final int STRIP_4_LENGTH = 12;
    private static final int[] LARGE_OFFSETS = {4, 6, 8};
    private static final int[] SMALL_OFFSETS = {1, 3, 5};
    private static final int[][] INDEX_RANGES = {
            {0, 3},
            {4, 7},
            {8, 11}
    };

    public LEDSubsystem(GoBildaPrismDriver prism) {
        this.prism = prism;
        this.prism.setStripLength(36);
    }

    public void setLights(int mode) {
        prism.clearAllAnimations();

        if (mode == 1) {
            setWhiteCountInEachStrip(1);
        } else if (mode == 2) {
            setWhiteCountInEachStrip(2);
        } else if (mode == 3) {
            setWhiteCountInEachStrip(3);
        }

        prism.updateAllAnimations();
    }

    public void setDecorative() {
        prism.clearAllAnimations();
        prism.insertAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                new PrismAnimations.Rainbow()
        );
        prism.insertAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_1,
                new PrismAnimations.Sparkle()
        );
        prism.updateAllAnimations();
    }

    public void clear() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

    public void setLights(int index, int colorCode) {
        if (index < 0 || index >= INDEX_RANGES.length) {
            return;
        }

        int start = INDEX_RANGES[index][0];
        int end = INDEX_RANGES[index][1];

        Color color;
        if (colorCode == 0) {
            color = BLACK;
        } else if (colorCode == 1) {
            color = Color.PURPLE;
        } else if (colorCode == 2) {
            color = Color.GREEN;
        } else {
            return;
        }

        // Each index uses two dedicated layers so all three index blocks can be shown together.
        int baseLayer = index * 2;
        if (baseLayer + 1 > 9) {
            return;
        }

        prism.insertAnimation(
                getLayer(baseLayer),
                new PrismAnimations.Solid(color, Math.min(start + 1, end), Math.min(start + 1, end))
        );
        prism.insertAnimation(
                getLayer(baseLayer + 1),
                new PrismAnimations.Solid(color, Math.min(start + 3, end), Math.min(start + 3, end))
        );
        prism.updateAllAnimations();
    }

    private void setWhiteCountInEachStrip(int count) {
        int boundedCount = Math.max(0, Math.min(count, 3));
        if (boundedCount == 0) {
            return;
        }

        int layer = 0;
        for (int i = 0; i < boundedCount; i++) {
            layer = applyPixelIfLayerAvailable(layer, STRIP_1_START + LARGE_OFFSETS[i]);
            layer = applyPixelIfLayerAvailable(layer, STRIP_2_START + SMALL_OFFSETS[i]);
            layer = applyPixelIfLayerAvailable(layer, STRIP_3_START + SMALL_OFFSETS[i]);
            layer = applyPixelIfLayerAvailable(layer, STRIP_4_START + LARGE_OFFSETS[i]);
        }
    }

    private int applyPixelIfLayerAvailable(int layerIndex, int pixelIndex) {
        GoBildaPrismDriver.LayerHeight layer = getLayer(layerIndex);
        if (layer == GoBildaPrismDriver.LayerHeight.DISABLED) {
            return layerIndex;
        }

        prism.insertAnimation(
                layer,
                new PrismAnimations.Solid(Color.WHITE, pixelIndex, pixelIndex)
        );
        return layerIndex + 1;
    }

    private GoBildaPrismDriver.LayerHeight getLayer(int layerIndex) {
        switch (layerIndex) {
            case 0:
                return GoBildaPrismDriver.LayerHeight.LAYER_0;
            case 1:
                return GoBildaPrismDriver.LayerHeight.LAYER_1;
            case 2:
                return GoBildaPrismDriver.LayerHeight.LAYER_2;
            case 3:
                return GoBildaPrismDriver.LayerHeight.LAYER_3;
            case 4:
                return GoBildaPrismDriver.LayerHeight.LAYER_4;
            case 5:
                return GoBildaPrismDriver.LayerHeight.LAYER_5;
            case 6:
                return GoBildaPrismDriver.LayerHeight.LAYER_6;
            case 7:
                return GoBildaPrismDriver.LayerHeight.LAYER_7;
            case 8:
                return GoBildaPrismDriver.LayerHeight.LAYER_8;
            case 9:
                return GoBildaPrismDriver.LayerHeight.LAYER_9;
            default:
                return GoBildaPrismDriver.LayerHeight.DISABLED;
        }
    }
}

