package org.firstinspires.ftc.teamcode.opModes.teleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareDevice

@TeleOp(name = "Prism Artboard Setup", group = "Setup")
class PrismArtboardSetup : LinearOpMode() {

    override fun runOpMode() {
        val prismClass = findClass(
            "org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver",
            "com.qualcomm.hardware.gobilda.GoBildaPrismDriver"
        )

        if (prismClass == null) {
            telemetry.addLine("Prism driver class not found")
            telemetry.update()
            waitForStart()
            return
        }

        @Suppress("UNCHECKED_CAST")
        val prism = hardwareMap.get(prismClass as Class<out HardwareDevice>, "fikerled")

        val colorClass = findClass(
            "org.firstinspires.ftc.teamcode.Prism.Color",
            "com.qualcomm.hardware.gobilda.GoBildaPrismDriver\$Color"
        )

        if (colorClass == null) {
            telemetry.addLine("Prism Color class not found")
            telemetry.update()
            waitForStart()
            return
        }

        val red = getStaticField(colorClass, "RED")
        val blue = getStaticField(colorClass, "BLUE")
        val yellow = getStaticField(colorClass, "YELLOW")
        val green = getStaticField(colorClass, "GREEN")
        val transparent = getStaticField(colorClass, "TRANSPARENT")

        val blinkClass = findClass(
            "org.firstinspires.ftc.teamcode.Prism.PrismAnimations\$Blink",
            "com.qualcomm.hardware.gobilda.PrismAnimations\$Blink"
        )

        val layer0 = getLayer0(prismClass)

        callMethod(prism, "setStripLength", 12)

        // helper to fill all leds with a color
        fun fill(color: Any) {
            for (i in 0 until 12) {
                callMethod(prism, "setPixelColor", i, color)
            }
        }

        fun applyBlink(primary: Any, secondary: Any) {
            val blink = createBlink(blinkClass, primary, secondary)
            if (blink != null) {
                val inserted = if (layer0 != null) {
                    callMethod(prism, "insertAnimation", layer0, blink) ||
                        callMethod(prism, "insertAndUpdateAnimation", layer0, blink)
                } else {
                    callMethod(prism, "insertAnimation", blink)
                }
                if (!inserted) {
                    telemetry.addLine("Could not insert blink animation")
                }
            } else {
                telemetry.addLine("Could not construct blink animation")
            }
        }

        waitForStart()
        if (isStopRequested) return

        // artboard 0: solid red
        fill(red)
        callMethod(prism, "saveToArtboard", 0)

        // artboard 1: blinking red
        fill(red)
        applyBlink(red, transparent)
        callMethod(prism, "saveToArtboard", 1)
        callMethod(prism, "clearAllAnimations")

        // artboard 2: solid blue
        fill(blue)
        callMethod(prism, "saveToArtboard", 2)

        // artboard 3: blinking blue
        fill(blue)
        applyBlink(blue, transparent)
        callMethod(prism, "saveToArtboard", 3)
        callMethod(prism, "clearAllAnimations")

        // artboard 4: solid yellow
        fill(yellow)
        callMethod(prism, "saveToArtboard", 4)

        // artboard 5: blinking yellow
        fill(yellow)
        applyBlink(yellow, transparent)
        callMethod(prism, "saveToArtboard", 5)
        callMethod(prism, "clearAllAnimations")

        // artboard 6: solid green
        fill(green)
        callMethod(prism, "saveToArtboard", 6)

        // artboard 7: blinking green
        fill(green)
        applyBlink(green, transparent)
        callMethod(prism, "saveToArtboard", 7)
        callMethod(prism, "clearAllAnimations")

        telemetry.addLine("Artboards programmed successfully")
        telemetry.update()
        sleep(2000)
    }

    private fun findClass(vararg classNames: String): Class<*>? {
        for (name in classNames) {
            try {
                return Class.forName(name)
            } catch (_: Throwable) {
                // try next
            }
        }
        return null
    }

    private fun getStaticField(clazz: Class<*>, fieldName: String): Any {
        return requireNotNull(clazz.getField(fieldName).get(null))
    }

    private fun getLayer0(prismClass: Class<*>): Any? {
        return try {
            val layerClass = prismClass.declaredClasses.firstOrNull { it.simpleName == "LayerHeight" } ?: return null
            layerClass.getField("LAYER_0").get(null)
        } catch (_: Throwable) {
            null
        }
    }

    private fun createBlink(blinkClass: Class<*>?, primary: Any, secondary: Any): Any? {
        if (blinkClass == null) return null

        val colorClass = primary.javaClass

        return try {
            blinkClass.getConstructor(colorClass, colorClass, Int::class.javaPrimitiveType, Int::class.javaPrimitiveType)
                .newInstance(primary, secondary, 500, 250)
        } catch (_: Throwable) {
            try {
                blinkClass.getConstructor(Int::class.javaPrimitiveType, Int::class.javaPrimitiveType)
                    .newInstance(500, 250)
            } catch (_: Throwable) {
                null
            }
        }
    }

    private fun callMethod(target: Any, methodName: String, vararg args: Any?): Boolean {
        val methods = target.javaClass.methods.filter { it.name == methodName }
        for (m in methods) {
            if (m.parameterTypes.size != args.size) continue
            try {
                m.invoke(target, *args)
                return true
            } catch (_: Throwable) {
                // try next overload
            }
        }
        return false
    }
}
