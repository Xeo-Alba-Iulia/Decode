package org.firstinspires.ftc.teamcode.pedropathing

import com.pedropathing.telemetry.SelectScope
import com.pedropathing.telemetry.Selector
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.Provider

abstract class SelectableOpMode<T: SelectableOpMode<T>>(
    private val name: String,
    private val block: SelectScope<Provider<OpMode>>.(opMode: T) -> Unit
) : OpMode() {

    companion object {
        val MESSAGE = """
            Use the d-pad to move the cursor.
            Press right bumper to select.
            Press left bumper to go back
        """.trimIndent()
    }

    private lateinit var selectedOpMode: OpMode
    private lateinit var selector: Selector<Provider<OpMode>>

    override fun init() {
        @Suppress("UNCHECKED_CAST")
        selector = Selector.create(name, { block(it, this as T) }, MESSAGE)
        selector.onSelect {
            selectedOpMode = it()
            selectedOpMode.gamepad1 = gamepad1
            selectedOpMode.gamepad2 = gamepad2
            selectedOpMode.telemetry = telemetry
            selectedOpMode.hardwareMap = hardwareMap
            selectedOpMode.init()
        }
    }

    override fun init_loop() {
        if (::selectedOpMode.isInitialized) {
            selectedOpMode.init_loop()
            return
        }

        selector.run {
            when {
                gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed() -> decrementSelected()
                gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed() -> incrementSelected()
                gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed() -> select()
                gamepad1.leftBumperWasPressed() || gamepad2.rightBumperWasPressed() -> goBack()
            }

            lines.forEach { telemetry.addLine(it) }
        }
    }

    override fun start() {
        require(::selectedOpMode.isInitialized) { "No OpMode selected!" }
        selectedOpMode.start()
    }

    override fun loop() = selectedOpMode.loop()

    override fun stop() {
        if (::selectedOpMode.isInitialized) selectedOpMode.stop()
    }
}