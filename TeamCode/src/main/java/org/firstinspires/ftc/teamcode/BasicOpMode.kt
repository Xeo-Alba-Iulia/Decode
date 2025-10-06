package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class BasicOpMode : OpMode() {
    override fun init() {
        telemetry.addLine("Hello, FTC!")
        telemetry.update()
    }

    override fun loop() {
        telemetry.addLine("Running...")
        telemetry.update()
    }
}