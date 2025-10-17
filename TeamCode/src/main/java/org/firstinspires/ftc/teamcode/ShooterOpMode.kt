package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Shooter")
open class ShooterOpMode : OpMode() {
    lateinit var shooter: Shooter

    open val appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)

    override fun init() {
        shooter = appGraph.shooter
        shooter.hood = 0.5
    }

    override fun loop() {
        if (gamepad1.a) {
            shooter.isRunning = true
        }
        if (gamepad1.b) {
            shooter.isRunning = false
        }
        if (gamepad1.y) shooter.shooterSpeed += 0.001
        if (gamepad1.x) shooter.shooterSpeed -= 0.001
        if (gamepad1.dpad_up) {
            shooter.hood += 0.001
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.001
        }

        this.telemetry.addLine(shooter.toString())
        this.telemetry.update()
    }
}
