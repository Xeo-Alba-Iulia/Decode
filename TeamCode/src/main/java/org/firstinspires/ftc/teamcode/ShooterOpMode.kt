package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

@TeleOp
class ShooterOpMode : OpMode() {
    lateinit var shooter: Shooter

    lateinit var appGraph: OpModeGraph

    override fun init() {
        appGraph = createGraphFactory<OpModeGraph.Factory>().create(this)
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
        if (gamepad1.y) shooter.shooterSpeed += 0.01
        if (gamepad1.x) shooter.shooterSpeed -= 0.01
        if (gamepad1.dpad_up) {
            shooter.hood += 0.0003
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.0003
        }

        this.telemetry.addLine(shooter.toString())
        this.telemetry.update()
    }
}
