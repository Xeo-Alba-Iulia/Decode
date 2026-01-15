package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.shooter.Shooter

@TeleOp(group = "Shooter")
open class ShooterOpMode : CoroutineOpMode() {
    lateinit var shooter: Shooter

    private var currentJob: Job? = null
    private var currentFlow: Flow<Shooter.State>? = null

    override fun init() {
        shooter = opModeGraph.shooter
        shooter.hood = 0.5
        telemetry = opModeGraph.telemetry
    }

    override fun loop() {
        if (gamepad1.aWasPressed()) {
            if (currentFlow == null) {
                currentFlow = shooter.shoot()
            }
            currentJob = opModeScope.launch {
                currentFlow!!.collect {
                    telemetry.addData("Shooter State", it)
                    telemetry.update()
                }
            }
        }
        if (gamepad1.bWasPressed()) {
            currentJob?.cancel()
            currentJob = null
            currentFlow = null
        }
        if (gamepad1.y) shooter.velocity += 1.0
        if (gamepad1.x) shooter.velocity -= 1.0
        if (gamepad1.dpad_up) {
            shooter.hood += 0.01
        }
        if (gamepad1.dpad_down) {
            shooter.hood -= 0.01
        }
        when {
            gamepad1.dpad_right -> shooter.angleDegrees += 0.01
            gamepad1.dpad_left -> shooter.angleDegrees -= 0.01
        }
    }
}
