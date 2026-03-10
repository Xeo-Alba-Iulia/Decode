package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.createGraphFactory
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

abstract class CoroutineOpMode(isAuto: Boolean? = null) : OpMode() {
    val isAuto = isAuto ?: (this::class.java.annotations.count { it is Autonomous } >= 1)
    val opModeGraph = createGraphFactory<OpModeGraph.Factory>().create(this, this.isAuto)
    protected val opModeScope = opModeGraph.opModeScope

    override fun loop() {}
}