package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import dev.zacsweers.metro.createGraphFactory
import kotlinx.coroutines.cancel
import org.firstinspires.ftc.teamcode.metro.OpModeGraph

abstract class CoroutineOpMode(isAuto: Boolean? = null) : OpMode() {
    val isAuto = isAuto ?: (javaClass.annotations.count { it is Autonomous } >= 1)
    val opModeGraph = createGraphFactory<OpModeGraph.Factory>().create(this, this.isAuto)
    protected val opModeScope = opModeGraph.opModeScope

    /**
     * A [lazy] whose value is automatically computed in the [init] function.
     *
     * # Be careful to call [CoroutineOpMode.init]!!
     *
     * Even though the functions are called in order of declaration they use [lazy],
     * so out of order initialization is perfectly fine.
     */
    protected fun <T> onInit(initializer: () -> T) = lazy(initializer).also { onInitList += it }

    private val onInitList = mutableListOf<Lazy<*>>()

    override fun init() {
        onInitList.forEach { val _ = it.value }
    }

    override fun stop() {
        opModeScope.cancel("OpMode stop called.")
    }

    override fun loop() {}
}