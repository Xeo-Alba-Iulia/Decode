package org.firstinspires.ftc.teamcode

import dev.zacsweers.metro.DependencyGraph
import dev.zacsweers.metro.Provides
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.test.TestScope
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.OpModeScope
import org.firstinspires.ftc.teamcode.sorter.SorterImpl

@DependencyGraph(OpModeScope::class)
interface TestOpModeGraph {
    val sorterImpl: SorterImpl

    @Provides
    fun provideTelemetry(): Telemetry = error("Not available in tests")
    @Provides
    fun provideIsAuto(): Boolean = false
    @Provides
    fun provideCoroutineScope(): CoroutineScope = TestScope(SupervisorJob())
}