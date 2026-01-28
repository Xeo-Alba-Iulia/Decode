package org.firstinspires.ftc.teamcode.sorter

import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.DependencyGraph
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.metro.CoroutineProviders
import org.firstinspires.ftc.teamcode.metro.ElevatorProviders
import org.firstinspires.ftc.teamcode.metro.IntakeProviders
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@DependencyGraph(
    OpModeScope::class,
    excludes = [CoroutineProviders::class, ElevatorProviders::class, IntakeProviders::class]
)
interface TestOpModeGraph {
    val sorterImpl: SorterImpl
    val sorter: Sorter

    @Provides
    fun provideHardwareMap(): HardwareMap = error("Not available in tests")

    @Provides
    fun provideTelemetry(): Telemetry = error("Not available in tests")

    @Provides
    fun provideIsAutonomous(): Boolean = false
}