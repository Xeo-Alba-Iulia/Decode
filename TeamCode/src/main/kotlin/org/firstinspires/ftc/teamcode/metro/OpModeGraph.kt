package org.firstinspires.ftc.teamcode.metro

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.DependencyGraph
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.SingleIn
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.sorter.Sorter

@DependencyGraph(OpModeScope::class)
interface OpModeGraph {
    val opMode: OpMode
    val telemetry: Telemetry

    val shooter: Shooter
    val sorter: Sorter

    @Provides
    fun provideHardwareMap(opMode: OpMode): HardwareMap = opMode.hardwareMap

    @Provides
    @SingleIn(OpModeScope::class)
    @Named("classic")
    fun provideClassicTelemetry(opMode: OpMode): Telemetry = opMode.telemetry

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideTelemetry(@Named("classic") telemetry: Telemetry): Telemetry =
        MultipleTelemetry(
            FtcDashboard.getInstance().telemetry,
            telemetry
        )

    @DependencyGraph.Factory
    fun interface Factory {
        fun create(@Provides opMode: OpMode): OpModeGraph
    }
}