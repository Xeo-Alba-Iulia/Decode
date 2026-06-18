package org.firstinspires.ftc.teamcode.metro

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.pedropathing.follower.Follower
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.DependencyGraph
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.Provider
import dev.zacsweers.metro.SingleIn
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.SupervisorJob
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.elevator.Elevator
import org.firstinspires.ftc.teamcode.intake.Intake
import org.firstinspires.ftc.teamcode.pedropathing.TuningOpModeKey
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl
import org.firstinspires.ftc.teamcode.sorter.Sorter

@DependencyGraph(OpModeScope::class, additionalScopes = [HardwareScope::class])
interface OpModeGraph {
    val opMode: OpMode
    val telemetry: Telemetry
    val opModeScope: CoroutineScope
    val elevator: Elevator
    val follower: Follower
    val intake: Intake
    val limelight: Limelight3A
    val shooter: ShooterImpl
    val sorter: Sorter
    val tuningOpModesMap: Map<TuningOpModeKey, Provider<OpMode>>

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

    @Provides
    @SingleIn(OpModeScope::class)
    fun provideOpModeScope(): CoroutineScope = CoroutineScope(SupervisorJob())

    @DependencyGraph.Factory
    interface Factory {
        fun create(@Provides opMode: OpMode, @Provides isAuto: Boolean = false): OpModeGraph
    }
}
