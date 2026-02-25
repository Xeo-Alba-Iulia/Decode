package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.*
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.intake.Intake

@ContributesTo(OpModeScope::class)
interface IntakeProviders {
    val intake: Intake

    @Provides
    @Named("intake")
    fun provideIntakeMotor(map: HardwareMap): DcMotorEx = map.getCast("intake")

    @Provides
    @Named("intake")
    fun provideIntakeServo(map: HardwareMap): CRServo = map.getCast("intake servo")

    @Provides
    fun provideColorSensor(map: HardwareMap): ColorRangeSensor = map.getCast("sensor")

    @Provides
    fun provideDistanceSensor(map: HardwareMap): DistanceSensor = map.getCast("sensor2")
}