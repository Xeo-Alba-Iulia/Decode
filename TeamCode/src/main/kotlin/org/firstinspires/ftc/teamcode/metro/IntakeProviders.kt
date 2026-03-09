package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorRangeSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.intake.Intake

@ContributesTo(HardwareScope::class)
interface IntakeProviders {
    val intake: Intake

    @Provides
    @Named("intake")
    fun provideIntakeMotor(map: HardwareMap): DcMotorEx = map.getCast("intake")

    @Provides
    @Named("intake")
    fun provideIntakeServo(map: HardwareMap): CRServo = map.getCast("intake servo")

    @Provides
    fun provideColorSensors(map: HardwareMap): List<ColorRangeSensor> =
        listOf(map.getCast<RevColorSensorV3>("sensor"))
}