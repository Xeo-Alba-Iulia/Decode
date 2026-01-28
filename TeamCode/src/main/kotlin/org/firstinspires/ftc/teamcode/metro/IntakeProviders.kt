package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.intake.Intake

@ContributesTo(OpModeScope::class)
interface IntakeProviders {
    val intake: Intake

    @Provides
    @Named("intakeMotor")
    fun provideIntakeMotor(map: HardwareMap): DcMotor = map.dcMotor["intake"]

    @Provides
    fun provideColorSensor(map: HardwareMap): RevColorSensorV3 = map.getCast("sorter sensor")
}