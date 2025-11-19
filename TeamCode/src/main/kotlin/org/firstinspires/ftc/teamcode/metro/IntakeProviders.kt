package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides

@ContributesTo(OpModeScope::class)
interface IntakeProviders {
    @Provides
    @Named("intakeMotor")
    fun provideIntakeMotor(map: HardwareMap): DcMotor = map.dcMotor["intake"]
}