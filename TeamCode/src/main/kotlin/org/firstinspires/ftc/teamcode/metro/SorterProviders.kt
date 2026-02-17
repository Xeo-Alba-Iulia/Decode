package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.sorter.Sorter

@ContributesTo(OpModeScope::class)
interface SorterProviders {
    val sorter: Sorter

    companion object {
        @Provides
        @Named("sorterServo")
        fun provideServo(map: HardwareMap): Servo = map.servo["sorter"]

        @Provides
        @Named("transfer")
        fun provideTransferMotor(map: HardwareMap): DcMotor = map.dcMotor["transfer"]
    }
}