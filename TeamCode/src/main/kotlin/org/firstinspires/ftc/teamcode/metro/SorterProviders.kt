package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.sorter.Transfer

@ContributesTo(OpModeScope::class)
interface SorterProviders {
    val transfer: Transfer

    companion object {
        @Provides
        @Named("sorterServo")
        fun provideServo(map: HardwareMap): Servo = map.servo["sorter"]

        @Provides
        @Named("transferServo")
        fun provideTransferServo(map: HardwareMap): CRServo = map.crservo["transfer"]
    }
}