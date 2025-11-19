package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.Binds
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.ShooterNoMotorImpl

@ContributesTo(OpModeScope::class)
interface ShooterProviders {
    @Binds
    val ShooterNoMotorImpl.bind: Shooter

    @Provides
    @Named("shooterHoodServo")
    fun provideShooterHoodServo(map: HardwareMap): Servo = map.servo["hood"]
}