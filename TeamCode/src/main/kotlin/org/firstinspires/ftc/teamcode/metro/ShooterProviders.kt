package org.firstinspires.ftc.teamcode.metro

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import dev.zacsweers.metro.ContributesTo
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import org.firstinspires.ftc.teamcode.shooter.ShooterImpl

@ContributesTo(HardwareScope::class)
interface ShooterProviders {
    val shooter: ShooterImpl
    val limelight: Limelight3A

    @Provides
    @Named("shooterHoodServo")
    fun provideShooterHoodServo(map: HardwareMap): Servo = map.servo["hood"]

    @Provides
    @Named("turret")
    fun provideShooterRotationServos(map: HardwareMap): List<Servo> =
        listOf(
            map.servo["turret right"],
            map.servo["turret left"],
        )

    @Provides
    @Named("shooter")
    fun provideShooterMotor(map: HardwareMap): DcMotorEx =
        map.getCast<DcMotorEx>("shooter").apply { direction = DcMotorSimple.Direction.REVERSE }

    @Provides
    fun provideLimelight(map: HardwareMap): Limelight3A = map.getAll(Limelight3A::class.java).single()
}