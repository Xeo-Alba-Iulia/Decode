package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.zacsweers.metro.BindingContainer
import dev.zacsweers.metro.Named
import dev.zacsweers.metro.Provides
import dev.zacsweers.metro.createDynamicGraphFactory
import org.firstinspires.ftc.teamcode.ShooterOpMode
import org.firstinspires.ftc.teamcode.metro.OpModeGraph
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.shooter.ShooterNoDirectionImpl

@TeleOp(group = "Shooter")
class ShooterMotorOpMode : ShooterOpMode() {
    override val appGraph =
        createDynamicGraphFactory<OpModeGraph.Factory>(FakeShooterBindings).create(this)

    @BindingContainer
    object FakeShooterBindings {
        @Provides
        fun provideShooter(shooter: ShooterNoDirectionImpl): Shooter = shooter

        @Provides
        @Named("ShootingMotor")
        fun provideShootingMotor(map: HardwareMap): DcMotor = error("Motorul de shooter nu are port")
    }
}

