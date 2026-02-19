package org.firstinspires.ftc.teamcode.sorter

import dev.zacsweers.metro.ContributesBinding
import org.firstinspires.ftc.teamcode.metro.OpModeScope

@ContributesBinding(OpModeScope::class, replaces = [TransferImpl::class])
class FakeTransfer : Transfer {
    override var isRunning = false
}