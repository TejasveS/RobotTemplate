package org.ghrobotics.frc2020.commands

import org.ghrobotics.frc2020.subsystems.Drivetrain2
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.acceleration
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.meters

class VoltageDriveCommand: FalconCommand(Drivetrain2) {

    override fun initialize() {
        super.initialize()
    }

    override fun execute () {
        Drivetrain2.setPercent(left = 0.5, right = 0.5)
        Drivetrain2.setOutput(
                1.meters.velocity, 1.meters.velocity,
                0.meters.acceleration, SIUnit(0.0)
        )
    }

    override fun end(interrupted: Boolean) {
        Drivetrain2.setNeutral()
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }
}
