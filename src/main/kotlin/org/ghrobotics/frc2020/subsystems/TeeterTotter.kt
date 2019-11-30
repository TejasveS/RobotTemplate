package org.ghrobotics.frc2020.subsystems

import edu.wpi.first.wpilibj.command.PIDSubsystem


 object TeeterTotter : PIDSubsystem("TeeterTotter", 2.0, 0.5, 0.8,0.5, 20.0) {

    override fun initDefaultCommand() {}

    override fun returnPIDInput(): Double {
        return Drivetrain.navx.pitch as Double
    }

    override fun usePIDOutput(output: Double)  {
        println(output)
        Drivetrain.setPercent(output, output)
    }

    init {
        setAbsoluteTolerance(0.05)
    }
}