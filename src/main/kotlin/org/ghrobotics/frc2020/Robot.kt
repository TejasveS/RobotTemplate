/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.ghrobotics.frc2020.commands.TeeterTotterCommand
import org.ghrobotics.frc2020.subsystems.TeeterTotter


/**
 * Main robot class.
 */
object Robot : FalconTimedRobot() {
    // Constructor of the Robot class.
    init {
        // Add the drivetrain to the subsystem handler
        +Drivetrain
    }

    // Runs once when robot boots up
    override fun robotInit() {
        Drivetrain.navx.reset()
        printNavx()
    }

    // Runs once when autonomous period starts
    override fun autonomousInit() {
        Drivetrain.navx.reset()
//        VoltageDriveCommand().withTimeout(3.0).schedule()
        TeeterTotterCommand().withTimeout(30.0).schedule()
    }

    // Runs once when teleop period starts
    override fun teleopInit() {
      //  Drivetrain.navx.reset()
//        TeeterTotterCommand().withTimeout(30.0).schedule()
    }

    // Runs once when robot is disabled
    override fun disabledInit() {}

    // Runs every 20 ms when robot is on
    override fun robotPeriodic() {
        SmartDashboard.putNumber("NavxPitch", Drivetrain.navx.pitch.toDouble())
        SmartDashboard.putNumber("NavxRoll", Drivetrain.navx.roll.toDouble())
        SmartDashboard.putNumber("NavxYaw", Drivetrain.navx.yaw.toDouble())
        printNavx()
        Shuffleboard.update()
    }

    // Runs every 20 ms when autonomous is enabled
    override fun autonomousPeriodic() {}

    // Runs every 20 ms when teleop is enabled
    override fun teleopPeriodic() {}

    // Runs every 20 ms when robot is disabled
    override fun disabledPeriodic() {}

    fun printNavx() {
//        println("angle:"+Drivetrain.navx.angle)
//        println("roll:"+Drivetrain.navx.roll)
//        println("pitch:"+Drivetrain.navx.pitch)
//        println("yaw:"+Drivetrain.navx.yaw)
//        println("altitude:"+Drivetrain.navx.altitude)
    }
}
