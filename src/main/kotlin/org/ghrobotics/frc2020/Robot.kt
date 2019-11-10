/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import org.ghrobotics.frc2020.commands.TeeterTotterCommand

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
        printNavx()
        TeeterTotterCommand().withTimeout(10.0).schedule()
    }

    // Runs once when teleop period starts
    override fun teleopInit() {
        Drivetrain.navx.reset()
        printNavx()
    }

    // Runs once when robot is disabled
    override fun disabledInit() {}

    // Runs every 20 ms when robot is on
    override fun robotPeriodic() {
        Shuffleboard.update()
    }

    // Runs every 20 ms when autonomous is enabled
    override fun autonomousPeriodic() {
        printNavx()
    }

    // Runs every 20 ms when teleop is enabled
    override fun teleopPeriodic() {
        printNavx()
    }

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
