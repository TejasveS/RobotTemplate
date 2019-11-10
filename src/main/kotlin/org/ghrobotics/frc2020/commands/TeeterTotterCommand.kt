/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2020.commands

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2020.Controls
import org.ghrobotics.frc2020.subsystems.Drivetrain
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volt
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kA

/**
 * Command to drive the robot using the Xbox controller in teleop.
 */
class TeeterTotterCommand : FalconCommand(Drivetrain) {
    var startClimb = false
    val setPoint = 0.0
    val kP = .65
    val kP1 = .45
    val kP2 = .25
    val threshold = 3.0
    val threshold1 = 6.0
    val threshold2 = 9.0

    override fun execute() {
        println(Drivetrain.navx.roll)
        if(Drivetrain.navx.roll > threshold){
            println("startClimb True")
            startClimb = true
        }
        val error = Drivetrain.navx.roll - setPoint
        if(startClimb) {
            var voltage = kP * error
            if(Drivetrain.navx.roll > threshold2 ) {
                println("peak")
                voltage = kP2 * error
            } else if (Drivetrain.navx.roll > threshold1){
                println("midway")
                voltage = kP1 * error
            } else {
                println("lowclimb")
                voltage = kP * error
            }
            println("voltage:$voltage")
            Drivetrain.leftMotor.setVoltage(voltage.volts)
            Drivetrain.rightMotor.setVoltage(voltage.volts)
        } else {
            println("valley")
            Drivetrain.leftMotor.setVoltage(3.volts)
            Drivetrain.rightMotor.setVoltage(3.volts)
        }
//    Drivetrain.setPercent(voltage, voltage)
//        val velocity = 0.2 * error
//        Drivetrain.leftMotor.setVelocity(velocity.feet.velocity)
//        Drivetrain.rightMotor.setVelocity(velocity.feet.velocity)
//    Drivetrain.leftMotor.setVoltage(voltage..voltage)
//    Drivetrain.rightMotor.setVoltage(voltage.feet.voltage)
    }
}
//abstract class FalconWestCoastDrive Train
//abstract val leftMotor falconmotor <Meter>
// abstract val leftMotor falconmotor <Meter>
// periodicIO
// overrde fun setneutral
//periodicIO.desireOutput = output nothing
//fun set output
//linear velocity linear acceleration
//
//val pitch error = drivetrain.get pitch
// arcadedrive
//linearpercent
//set percent
// periodicIO.desiredoutput = output.percent(left, right)
//periodIO.leftfeed forward = .volts
//periodicIO.rightfeedforward = .volts
//
//
//
//


