/*
 * Copyright (c) 2016 FRC Team 4931
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.frc4931.robot.arm;

import org.frc4931.robot.math.PIDGains;
import org.strongback.command.Requirable;
import org.strongback.components.AngleSensor;
import org.strongback.components.Switch;
import org.strongback.control.TalonController;

/**
 * The Arm is a subsystem of the robot that can be used to raise and lower to specific angles.
 * It is used to traverse tall defenses and open gates.
 */
public class Arm implements Requirable {
    public static final double MOTOR_SPEED = 0.8;

    private final TalonController controller;
    private final AngleSensor angleSensor;
    private final Switch homeSwitch;
    private final double maxCurrent;

    /**
     * Constructs a new Arm subsystem given a Talon SRX motor controller.
     * @param controller The Talon SRX that is responsible for controlling the arm.
     * @param maxCurrent The maximum current that the motor should use.
     */
    public Arm(TalonController controller, double maxCurrent) {
        this.controller = controller;
        this.maxCurrent = maxCurrent;
        angleSensor = controller.getSelectedSensor();
        //FIXME Strongback assigns reverse limit switch to forward
        homeSwitch = controller.getReverseLimitSwitch();
    }

    /**
     * Get this controller's current control mode.
     *
     * @return the control mode; never null
     */
    public TalonController.ControlMode getControlMode() {
        return controller.getControlMode();
    }

    /**
     * Set the control mode for this controller.
     * TODO See if this can be handled in the background
     *
     * @param mode the control mode; may not be null
     * @return this object so that methods can be chained; never null
     */
    public TalonController setControlMode(TalonController.ControlMode mode) {
        return controller.setControlMode(mode);
    }

    public void zero() {
        angleSensor.zero();
    }

    /**
     * Gets the current angle relative to its zero angle.
     * Positive angles are relatively lower, whereas negative angles are relatively higher.
     * @return The angle of the arm.
     */
    public double getCurrentAngle() {
        return angleSensor.getAngle();
    }

    public double getTargetAngle() {
        return controller.getTarget();
    }

    /**
     * Set the target angle of the arm's PID loop.
     * @param targetAngle The new target angle that the arm should move to.
     */
    public void setTargetAngle(double targetAngle) {
        controller.withTarget(targetAngle);
    }

    public boolean isAtTarget() {
        return controller.isWithinTolerance();
    }

    public void setGains(PIDGains gains) {
        setGains(gains.getP(), gains.getI(), gains.getD());
    }

    public void setGains(double p, double i, double d) {
        controller.withGains(p, i, d);
    }

    public void raise() {
        controller.setSpeed(MOTOR_SPEED);
    }

    public void lower() {
        controller.setSpeed(-MOTOR_SPEED);
    }

    public void stop() {
        controller.stop();
    }

    public boolean isAtHome() {
        return homeSwitch.isTriggered();
    }

    public void setSoftLimitsEnabled(boolean enabled) {
        controller.enableForwardSoftLimit(enabled);
        controller.enableReverseSoftLimit(enabled);
    }

    public double getCurrent() {
        return controller.getCurrentSensor().getCurrent();
    }

    public boolean isOverCurrent() {
        return getCurrent() > maxCurrent;
    }
}
