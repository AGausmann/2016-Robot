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

import org.strongback.control.TalonController;

/**
 * This is the command to move the arm to a specified angle
 * 
 * @author Julian Nieto
 */
public class MoveArmNear extends org.strongback.command.Command {
    private final Arm arm;
    private final double targetAngle;
    private final double tolerance;

    /**
     * Create a command with the desired target angle and default PID gains.
     *
     * @param arm arm inputed
     * @param targetAngle degrees desired
     * @param tolerance the tolerance
     */
    public MoveArmNear(Arm arm, double targetAngle, double tolerance) {
        super(1.5,arm);
        this.arm = arm;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
    }

    @Override
    public void initialize() {
        arm.setControlMode(TalonController.ControlMode.POSITION);
        arm.setTargetAngle(targetAngle);
    }

    @Override
    public boolean execute() {
        if (arm.isOverCurrent()) {
            arm.setControlMode(TalonController.ControlMode.SPEED);
            arm.stop();
            return true;
        }
        return Math.abs(arm.getCurrentAngle() - arm.getTargetAngle()) < this.tolerance;
    }
}
