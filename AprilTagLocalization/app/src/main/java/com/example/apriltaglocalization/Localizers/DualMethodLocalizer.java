package com.example.apriltaglocalization.Localizers;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;


public abstract class DualMethodLocalizer implements Localizer {


    /**
     * The error of the main localizer, computed by correctError
     */
    protected Vector2dDual<Time> vecError;
    protected double headingError;

    @Override
    public final Twist2dDual<Time> update() {

        Twist2dDual<Time> reading = updateMainLocalizer();
        if (vecError != null) {
            return new Twist2dDual<>(
                    reading.line.plus(vecError),
                    new DualNum<Time>(new double[]{
                            reading.angle.value() + headingError,
                            reading.angle.values().get(1),
                    })
            );
        }

        return reading;
    }

    /**
     * Make this method return the raw result of the main localization method.
     * @return - Returns a pose of the raw main method
     */
    public abstract Twist2dDual<Time> updateMainLocalizer();

    /**
     * Corrects the raw estimate given by the main localizer to create an error vector to add whenever we update.
     * IMPORTANT: you must set the variable vecError to your error vector when overriding this method!!!
     * @see DualMethodLocalizer#vecError
     */
    public abstract void correctError();
}
