package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.theRobot.subsystems.ImplementationTOF;
import frc.theRobot.subsystems.Spinner;
import frc.theRobot.Constants;

public class runMiniprojCommand extends Command {
    Spinner spiningMotor;
    ImplementationTOF tofSensor;
    double constSpeed;
    double desiredSpeed = 0;
    boolean isDone = false;
    boolean tofActivated = false;
    boolean tofActivated2 = false;
    boolean previousTofActivation = false;
    boolean previousTofActivation2 = false;
    double dataValue;
    double[] dataValues;
    boolean experimentRunning = false;

    public runMiniprojCommand( Spinner spin, ImplementationTOF tof, int cyc) {
        spiningMotor = spin;
        tofSensor = tof;
        desiredSpeed = (tofRangeInches - tof.getRangeInches());
        dataValues = new double[cyc];
        addRequirements(spin, tof);
    }

    @Override
    public void initialize() {
        isDone = false;
        cycles = desiredCycles;
        cyclesRun = 0;
        desiredSpeed = (13 - tof.getRangeInches());
        if(desiredSpeed < 0){
            desiredSpeed = 0;
        }

        spiningMotor.spinMotor(desiredSpeed);
        experimentRunning = false;
        System.out.println("Run experiment command initilized");
    }

    @Override
    public void execute() {
        //System.out.println("Experiment Running: " + experimentRunning);
        //System.out.println("Currentspeed " + spiningMotor.getSpeed());
        //System.out.println("Desired speed" + desiredSpeed);
        if (desiredSpeed < 0) {
            
            
        }
    }

    public double calculateVariance(double[] data) {
        double sum = 0.0;
        double mean = 0.0;
        double variance = 0.0;

        // Calculate the mean
        for (double value : data) {
            sum += value;
        }
        mean = sum / data.length;

        // Calculate the variance
        for (double value : data) {
            variance += Math.pow(value - mean, 2);
        }
        variance /= data.length; // For population variance

        return variance;
    }

    @Override
    public boolean isFinished() {
        return cycles == 0;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            System.out.println("THE COMMAND WAS INTERRUPTED!!");
        }
        // prints out final data
        spiningMotor.motorStop();
        double meanValue = 0;
        int skippedCycles = 0;
        double expectedCycleTime = 1.0/(2*desiredSpeed/60.0); //pole comes by 2 times per RPS
        double cycleTimeTol = 1.25; 
        double allowedCycleTime = cycleTimeTol * expectedCycleTime;
        double[] varianceData = new double[cyclesRun-1];
        System.out.println("Here is the Final Data:");
        for (int i = 1; i < cyclesRun; i++) {
            System.out.println(dataValues[i]);
            double x = dataValues[i] - dataValues[i - 1];
            // System.out.println(x);
            meanValue += x;
            varianceData[i - 1] = x;
            if (x > allowedCycleTime){
                skippedCycles++;
            }
        }
        System.out.println();
        for (int i = 0; i < cyclesRun-2; i++) {
            System.out.println(varianceData[i]);
        }
        meanValue = meanValue/(cyclesRun-1);
        System.out.println("The speed of the experiment was: " + constSpeed);
        System.out.println("Num Cycles run " + cyclesRun);
        System.out.println("The mean time between cycles was: " + meanValue);
        System.out.println("The variance was: " + calculateVariance(varianceData));
        System.out.println("Number of skipped cycles: " + skippedCycles);
        isDone = true;

    }
}
