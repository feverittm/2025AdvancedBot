// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 * Ensures all timestamps are synchronized, using the CANivore as the source of truth if connected.
 * <p>
 * When using a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling. This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore time synchronization.
 */
public class HighFrequencySamplingThread extends Thread {
    public static final double frequencyHz = switch (Constants.identity) {
        case COMPBOT, SIMBOT -> 250.0;
        case ALPHABOT -> 100.0;
    };

    public static final Lock highFrequencyLock = new ReentrantLock();
    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals

    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();

    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();

    private final List<DoubleSupplier> genericDoubleSignals = new ArrayList<>();
    private final List<Queue<Double>> genericDoubleQueues = new ArrayList<>();

    private final List<BooleanSupplier> genericBooleanSignals = new ArrayList<>();
    private final List<Queue<Boolean>> genericBooleanQueues = new ArrayList<>();

    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static HighFrequencySamplingThread instance = null;

    public static HighFrequencySamplingThread get() {
        if (instance == null) {
            instance = new HighFrequencySamplingThread();
        }
        return instance;
    }

    private HighFrequencySamplingThread() {
        setName("HighFrequencySamplingThread");
        setDaemon(true);
        super.start();
    }

    @Override
    public synchronized void start() {
        // Already started
    }

    /**
     * Registers a Phoenix signal to be read from the thread.
     */
    public Queue<Double> registerPhoenixSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a Spark signal to be read from the thread.
     */
    public Queue<Double> registerSparkSignal(SparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            sparks.add(spark);
            sparkSignals.add(signal);
            sparkQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a generic signal to be read from the thread.
     */
    public Queue<Double> registerGenericSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            genericDoubleSignals.add(signal);
            genericDoubleQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a generic signal to be read from the thread.
     */
    public Queue<Boolean> registerGenericSignal(BooleanSupplier signal) {
        Queue<Boolean> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        highFrequencyLock.lock();
        try {
            genericBooleanSignals.add(signal);
            genericBooleanQueues.add(queue);
        } finally {
            signalsLock.unlock();
            highFrequencyLock.unlock();
        }
        return queue;
    }

    /**
     * Returns a new queue that returns timestamp values for each sample.
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        highFrequencyLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            highFrequencyLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (Constants.CANivore.isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / frequencyHz, phoenixSignals);
                } else {
                    // "waitForAll" does not support blocking on multiple signals with a bus
                    // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                    // behavior is provided by the documentation.
                    Thread.sleep((long) (1000.0 / frequencyHz));
                    if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            highFrequencyLock.lock();
            try {
                // Sample timestamp is current FPGA time minus average CAN latency
                //     Default timestamps from Phoenix are NOT compatible with
                //     FPGA timestamps, this solution is imperfect but close
                double timestamp = Timer.getFPGATimestamp();
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }


                // Add new samples to queues
                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < sparkSignals.size(); i++) {
                    double value = sparkSignals.get(i).getAsDouble();
                    // If we don't give a value, things will get desynced
//                    if (sparks.get(i).getLastError() == REVLibError.kOk) {
                    sparkQueues.get(i).offer(value);
//                    }
                }
                for (int i = 0; i < genericDoubleSignals.size(); i++) {
                    genericDoubleQueues.get(i).offer(genericDoubleSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < genericBooleanSignals.size(); i++) {
                    genericBooleanQueues.get(i).offer(genericBooleanSignals.get(i).getAsBoolean());
                }
                for (Queue<Double> timestampQueue : timestampQueues) {
                    timestampQueue.offer(timestamp);
                }
            } finally {
                highFrequencyLock.unlock();
            }
        }
    }
}
