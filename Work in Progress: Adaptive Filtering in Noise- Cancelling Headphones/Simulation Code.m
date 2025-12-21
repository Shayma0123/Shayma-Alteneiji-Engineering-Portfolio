# Simulation Code 
# Adapted from: https://github.com/fhchl/adafilt/blob/master/examples/online_plant_identification.py
import numpy as np
import matplotlib.pyplot as plt
from adafilt import FastBlockLMSFilter, FIRFilter
from adafilt.io import FakeInterface
from adafilt.utils import wgn

# Define impulse responses for different scenarios
def setup_paths(scenario):
    h_pri, h_sec = np.zeros(64), np.zeros(64)
    if scenario == "Scenario 1":
        h_pri[50] = 0.8
        h_sec[10] = 0.5
    elif scenario == "Scenario 2":
        h_pri[30], h_pri[40] = 0.5, -0.3
        h_sec[20] = -0.4
    elif scenario == "Scenario 3":
        h_pri[25] = 0.7
        h_sec[15] = 0.3
    return h_pri, h_sec

# General parameters
length = 64
blocklength = 4
n_buffers = 2000
estimation_phase = 500

# Generate a predefined noise signal
np.random.seed(123)  # Fix seed for reproducibility
predefined_noise_signal = np.random.normal(0, 1, size=n_buffers * blocklength)

# Step sizes to iterate through
step_sizes = np.arange(0.01, 0.11, 0.01)

# Run simulations for each step size
all_results = {}

for step_size in step_sizes:
    scenario_results = {}
    for scenario in ["Scenario 1", "Scenario 2", "Scenario 3"]:
        # Set up paths
        h_pri, h_sec = setup_paths(scenario)

        # Initialize simulation with predefined noise signal
        sim = FakeInterface(
            blocklength, predefined_noise_signal, h_pri=h_pri, h_sec=h_sec, noise=wgn(predefined_noise_signal, 20, "dB")
        )
        filt = FastBlockLMSFilter(length, blocklength, stepsize=step_size, leakage=0.99999, power_averaging=0.9)
        filt.locked = True
        plant_model = FIRFilter(np.zeros(blocklength + length))
        adaptive_plant_model = FastBlockLMSFilter(length, blocklength, stepsize=step_size, leakage=0.99999)

        # Logging
        elog = []

        # Simulation loop
        y = np.zeros(blocklength)
        for i in range(n_buffers):
            if i < estimation_phase:
                v = np.random.normal(0, 1, blocklength)  # Estimation noise
            else:
                v = np.random.normal(0, 0.01, blocklength)  # Operation noise
                adaptive_plant_model.stepsize = step_size

            x, e, u, d = sim.playrec(-y + v)
            plant_model.w[blocklength:] = adaptive_plant_model.w
            fx = plant_model(x)

            if i >= estimation_phase:
                filt.adapt(fx, e)
                y = filt.filt(x)

            elog.append(np.abs(e))

        # Store results for each scenario
        scenario_results[scenario] = np.array(elog)

    # Store results for each step size
    all_results[step_size] = scenario_results

# Generate and save graphs for each step size
for step_size, results in all_results.items():
    plt.figure(figsize=(10, 6))
    for scenario, elog in results.items():
        # Calculate moving RMS error
        rms_error = np.sqrt(np.convolve(elog.flatten() ** 2, np.ones(50) / 50, mode="valid"))
        plt.plot(rms_error, label=f"{scenario} - RMS Error")

    plt.title(f"Oscillation and Convergence Speed (Step Size = {step_size:.2f})")
    plt.xlabel("Iterations")
    plt.ylabel("RMS Error")
    plt.legend()
    plt.grid(True)
    plt.show()
