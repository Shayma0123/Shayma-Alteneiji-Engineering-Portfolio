# Secondary Path Effects Modeling Code
# Adapted from https://github.com/fhchl/adafilt/blob/master/examples/online_plant_identification.py

import numpy as np
import sounddevice as sd
from adafilt import FastBlockLMSFilter

# Parameters for real-time processing
print(sd.query_devices())  # Query available sound devices

# Set input and output device indices (microphones)
input_device_a = 6  # Input device (reference microphones)
input_device_b = 6  # Input device (error microphone)

# Get information about the selected input device
device = sd.query_devices(input_device_a)
print(device)

# Get the sample rate from the input device (sampling rate for audio processing)
samplerate = device["default_samplerate"]
print(f"Samplerate: {samplerate}")
blocksize = 2048  # Block size for real-time processing
filter_length = blocksize * 4  # Length of the adaptive filter (number of taps)
latency = "low"  # Low latency for real-time processing
channels = 2  # Two channels: reference and error microphones

# Create an adaptive filter for estimating the secondary path (h_sec)
filt = FastBlockLMSFilter(length=filter_length, blocklength=blocksize)

# This will hold the estimated secondary path model (h_sec)
h_sec_estimate = np.zeros(filter_length)

# Convergence check parameters
threshold = 1e-6  # Small change to determine if weights have converged
stable_iterations = 50  # Number of iterations to check for convergence
convergence_counter = 0  # Counter for iterations where weights change is below threshold

# Track previous weights for comparison
previous_weights = np.zeros(filter_length)

# Callback function for real-time audio processing
def callback(indata, outdata, frames, time, status):
    global h_sec_estimate, filt, previous_weights, convergence_counter

    if status:
        print("Callback status:", status)

    # Reference signal comes from the reference microphone (index 0)
    x = indata[:, 0]  # Reference microphone signal

    # Error signal comes from the error microphone (index 1)
    d = indata[:, 1]  # Error microphone signal

    # Use the adaptive filter to estimate the secondary path
    # The filter output is used to estimate the secondary path
    yhat = filt.filt(x)  # Apply filter to the reference signal

    # Compute the error signal (difference between desired and predicted)
    e = d - yhat  # The error between the desired signal and the predicted output

    # Adapt the filter weights to minimize the error signal
    filt.adapt(x, e)

    # Check the change in filter weights to detect convergence
    weight_change = np.linalg.norm(filt.w - previous_weights)  # Euclidean distance between current and previous weights

    if weight_change < threshold:
        convergence_counter += 1
    else:
        convergence_counter = 0  # Reset counter if the change is too large

    # Store the current weights for the next iteration
    previous_weights = filt.w.copy()

    # If the weights have been stable for a sufficient number of iterations, print the coefficients
    if convergence_counter >= stable_iterations:
        print("Filter coefficients have converged!")
        print("Estimated Secondary Path Model (FIR Coefficients):")
        print(filt.w)  # This prints the filter weights representing the secondary path (h_sec)

# Create a stream for real-time audio processing
stream = sd.Stream(
    device=(input_device_a, input_device_b),
    samplerate=samplerate,
    blocksize=blocksize,
    dtype="float32",
    channels=channels,
    latency=latency,
    callback=callback,
)

# Start the audio stream and process in real-time
try:
    with stream:
        print("Starting real-time secondary path estimation...")
        sd.sleep(10000)  # Run for 10 seconds (or however long needed for system identification)
except KeyboardInterrupt:
    print("Stream interrupted.")
