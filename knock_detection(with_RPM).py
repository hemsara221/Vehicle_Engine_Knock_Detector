import serial
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# SETTINGS
# ============================================================

SERIAL_PORT = 'COM3'
BAUD_RATE = 500000

SAMPLE_RATE = 72540      # ESP32 I2S requested sampling rate
SAMPLES = 512

HEADER = b'\xaa\xbb\xcc\xdd'

MIN_FREQ = 3000            # Ignore frequencies below 
MAX_FREQ = 11000          # Detect up to this frequency

# Minimum  strength required before reporting a frequency
MIN_STRENGTH = 700

# expected knock range to show in orange color in plot
KNOCK_RANGE = [4500, 6500]



# ============================================================
# SERIAL
# ============================================================

try:
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0.1
    )

    print(
        f"Connected to ESP32 on {SERIAL_PORT} "
        f"at {SAMPLE_RATE} Hz"
    )

except Exception as e:
    print(f"Error: {e}")
    sys.exit()


# ============================================================
# FFT SETUP
# ============================================================

frequencies = np.fft.rfftfreq(
    SAMPLES,
    d=1 / SAMPLE_RATE
)

# Hann window
window = np.hanning(SAMPLES)

# Only search between MIN_FREQ and MAX_FREQ
frequency_mask = (
    (frequencies >= MIN_FREQ) &
    (frequencies <= MAX_FREQ)
)

valid_indices = np.where(frequency_mask)[0]

# Pre-calculate the frequencies in kHz
frequencies_khz = frequencies[valid_indices] / 1000.0


# ============================================================
# MATPLOTLIB INTERACTIVE LIVE GRAPH SETUP
# ============================================================

plt.ion()
fig, ax = plt.subplots(figsize=(10, 5))
line, = ax.plot(frequencies_khz, np.zeros(len(valid_indices)), color='#007acc', lw=2)


ax.set_title(f"Engine Knock Spectrum ({MIN_FREQ} Hz - {MAX_FREQ} Hz)", fontsize=12, fontweight='bold')
ax.set_xlabel("Frequency (kHz)", fontsize=10)
ax.set_ylabel("Strength", fontsize=10)
ax.set_xlim(MIN_FREQ / 1000.0, MAX_FREQ / 1000.0)
ax.set_ylim(0, 2500)  
ax.grid(True, linestyle='--', alpha=0.5)


ax.axvspan(KNOCK_RANGE[0]/1000.0 , KNOCK_RANGE[1]/1000.0 , color='orange', alpha=0.2, label='Knock Target Window')
ax.legend(loc='upper right')


info_text = fig.text(
    0.12, 0.96, 
    "Initializing Sensor...", 
    fontsize=11, 
    fontweight='bold',
    color='#333333',
    bbox=dict(facecolor='white', alpha=0.9, edgecolor='#cccccc', boxstyle='round,pad=0.4')
)


# Link the window's close button to our handler function
def on_close(event):
    print("\nGraph window closed. Cleaning up and stopping program...")
    try:
        ser.close()
    except:
        pass
    sys.exit(0) 

fig.canvas.mpl_connect('close_event', on_close)


# ============================================================
# REAL-TIME ANIMATION STREAM PROCESSING
# ============================================================

def update(frame):
    if ser.in_waiting > 2048:
        ser.reset_input_buffer()
        
    if ser.in_waiting < 4:
        return line, info_text

    # Search for sync header sequence
    current_byte = ser.read(1)
    if not current_byte or current_byte != HEADER[0:1]:
        return line, info_text

    if ser.read(3) != HEADER[1:4]:
        return line, info_text

    # Read binary string: 512 samples (1024 bytes) + 2 tail pulse bytes = 1026 total
    raw_bytes = ser.read((SAMPLES * 2) + 2)  
    if len(raw_bytes) != (SAMPLES * 2) + 2:
        return line, info_text

    # Unpack raw audio array
    piezo_bytes = raw_bytes[:SAMPLES * 2]
    raw_samples = np.frombuffer(piezo_bytes, dtype=np.uint16).astype(np.float64)
    
    # Extract the engine rotation pulse values
    pulse_bytes = raw_bytes[SAMPLES * 2:]
    pulse_delta_array = np.frombuffer(pulse_bytes, dtype=np.uint16)
    pulse_delta_ms = float(pulse_delta_array[0]) if len(pulse_delta_array) > 0 else 0.0

    # Translate pulse latency metrics to RPM
    if pulse_delta_ms > 0.0:
        engine_rpm = (1000.0 / pulse_delta_ms) * 60.0
    else:
        engine_rpm = 0.0

    # Execute math operations
    raw_samples -= np.mean(raw_samples)
    signal = raw_samples * window
    fft_result = np.fft.rfft(signal)
    fft_magnitude = np.abs(fft_result)

    # Filter out targeted band array
    selected_magnitudes = fft_magnitude[valid_indices]
    peak_position = np.argmax(selected_magnitudes)
    peak_index = valid_indices[peak_position]
    peak_frequency = frequencies[peak_index]
    peak_strength = fft_magnitude[peak_index]

    # ----------------------------------------------------
    # DATA INTERPRETATION AND RENDERING 
    # ----------------------------------------------------
    if peak_strength >= MIN_STRENGTH:
        display_str = f"Vibration: {peak_frequency / 1000:.2f} kHz ({int(peak_strength)}) | Speed: {int(engine_rpm)} RPM"
        
        line.set_ydata(selected_magnitudes)
        current_max = np.max(selected_magnitudes)

        info_text.set_text(f" {display_str}")
        info_text.set_color("#1F52C0") 

        
    else:
        info_text.set_text(f"Vibration: Normal | Engine Speed: {int(engine_rpm)} RPM")
        info_text.set_color('#555555') 

        line.set_ydata(np.zeros(len(valid_indices)))
        current_max = 0

    # ----------------------------------------------------
    # TWO-WAY ADAPTIVE SCALING
    # ----------------------------------------------------
    current_ymin, current_ymax = ax.get_ylim()
    
    if current_max > current_ymax:
        ax.set_ylim(0, current_max * 1.3)
        
    elif current_max < (current_ymax * 0.5):
        new_ceiling = max(2500, current_max * 1.5)
        ax.set_ylim(0, new_ceiling)
        
    return line, info_text


# ============================================================
# EXECUTION ANCHOR
# ============================================================

try:

    ani = FuncAnimation(fig, update, blit=False, interval=33, cache_frame_data=False)
    plt.show(block=True)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    try:
        ser.close()
    except:
        pass
