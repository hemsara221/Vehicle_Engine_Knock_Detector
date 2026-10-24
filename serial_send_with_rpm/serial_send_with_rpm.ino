#include <driver/i2s.h>
#include <WiFi.h>

#define I2S_SAMPLE_RATE     90000          // 90 kHz sampling rate
#define ADC_INPUT_PIN       ADC1_CHANNEL_6 // GPIO 34 (Piezo Input)
#define PULSE_PIN           4             // GPIO 23 (New Safe Pickup Coil Pulse Pin)
#define SAMPLES             512            // Block size sent to Python

// Volatile variables for the background RPM hardware interrupt tracking routine
volatile unsigned long last_pulse_time = 0;
volatile uint16_t time_between_pulses_ms = 0;

// Interrupt Service Routine: Fires instantaneously when the flywheel passing magnet triggers the coil pulse
void IRAM_ATTR onPulse() {
    unsigned long now = millis();
    unsigned long duration = now - last_pulse_time;
    if (duration > 5) { // 5 millisecond hardware debounce to reject spark plug firing noise
        time_between_pulses_ms = (uint16_t)duration;
        last_pulse_time = now;
    }
}

void setup_i2s_adc() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate =  I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = SAMPLES,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT_PIN);
    i2s_adc_enable(I2S_NUM_0);
}

void setup() {
    WiFi.disconnect(true);  // Disconnects from the network and clears config
    WiFi.mode(WIFI_OFF);    // Turns off the Wi-Fi mode completely
    btStop();    

    Serial.begin(500000);     
    setup_i2s_adc();
    
    // Configure the physical digital pulse lane with the interrupt macro hook
    pinMode(PULSE_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN), onPulse, FALLING);
}

void loop() {
    uint16_t i2s_buffer[SAMPLES];
    size_t bytes_read;
  
    // Read raw data from DMA memory
    i2s_read(I2S_NUM_0, &i2s_buffer, sizeof(i2s_buffer), &bytes_read, portMAX_DELAY);
    int num_samples = bytes_read / 2;
    
    if (num_samples == SAMPLES) {
        // Apply 12-bit mask to clean up the ADC data before sending
        for (int i = 0; i < SAMPLES; i++) {
            i2s_buffer[i] &= 0x0FFF;
        }
        
        // 1. Send the unique 4-byte header sequence (4 bytes)
        uint8_t sync_header[4] = {0xAA, 0xBB, 0xCC, 0xDD};
        Serial.write(sync_header, 4);
        
        // 2. Write the raw array directly as raw binary bytes (512 samples * 2 bytes = 1024 bytes)
        Serial.write((uint8_t*)i2s_buffer, SAMPLES * 2);

        // 3. Append the newest 2-byte rotation measurement timestamp right to the package tail
        // (This adds 0 overhead and preserves your high-speed loop)
        uint16_t current_pulse_delta = time_between_pulses_ms;
        Serial.write((uint8_t*)&current_pulse_delta, 2);
        
    }
    // Reset pulse time
    time_between_pulses_ms = 0;
    
    // To stops overwhelming the PC from serial connection.
    delay(40); 
}
