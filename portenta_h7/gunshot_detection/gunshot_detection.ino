/* Edge Impulse Arduino examples
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** `<ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/`.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Includes ---------------------------------------------------------------- */
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <ArduinoBLE.h>

#include <PDM.h>
#include <Gunshot_Detection_inferencing.h>

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// BLE
BLEDevice central;
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* stringCharacteristicUuid = "1A3AC131-31EF-758B-BC51-54A61958EF82";
const int value_length = 20;
BLEService datasetRecorder(deviceServiceUuid);
BLECharacteristic stringCharacteristic(stringCharacteristicUuid, BLERead | BLEWrite, value_length);

const char broker[] = SECRET_BROKER;
int        port     = SECRET_PORT;
const char portenta_topic[]  = SECRET_PORTENTA_TOPIC;
const char ble_sense_topic[] = SECRET_BLESENSE_TOPIC;
const char user_name[] = SECRET_USER_NAME;
const char password[] = SECRET_PASSWORD;

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
static signed short sampleBuffer[2048];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static volatile bool record_ready = false;

// Set to false if not in Debug mode
static const bool debug_mode = true;

void print(const String information, bool new_line = true)
{
  if (!debug_mode) return;
  
  if (new_line) Serial.println(information);
  else Serial.print(information);
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
    // Setup LEDs
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    
    digitalWrite(LEDR, HIGH);   // will turn the LED off
    digitalWrite(LEDG, HIGH);   // will turn the LED off
    digitalWrite(LEDB, HIGH);   // will turn the LED off 

    // put your setup code here, to run once:
    Serial.begin(115200);

    // attempt to connect to WiFi network:
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
      // failed, retry
      Serial.print(".");
      delay(3000);
    }
    Serial.println("Connected to the network");
    Serial.println();

    mqttClient.setId("Portenta_H7");

    // You can provide a username and password for authentication
    mqttClient.setUsernamePassword(user_name, password);
  
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);
  
    if (!mqttClient.connect(broker, port)) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqttClient.connectError());
  
      while (1);
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

     // Initialise BLE communication
    if (!BLE.begin()) {
      print("Starting Bluetooth® Low Energy failed!");
      while (1) blinkLED(LEDR, 1);
    }
  
    // Set advertised local name and service UUID:
    BLE.setLocalName("Dataset_Recorder");
    BLE.setAdvertisedService(datasetRecorder);
  
    // Add the characteristic to the service
    datasetRecorder.addCharacteristic(stringCharacteristic);
  
    // Add service
    BLE.addService(datasetRecorder);
  
    // start advertising
    BLE.advertise();
    blinkLED(LEDG, 3);
    print("Dataset Recorder Ready");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    ei_printf("Starting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Recording...\n");

    bool m = microphone_inference_record();
    if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
    }

    ei_printf("Recording done\n");

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif

    if (central && central.connected())
    {
        char read_buff[value_length] = {'\0'};
        stringCharacteristic.readValue(read_buff, stringCharacteristic.valueLength());
        String read_val = String(read_buff);
        print(read_val);
        if (read_val == "gunshot"){
            mqtt_publish(ble_sense_topic, read_buff);   ///< Send data to the cloud via mqtt
        }
    }
    else
    {
        central = BLE.central();
        if (central)
        {
            print("Connected to central: " + central.address());
            blinkLED(LEDB, 2);
        }
    }

    // Process the result and send report to the cloud
    process_result(result);
}

/**
 * @brief      PDM buffer full callback
 *             Copy audio data to app buffers
 */
static void pdm_data_ready_inference_callback(void)
{
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);

    if ((inference.buf_ready == 0) && (record_ready == true)) {
        for(int i = 0; i < bytesRead>>1; i++) {
            inference.buffer[inference.buf_count++] = sampleBuffer[i];

            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;

    // configure the data receive callback
    PDM.onReceive(&pdm_data_ready_inference_callback);

    // optionally set the gain, defaults to 24
    // Note: values >=52 not supported
    //PDM.setGain(40);

    PDM.setBufferSize(2048);

    // initialize PDM with:
    // - one channel (mono mode)
    if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("ERR: Failed to start PDM!");
        microphone_inference_end();
        return false;
    }

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;


    record_ready = true;
    while (inference.buf_ready == 0) {
        delay(10);
    }

    inference.buf_ready = 0;
    record_ready = false;

    return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    PDM.end();
    ei_free(inference.buffer);
}

/**
 * @brief      Publish information to the mqtt cloud
 */
static void process_result(ei_impulse_result_t& result)
{
    mqttClient.poll();
    const String label_to_look_for = "gunshot";
    int i = find_winner(result);

    if (result.classification[i].label == label_to_look_for)
    {
        mqtt_publish(portenta_topic, label_to_look_for);
    }
}

/**
 * @brief      Find the index of the max value in the result
 */
static int find_winner(ei_impulse_result_t& result)
{
    float max_val = result.classification[0].value;
    int counter = 0;

    for (int i = 1; i < EI_CLASSIFIER_LABEL_COUNT; ++i)
    {
        if (result.classification[i].value > max_val)
        {
            max_val = result.classification[i].value;
            counter = i;
        }
    }

    return counter;
}

/**
 * @brief      Publish information to the mqtt cloud
 */
static void mqtt_publish(const String topic, const String msg)
{
    // send message
    mqttClient.beginMessage(topic);
    mqttClient.print(msg);
    mqttClient.endMessage();
}

/**
 * @brief      Function to blink LED
 */
void blinkLED(int led_colour, int blink_count)
{
    for (int i = 0; i < blink_count; ++i)
    {
        digitalWrite(led_colour, LOW);
        delay(150);
        digitalWrite(led_colour, HIGH);
        delay(150);
    }
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
