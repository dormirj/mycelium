#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include <Adafruit_NeoPixel.h>

#include <Wire.h>
#include "Adafruit_MPR121.h"

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#define PIN_NEO_PIXEL 10  // Arduino pin that connects to NeoPixel
#define NUM_PIXELS 18     // The number of LEDs (pixels) on NeoPixel
#define PIXEL_INTERVAL 100
int pixel_count;
int length_segment = 3;
int num_segments = 6;

//PWM STUFF
const int PWM_CHANNEL = 0;     // ESP32 has 16 channels which can generate 16 independent waveforms

const int PWM_CONTROL_1 = 1;
const int PWM_CONTROL_2 = 2;
const int PWM_CONTROL_3 = 3;

const int PWM_FREQ = 125000;   // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8;  // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);
const int LED_OUTPUT_PIN = 18;

const int CONTROL_1_OUTPUT_PIN = 17;
const int CONTROL_2_OUTPUT_PIN = 16;
const int CONTROL_3_OUTPUT_PIN = 4;

const int LED_GPIO_PIN = 5;
#define WIFI_CHANNEL_MAX (13)

unsigned long updateAudio;
unsigned long updatePixel;
unsigned long updateValues;
int timeDiffAudio;
int timeDiffPixel;
int timeDiffValues;
unsigned long count;

int count_mgmt;
int count_data;
int count_ctrl;
int count_misc;
int count_sender;
int count_recv;
int count_ap;

float control_count;
float control_mgmt;
float control_data;

uint8_t level = 0, channel = 1;

static wifi_country_t wifi_country = { .cc = "DE", .schan = 1, .nchan = 13 };  //Most recent esp32 library struct

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_RGBW + NEO_KHZ400);

Adafruit_MPR121 cap = Adafruit_MPR121();

uint16_t lasttouched = 0;
uint16_t currtouched = 0;

typedef struct {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);

esp_err_t event_handler(void *ctx, system_event_t *event) {
  return ESP_OK;
}

void wifi_sniffer_init(void) {
  nvs_flash_init();
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel) {
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type) {
  switch (type) {
    case WIFI_PKT_MGMT: return "MGMT";
    case WIFI_PKT_DATA: return "DATA";
    case WIFI_PKT_CTRL: return "CTRL";
    case WIFI_PKT_MISC: return "MISC";
    default: break;
  }
}

void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type) {
  // if (type != WIFI_PKT_MGMT)
  //   return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  // printf("PACKET TYPE=%s, CHAN=%02d, RSSI=%02d,"
  //        " ADDR1=%02x:%02x:%02x:%02x:%02x:%02x,"
  //        " ADDR2=%02x:%02x:%02x:%02x:%02x:%02x,"
  //        " ADDR3=%02x:%02x:%02x:%02x:%02x:%02x\n",
  //        wifi_sniffer_packet_type2str(type),
  //        ppkt->rx_ctrl.channel,
  //        ppkt->rx_ctrl.rssi,
  //        /* ADDR1 */
  //        hdr->addr1[0], hdr->addr1[1], hdr->addr1[2],
  //        hdr->addr1[3], hdr->addr1[4], hdr->addr1[5],
  //        /* ADDR2 */
  //        hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
  //        hdr->addr2[3], hdr->addr2[4], hdr->addr2[5],
  //        /* ADDR3 */
  //        hdr->addr3[0], hdr->addr3[1], hdr->addr3[2],
  //        hdr->addr3[3], hdr->addr3[4], hdr->addr3[5]);
  count++;

  switch (type) {
    case WIFI_PKT_MGMT: count_mgmt++;
    case WIFI_PKT_DATA: count_data++;
    case WIFI_PKT_CTRL: count_ctrl++;
    case WIFI_PKT_MISC: count_misc++;
    default: break;
  }
}

// Smoothing Filter ----------------------------
float smoothFactor = 0.2;

float getSmoothed(float value, float feedNewValue) {

 value = smoothFactor* feedNewValue + (1.0-smoothFactor)*value;
 return value;
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);

  delay(20);

  if (!cap.begin(0x5A)) {
    printf("MPR121 not found, check wiring?\n");
    while (1);
  }
  printf("MPR121 found!\n");

  NeoPixel.begin();

  updateAudio = 0;
  updatePixel = 0;
  updateValues = 0;

  control_count = 0;
  control_mgmt = 0;
  control_data = 0;

  pinMode(LED_GPIO_PIN, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LED_OUTPUT_PIN, PWM_CHANNEL);

  ledcSetup(PWM_CONTROL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CONTROL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CONTROL_3, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(CONTROL_1_OUTPUT_PIN, PWM_CONTROL_1);
  ledcAttachPin(CONTROL_2_OUTPUT_PIN, PWM_CONTROL_2);
  ledcAttachPin(CONTROL_3_OUTPUT_PIN, PWM_CONTROL_3);

  delay(20);

  pixel_count = 0;

  count = 0;
  count_mgmt = 0;
  count_data = 0;
  count_ctrl = 0;
  count_misc = 0;
  count_ap = 0;
  count_sender = 0;
  count_recv = 0;

  wifi_sniffer_init();
}

int sine[] = {
  4, 3, 2, 1, 0, 15, 14, 13, 12, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  36, 35, 34, 33, 32, 47, 46, 45, 44, 52, 53, 54, 55, 56, 57, 58, 59, 60
};

// the loop function runs over and over again forever
void loop() {
  //Serial.print("inside loop");

  timeDiffAudio = millis() - updateAudio;
  timeDiffPixel = millis() - updatePixel;
  timeDiffValues = millis() - updateValues;

  if (timeDiffAudio > 30) {

    control_count = getSmoothed(control_count, count);
    control_mgmt = getSmoothed(control_mgmt, count_mgmt);
    control_data = getSmoothed(control_data, count_data);

    ledcWrite(PWM_CHANNEL, esp_random() % 255);
    ledcWrite(PWM_CONTROL_1, control_count);
    ledcWrite(PWM_CONTROL_2, control_mgmt);
    ledcWrite(PWM_CONTROL_3, control_data);

    updateAudio = millis();
  }

  if (timeDiffPixel > PIXEL_INTERVAL) {
    currtouched = cap.touched();

    bool seg[12];

    for(int i = 0; i++; i < 12)
    {seg[i] = false;}

    for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      printf("%i touched\n", i);
        seg[i] = true;
      }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      printf("%i released\n", i);
        seg[i] = false;
      }
    }

    //draw pixel shit
    bool seg1 = seg[0];
    bool seg2 = seg[1];
    bool seg3 = seg[2];
    bool seg4 = seg[3];
    bool seg5 = seg[4];
    bool seg6 = seg[5];

    lasttouched = currtouched;

    NeoPixel.clear();
    for (int k = 0; k < num_segments; k++) {
      for (int j = 0; j < length_segment; j++) {
        int current = k * length_segment + j;
        if (current == pixel_count) {
          if (seg1)
            if (k == 0)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);  // it only takes effect if pixels.show() is called
          if (seg2)
            if (k == 1)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);
          if (seg3)
            if (k == 2)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);
          if (seg4)
            if (k == 3)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);
          if (seg5)
            if (k == 4)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);
          if (seg6)
            if (k == 5)
              NeoPixel.setPixelColor(current, sine[current] + esp_random(), 0, esp_random(), 0);
              else NeoPixel.setPixelColor(current, 0, 0, 0, 0);
        }
      }
    }
    NeoPixel.show();

    pixel_count++;
    pixel_count %= NUM_PIXELS;

    updatePixel = millis();
  }

  if (timeDiffValues > 1000) {

    printf("how many received?: %i, control1: %f, control2: %f, control3: %f\n", count, control_count, control_mgmt, control_data);

    if (digitalRead(LED_GPIO_PIN) == LOW)
      digitalWrite(LED_GPIO_PIN, HIGH);
    else
      digitalWrite(LED_GPIO_PIN, LOW);
    
    wifi_sniffer_set_channel(channel);
    channel = (channel % WIFI_CHANNEL_MAX) + 1;

    count = 0;
    count_ap = 0;
    count_sender = 0;
    count_recv = 0;
    count_mgmt = 0;
    count_data = 0;
    count_ctrl = 0;
    count_misc = 0;
    updateValues = millis();
  }
}