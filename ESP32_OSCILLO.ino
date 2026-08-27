#include <SPI.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

/* ---------------- Pin config ---------------- */
#define PIN_SCK    18   // spi pins to connect with stm32
#define PIN_MISO   19
#define PIN_MOSI   23
#define PIN_CS      5
#define PIN_DRDY    4

/* ---------------- WiFi config ---------------- */
const char *WIFI_SSID = "sourav";    // WiFi name
const char *WIFI_PASS = "12345678";  // WiFi password

/* ---------------- Protocol constants ---------------- */
#define FRAME_MAGIC0   0xAA  // header code sent by stm32 to validate data 
#define FRAME_MAGIC1   0x55  // header code 2 
#define CMD_WAVEFORM   0x02
#define MAX_SAMPLES    256   // max buffer size 

/* ---------------- Voltage/frequency config ---------------- */
#define VREF_MV         3300.0f    // stm32 operating voltage hardcoded, should be calculated and callibrated
#define ADC_FULLSCALE   4095.0f    // 2^12 adc resolutions
#define SAMPLE_RATE_HZ  10000.0f  /* same as stm32 adc sample rate*/
struct WaveStats {
  float vmin_mv;
  float vmax_mv;
  float vpp_mv;     // max - min
  float vavg_mv;    // average / DC level
  float freq_hz;    // rough estimate 
};

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

volatile bool drdy_flag = false;
volatile uint32_t drdy_count = 0;   // no. of times data ready is fired 
uint32_t frames_ok = 0;             // no. of good frames recieved
uint32_t frames_bad_header = 0;
uint32_t frames_bad_checksum = 0;
uint32_t frames_bad_len = 0;

void IRAM_ATTR onDrdyRising()
{
    drdy_flag = true;
    drdy_count++;
}

WaveStats computeStats(const uint16_t *samples, uint16_t len)
{
  WaveStats s = {0, 0, 0, 0, 0};
  if (len == 0) return s;

  uint16_t vmin_raw = 0xFFFF, vmax_raw = 0;
  uint32_t sum = 0;

  for (uint16_t i = 0; i < len; i++) {
    uint16_t v = samples[i];
    sum += v;                        // sum of all 128 samples for calculating avg voltage
    if (v < vmin_raw) vmin_raw = v;  // taking v min the highest value and then comaparing it with each voltage value and decrementing value to get the lowest
    if (v > vmax_raw) vmax_raw = v;  // taking v max the lowest value and then coparing it with each value and increment to get highest value;
  }

  float avg_raw = (float)sum / (float)len;  // avg voltage in mv

  s.vmin_mv = vmin_raw * VREF_MV / ADC_FULLSCALE;
  s.vmax_mv = vmax_raw * VREF_MV / ADC_FULLSCALE;
  s.vpp_mv  = s.vmax_mv - s.vmin_mv;               // vpp is vmax - vmin per frame
  s.vavg_mv = avg_raw * VREF_MV / ADC_FULLSCALE;

  uint32_t crossings = 0;
  for (uint16_t i = 1; i < len; i++) {
    if ((float)samples[i - 1] < avg_raw && (float)samples[i] >= avg_raw) {
      crossings++;
    }
  }
  if (crossings > 0 && len > 1) {
    float duration_s = (float)(len - 1) / SAMPLE_RATE_HZ;
    s.freq_hz = (float)crossings / duration_s;
  }

  return s;
}

void sendStats(const WaveStats &s)
{
  char json[160];
  snprintf(json, sizeof(json),
    "{\"vmin\":%.1f,\"vmax\":%.1f,\"vpp\":%.1f,\"vavg\":%.1f,\"freq\":%.1f,\"vref\":%.1f}",
    s.vmin_mv, s.vmax_mv, s.vpp_mv, s.vavg_mv, s.freq_hz, (double)VREF_MV);
  ws.textAll(json);
}

/* ---------------- web page ---------------- */
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>STM32 Oscilloscope</title>
<style>
  body { background:#111; color:#ddd; font-family: sans-serif; text-align:center; }
  canvas { background:#000; border:1px solid #444; margin-top:20px; }
  #status { margin-top:10px; font-size:14px; color:#8f8; }
  #stats {
    display:flex; flex-wrap:wrap; justify-content:center; gap:10px;
    max-width:900px; margin:16px auto 0;
  }
  .stat {
    background:#1b1b1b; border:1px solid #333; border-radius:6px;
    padding:8px 14px; min-width:110px;
  }
  .stat .label { font-size:11px; color:#888; text-transform:uppercase; letter-spacing:0.5px; }
  .stat .value { font-size:20px; color:#0f0; font-weight:bold; margin-top:2px; }
</style>
</head>
<body>
<h2>STM32 Oscilloscope</h2>
<canvas id="c" width="900" height="320"></canvas>
<div id="status">connecting...</div>

<div id="stats">
  <div class="stat"><div class="label">Vpp</div><div class="value" id="s_vpp">-- mV</div></div>
  <div class="stat"><div class="label">Vavg</div><div class="value" id="s_vavg">-- mV</div></div>
  <div class="stat"><div class="label">Vmin</div><div class="value" id="s_vmin">-- mV</div></div>
  <div class="stat"><div class="label">Vmax</div><div class="value" id="s_vmax">-- mV</div></div>
  <div class="stat"><div class="label">Freq (est.)</div><div class="value" id="s_freq">-- Hz</div></div>
</div>

<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
let frameCount = 0;
let lastSecond = Date.now();
let fps = 0;

const statEls = {
  vpp: document.getElementById('s_vpp'),
  vavg: document.getElementById('s_vavg'),
  vmin: document.getElementById('s_vmin'),
  vmax: document.getElementById('s_vmax'),
  freq: document.getElementById('s_freq'),
};

/* Filled in from each stats message so the plot's voltage labels track
   the firmware's actual VREF_MV - this default is just a starting
   guess before the first message arrives. */
let latestVrefMv = 3300;

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.binaryType = 'arraybuffer';

  ws.onopen = () => { statusEl.textContent = 'connected'; };
  ws.onclose = () => { statusEl.textContent = 'disconnected - retrying...'; setTimeout(connect, 1000); };
  ws.onerror = () => { ws.close(); };

  ws.onmessage = (evt) => {
    if (typeof evt.data === 'string') {
      updateStats(evt.data);
      return;
    }

    const data = new Uint16Array(evt.data);
    draw(data);

    frameCount++;
    const now = Date.now();
    if (now - lastSecond >= 1000) {
      fps = frameCount;
      frameCount = 0;
      lastSecond = now;
    }
    statusEl.textContent = `connected - ${data.length} ;
  };
}

function updateStats(jsonText) {
  let d;
  try { d = JSON.parse(jsonText); } catch (e) { return; }

  statEls.vpp.textContent  = `${d.vpp.toFixed(0)} mV`;
  statEls.vavg.textContent = `${d.vavg.toFixed(0)} mV`;
  statEls.vmin.textContent = `${d.vmin.toFixed(0)} mV`;
  statEls.vmax.textContent = `${d.vmax.toFixed(0)} mV`;
  statEls.freq.textContent = d.freq > 0 ? `${d.freq.toFixed(1)} Hz` : '-- Hz';

  if (d.vref) latestVrefMv = d.vref;
}

function draw(data) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  /* grid, with each horizontal line labeled in volts. Top of canvas =
     full-scale (Vref), bottom = 0V, matching the y-mapping used below
     for the trace itself. */
  ctx.strokeStyle = '#222';
  ctx.beginPath();
  for (let gx = 0; gx < canvas.width; gx += 50) { ctx.moveTo(gx, 0); ctx.lineTo(gx, canvas.height); }
  for (let gy = 0; gy < canvas.height; gy += 40) { ctx.moveTo(0, gy); ctx.lineTo(canvas.width, gy); }
  ctx.stroke();

  ctx.fillStyle = '#0c0';
  ctx.font = '10px sans-serif';
  ctx.textBaseline = 'middle';
  for (let gy = 0; gy < canvas.height; gy += 40) {
    const volts = (latestVrefMv / 1000) * (1 - gy / canvas.height);
    ctx.fillText(`${volts.toFixed(2)}V`, 4, gy + 8);
  }

  /* waveform */
  ctx.strokeStyle = '#0f0';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  const xScale = canvas.width / data.length;
  for (let i = 0; i < data.length; i++) {
    const y = canvas.height - (data[i] / 4095) * canvas.height;
    const x = i * xScale;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }
  ctx.stroke();
}

connect();
</script>
</body>
</html>
)HTML";

/* ---------------- SPI frame reading ---------------- */

void readAndBroadcastFrame()
{
  uint8_t header[5];     // buffer for reading the 5 byte header

  digitalWrite(PIN_CS, LOW);     // putting cs low for start spi transfer
  for (int i = 0; i < 5; i++) {
    header[i] = SPI.transfer(0x00);   // transfer random 8 bit value to stm to get adc value
  }

  if (header[0] != FRAME_MAGIC0 || header[1] != FRAME_MAGIC1) {    // validating header are correct or not, return early if not
    digitalWrite(PIN_CS, HIGH);
    frames_bad_header++;       // keeping record no. of bad frames
    return;
  }

  uint8_t cmd = header[2];      // getting cmd 
  uint16_t len = header[3] | ((uint16_t)header[4] << 8);  // getting the length of adc buffer

  if (len == 0 || len > MAX_SAMPLES) {   //early return if len > max samples 
    digitalWrite(PIN_CS, HIGH);
    frames_bad_len++;
    return;
  }

  static uint8_t payload[MAX_SAMPLES * 2];
  for (uint16_t i = 0; i < len * 2; i++) {
    payload[i] = SPI.transfer(0x00);
  }

  uint8_t checksum = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);

  uint8_t calc = 0;
  for (uint16_t i = 0; i < len * 2; i++) calc ^= payload[i];

  if (calc != checksum) {
    frames_bad_checksum++;
    return;
  }

  if (cmd == CMD_WAVEFORM) {
    frames_ok++;

    ws.binaryAll(payload, len * 2);

    static uint16_t samples[MAX_SAMPLES];
    for (uint16_t i = 0; i < len; i++) {
      samples[i] = (uint16_t)payload[i * 2] | ((uint16_t)payload[i * 2 + 1] << 8);
    }

    WaveStats stats = computeStats(samples, len);
    sendStats(stats);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_DRDY, INPUT);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
  SPI.setFrequency(1000000);      
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), onDrdyRising, RISING);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", INDEX_HTML);
  });
  server.begin();
}

void loop()
{
  if (drdy_flag) {
    drdy_flag = false;
    readAndBroadcastFrame();
  }

  static uint32_t last_report = 0;
  uint32_t now = millis();
  if (now - last_report >= 1000) {
    last_report = now;
    Serial.printf(
      "drdy=%lu ok=%lu bad_header=%lu bad_len=%lu bad_checksum=%lu clients=%u\n",
      (unsigned long)drdy_count, (unsigned long)frames_ok,
      (unsigned long)frames_bad_header, (unsigned long)frames_bad_len,
      (unsigned long)frames_bad_checksum, ws.count()
    );
  }
}
