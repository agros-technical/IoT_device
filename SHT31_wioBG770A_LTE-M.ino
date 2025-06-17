/*
 * SHT31-forSORACOM_20250603.ino
 * 使用センサ：Grove温湿度計SHT31(I2C)
 * 測定値をSORACOM Unified endpointへ5分間隔で送信
 * Grove電源を測定時だけON
 * 本体RTCの日時をGMTで取得し、時差とシステム時刻を加算してタイムスタンプとしてJSONに付与
 * プログラムのモジュール化、loop内の処理時間を補正してsample_timeを出力
 * 20260609動作確認済み
 * デバッグ時はUSBDevice.detach();をコメントアウトしてシリアル通信を復帰
 */
///////////////////////////////////////////////////////////////////////////////
// Libraries:
//   http://librarymanager#ArduinoJson 7.0.4
//   http://librarymanager#GroveDriverPack 1.12.0
///////////////////////////////////////////////////////////////////////////////

#include <Adafruit_TinyUSB.h>  //USBシリアル
#include <climits>             //整数型データの制限値定義
#include <csignal>             //シグナル処理
#include <WioCellular.h>       //セルラー回線の利用
#include <ArduinoJson.h>       //JSON取り扱い
#include <ctime>               //strftimeの使用
#include <GroveDriverPack.h>   //Groveセンサの利用
#include <map>                 //HTTPレスポンスの構造体に利用
//#include <Wire.h> //One Wireの使用

static GroveBoard Board;
static JsonDocument JsonDoc;

static GroveTempHumiSHT31 TempHumi(&Board.I2C);  //SHT31向け
float temperature, humidity;                     //測定項目の宣言

//タイムスタンプ関連宣言
time_t currentTime;               //現在時刻のUNIX時間(秒で取得)
unsigned long lastRtcUpdate = 0;  // 最後に RTC を更新した時間
char iso8601[25];  //ISO8601タイムスタンプ用
int diff; //UTCとの時差格納用

//セルラー通信設定
#define SEARCH_ACCESS_TECHNOLOGY (WioCellularNetwork::SearchAccessTechnology::LTEM)  // https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
#define LTEM_BAND (WioCellularNetwork::ALL_LTEM_BAND)                                // すべてのネットワークを探索 https://seeedjp.github.io/Wiki/Wio_BG770A/kb/kb4.html
static const char APN[] = "soracom.io";
static const char HOST[] = "uni.soracom.io";  //Unified endpoint
static constexpr int PORT = 23080;

//各種時間設定
static constexpr int INTERVAL = 1000 * 60 * 5;             // 測定インターバル[ms]
const unsigned long RTC_UPDATE_INTERVAL = 1000 * 60 * 60;  // RTC更新インターバル(1時間)[ms]
static constexpr int POWER_ON_TIMEOUT = 1000 * 20;         // 電源タイムアプト時間[ms]
static constexpr int NETWORK_TIMEOUT = 1000 * 60 * 2;      // ネットワークタイムアウト時間[ms]
static constexpr int RECEIVE_TIMEOUT = 1000 * 10;          // 受信タイムアウト時間[ms]

//HTTPレスポンスの構造体
struct HttpResponse {
  int statusCode;
  std::map<std::string, std::string> headers;
  std::string body;
};

// SHT31センサー用モジュール ※センサ種類に応じて要変更
class SensorModule {
public:
  SensorModule(GroveBoard *board)
    : tempHumi(&board->I2C) {}

  bool init() {
    return tempHumi.Init();
  }

  void read(float &temperature, float &humidity) {
    tempHumi.Read();
    temperature = tempHumi.Temperature;
    humidity = tempHumi.Humidity;
  }

private:
  GroveTempHumiSHT31 tempHumi;
};

// JSON生成用モジュール ※出力項目に応じて要変更
class JsonModule {
public:
  void createJson(JsonDocument &doc, float temperature, float humidity, time_t rawTime) {
    struct tm *jstTime = gmtime(&rawTime);  //UNIX時間(time_t型)を日本時間のtm構造体に変換
    char iso8601[25];                          //ISO8601形式の配列
    strftime(iso8601, sizeof(iso8601), "%Y-%m-%dT%H:%M:%S", jstTime);

    doc["sample_time"] = iso8601;
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
  }
};

//クラスのオブジェクト生成
SensorModule sensor(&Board);
JsonModule jsonModule;

void setup() {
  Serial.begin(115200);  //シリアルポートの設定
  {
    const auto start = millis();
    while (!Serial && millis() - start < 5000) {
      delay(2);
    }
  }

  //USBDevice.detach();   // *********デバッグ時ON*********USB CDC シリアル通信を切断
  
  Serial.println();

  Serial.println("Startup");
  digitalWrite(LED_BUILTIN, HIGH);  //LED ON

  WioCellular.begin();  //セルラー通信の開始
  if (WioCellular.powerOn(POWER_ON_TIMEOUT) != WioCellularResult::Ok) abort();

  WioNetwork.config.searchAccessTechnology = SEARCH_ACCESS_TECHNOLOGY;
  WioNetwork.config.ltemBand = LTEM_BAND;  //LTE-Mバンドの選択
  WioNetwork.config.apn = APN;

  WioNetwork.begin();
  if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) abort();
  digitalWrite(PIN_VGROVE_ENABLE, LOW);  //Grove電源ON
  delay(500);

  //I2Cポートの初期化
  Board.I2C.Enable();
  sensor = SensorModule(&Board);
  if (!sensor.init()) {
    Serial.println("Sensor not found. (´・ω・｀)");
  } else {
    Serial.println("Sensor is found. d(・∀・)");
  }
}

void loop() {
  int time_before_processing = millis(); //処理時間補正用の処理前時刻を格納
  checkAndReconnectNetwork();  // ネットワークの接続状態を監視

  //最初のループで時刻取得
  if (currentTime == 0) {
    lastRtcUpdate = millis();  // 更新時間を記録
    WioCellular.getClock(&currentTime, &diff);  //currentTimeにRTC時刻を格納
    Serial.println("RTC updated in first loop");
  }

  digitalWrite(PIN_VGROVE_ENABLE, LOW);  //Grove電源ON
  digitalWrite(LED_BUILTIN, HIGH);
  JsonDoc.clear();

  // **1時間ごとに RTC の時刻を取得**
  if (millis() - lastRtcUpdate >= RTC_UPDATE_INTERVAL) {
    lastRtcUpdate = millis();  // 更新時間を記録
    WioCellular.getClock(&currentTime, &diff);
    Serial.println("RTC updated");
  }

  sensor.read(temperature, humidity);
  time_t rawTime = currentTime +(diff * 60 * 60 ) + ((millis() - lastRtcUpdate) / 1000);  //現在時刻に時差と前回更新からの経過時間を追加

  jsonModule.createJson(JsonDoc, temperature, humidity, rawTime);

  send(JsonDoc);  //JSON

  Serial.println();

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_VGROVE_ENABLE, HIGH);  //Grove電源OFF
  WioCellular.doWorkUntil(INTERVAL - (millis() - time_before_processing));
}

static bool send(const JsonDocument &doc) {  //測定データの送信およびシリアル出力関数
  Serial.println("### Sending ###");

  Serial.print("Connecting ");
  Serial.print(HOST);
  Serial.print(":");
  Serial.println(PORT);

  {
    WioCellularTcpClient2<WioCellularModule> client{ WioCellular };
    if (!client.open(WioNetwork.config.pdpContextId, HOST, PORT)) {
      Serial.printf("ERROR: Failed to open %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    if (!client.waitforConnect()) {
      Serial.printf("ERROR: Failed to connect %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    Serial.print("Sending ");
    std::string str;
    serializeJson(doc, str);
    printData(Serial, str.data(), str.size());
    Serial.println();

    if (!client.send(str.data(), str.size())) {
      Serial.printf("ERROR: Failed to send socket %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    Serial.println("### Receiving ###");
    static uint8_t recvData[WioCellular.RECEIVE_SOCKET_SIZE_MAX];
    size_t recvSize;
    if (!client.receive(recvData, sizeof(recvData), &recvSize, RECEIVE_TIMEOUT)) {
      Serial.printf("ERROR: Failed to receive socket %s\n", WioCellularResultToString(client.getLastResult()));
      return false;
    }

    printData(Serial, recvData, recvSize);
    Serial.println();
  }

  Serial.println("### Completed ###");

  return true;
}

void checkAndReconnectNetwork() {  //ネットワーク再探索処理
  if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) {
    Serial.println("LTE-M is disconnect. reconnecting... ");

    // ネットワークを一旦停止
    WioNetwork.end();
    delay(2000);  // 少し待機

    // 再探索を開始
    WioNetwork.begin();
    if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) {
      Serial.println("LTE-M reconnect failed");
    } else {
      Serial.println("LTE-M recconect succeeded.");
    }
  }
}

static void abortHandler(int sig) {  //LEDの点滅
  while (true) {
    ledOn(LED_BUILTIN);
    delay(100);
    ledOff(LED_BUILTIN);
    delay(100);
  }
}

template<typename T>                                        //テンプレート定義
void printData(T &stream, const void *data, size_t size) {  //バイナリから1byteずつASCII文字出力
  auto p = static_cast<const char *>(data);

  for (; size > 0; --size, ++p)
    stream.write(0x20 <= *p && *p <= 0x7f ? *p : '.');
}
