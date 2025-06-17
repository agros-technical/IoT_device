/*
 * SLT5006_BG770A_LTE-M.ino
 * Grove温湿度計SHT31の値をSORACOM Unified endpointへ5分間隔で送信
 * Grove電源を測定時だけONすると測定開始ができない
 * ※20250617修正、動作確認
 */
///////////////////////////////////////////////////////////////////////////////
// Libraries:
//   http://librarymanager#ArduinoJson 7.0.4
//   http://librarymanager#GroveDriverPack 1.12.0
///////////////////////////////////////////////////////////////////////////////

#include <Adafruit_TinyUSB.h>  //USBシリアル
#include <climits>             //整数型データの制限値定義
#include <csignal>             //シグナル処理
//#include <Wire.h> //One Wireの使用
#include <WioCellular.h>      //セルラー回線の利用
#include <ArduinoJson.h>      //JSON取り扱い
#include <ctime>              //strftimeの使用
#include <map>                //HTTPレスポンスの構造体に利用
#include <GroveDriverPack.h>  //Groveセンサの利用

static GroveBoard Board;
static JsonDocument JsonDoc;

char SoilData[100];                                             //土壌センサ用データ取得配列
int SoilDataLength;                                             //土壌センサ用データ長
float TEMP, EC_BULK, VWC_ROCK, VWC, VWC_COCO, EC_PORE = { 0 };  //土壌センサ測定値変数

//タイムスタンプ関連宣言
time_t currentTime;               //現在時刻のUNIX時間(秒で取得)
unsigned long lastRtcUpdate = 0;  // 最後に RTC を更新した時間
char iso8601[25];                 //ISO8601タイムスタンプ用
int diff;                         //UTCとの時差格納用

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

// 土壌センサー用モジュール ※センサ種類に応じて要変更
class SensorModule {
public:
  void start() {
    ///土壌センサSLT5006への測定開始////////////////
    uint8_t txData[] = { 0x02, 0x07, 0x01, 0x01, 0x0D, 0x70 };  //SLT5006用測定開始コマンド
    Serial1.write(txData, sizeof(txData));
    delay(50);
    if (!Serial1.available()) {                // 受信データがあるか？
      Serial.println("Sensor is not found.");  //データが無いとき
    } else {
      receiveData1();  //測定開始信号のレスポンス確認
    }
  }
  bool read(float &TEMP, float &EC_BULK, float &VWC_ROCK, float &VWC, float &VWC_COCO, float &EC_PORE) {
    int flag = 0;
    while (flag == 0) {
      uint8_t txData2[] = { 0x01, 0x08, 0x01, 0x00, 0xE6 };
      Serial1.write(txData2, sizeof(txData2));
      delay(50);
      receiveData2(&flag);
      delay(500);
    }

    uint8_t txData3[] = { 0x01, 0x13, 0x10, 0xFC, 0x2C };  //SLT5006用測定データ読出しコマンド
    Serial1.write(txData3, sizeof(txData3));
    delay(50);
    uint8_t val[21] = { 1 };
    receiveData3(val);

    TEMP = 0;
    EC_BULK = 0;
    VWC_ROCK = 0;
    VWC = 0;
    VWC_COCO = 0;
    EC_PORE = 0;
    TEMP = (val[3] + val[4] * 256) * 0.0625;
    EC_BULK = (val[5] + val[6] * 256) * 0.001;
    VWC_ROCK = (val[7] + val[8] * 256) * 0.1;
    VWC = (val[9] + val[10] * 256) * 0.1;
    VWC_COCO = (val[11] + val[12] * 256) * 0.1;
    EC_PORE = (val[15] + val[16] * 256) * 0.001;

    return true;
  }

private:
  void receiveData1() {
    if (Serial1.available() > 0) {
      uint8_t receivedData[6];                                                // 受信データ用バッファ
      size_t length = Serial1.readBytes(receivedData, sizeof(receivedData));  // データを受信
      Serial.print("Received1: ");
      for (size_t i = 0; i < length; i++) {
        Serial.print(receivedData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  void receiveData2(int *flag) {
    if (Serial1.available() > 0) {
      uint8_t receivedData[6];
      size_t length = Serial1.readBytes(receivedData, sizeof(receivedData));
      Serial.print("Received2: ");
      for (size_t i = 0; i < length; i++) {
        Serial.print(receivedData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      if (receivedData[3] != 0) {
        *flag = 1;
      }
    }
  }

  void receiveData3(uint8_t *val) {
    if (Serial1.available() > 0) {
      uint8_t receivedData[21];
      size_t length = Serial1.readBytes(receivedData, sizeof(receivedData));
      Serial.print("Received3: ");
      for (size_t i = 0; i < length; i++) {
        val[i] = receivedData[i];
        Serial.print(receivedData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
};

// JSON生成用モジュール ※出力項目に応じて要変更
class JsonModule {
public:
  void createJson(JsonDocument &doc, float TEMP, float EC_BULK, float VWC_ROCK, float VWC, float VWC_COCO, float EC_PORE, time_t rawTime) {
    struct tm *jstTime = gmtime(&rawTime);  //UNIX時間(time_t型)を日本時間のtm構造体に変換
    char iso8601[25];                       //ISO8601形式の配列
    strftime(iso8601, sizeof(iso8601), "%Y-%m-%dT%H:%M:%S", jstTime);

    doc["sample_time"] = iso8601;
    doc["TEMP"] = TEMP;
    doc["EC_BULK"] = EC_BULK;
    doc["VWC_ROCK"] = VWC_ROCK;
    doc["VWC"] = VWC;
    doc["VWC_COCO"] = VWC_COCO;
    doc["EC_PORE"] = EC_PORE;
  }
};

//クラスのオブジェクト生成
SensorModule sensor;  // センサーモジュールを作成
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

  Serial.print("Startup");          //セルラー通信の開始
  digitalWrite(LED_BUILTIN, HIGH);  //LED ON

  WioCellular.begin();
  if (WioCellular.powerOn(POWER_ON_TIMEOUT) != WioCellularResult::Ok) abort();

  WioNetwork.config.searchAccessTechnology = SEARCH_ACCESS_TECHNOLOGY;
  WioNetwork.config.ltemBand = LTEM_BAND;  //LTE-Mバンドの選択
  WioNetwork.config.apn = APN;

  WioNetwork.begin();
  if (!WioNetwork.waitUntilCommunicationAvailable(NETWORK_TIMEOUT)) abort();
  digitalWrite(PIN_VGROVE_ENABLE, LOW);  //Grove電源ON

  Serial1.begin(9600);  // UARTの通信ボーレート設定
}


//メインのループ処理/////////////
void loop() {
  int time_before_processing = millis();  //処理時間補正用の処理前時刻を格納
  checkAndReconnectNetwork();             // ネットワークの接続状態を監視

  //最初のループで時刻取得
  if (currentTime == 0) {
    lastRtcUpdate = millis();                   // 更新時間を記録
    WioCellular.getClock(&currentTime, &diff);  //currentTimeにRTC時刻を格納
    Serial.println("RTC updated in first loop");
  }

  digitalWrite(PIN_VGROVE_ENABLE, LOW);  //Grove電源ON
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  sensor.start();  // 土壌センサの測定開始設定
  JsonDoc.clear();

  // **1時間ごとに RTC の時刻を取得**
  if (millis() - lastRtcUpdate >= RTC_UPDATE_INTERVAL) {
    lastRtcUpdate = millis();  // 更新時間を記録
    WioCellular.getClock(&currentTime, &diff);
    Serial.println("RTC updated");
  }

  sensor.read(TEMP, EC_BULK, VWC_ROCK, VWC, VWC_COCO, EC_PORE);
  time_t rawTime = currentTime + (diff * 60 * 60) + ((millis() - lastRtcUpdate) / 1000);  //現在時刻に時差と前回更新からの経過時間を追加

  jsonModule.createJson(JsonDoc, TEMP, EC_BULK, VWC_ROCK, VWC, VWC_COCO, EC_PORE, rawTime);

  send(JsonDoc);  //JSONを送信

  Serial.println();


  //測定の終了//////////////////////////////
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(PIN_VGROVE_ENABLE, HIGH);  //Grove電源OFF
  WioCellular.doWorkUntil(INTERVAL - (millis() - time_before_processing));
}


//send:データの外部送信/////////////////
static bool send(const JsonDocument &doc) {  //測定データの送信およびシリアル出力関数
  Serial.println("### Sending");

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

//abortHandler:LEDの点滅関数/////////////////////
static void abortHandler(int sig) {  //LEDの点滅
  while (true) {
    ledOn(LED_BUILTIN);
    delay(100);
    ledOff(LED_BUILTIN);
    delay(100);
  }
}

//テンプレート/////////////////////
template<typename T>
void printData(T &stream, const void *data, size_t size) {
  auto p = static_cast<const char *>(data);

  for (; size > 0; --size, ++p)
    stream.write(0x20 <= *p && *p <= 0x7f ? *p : '.');
}
