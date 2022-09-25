// usb hid lib : https://github.com/espressif/arduino-esp32/tree/master/libraries/USB
#include "USB.h"
#include "USBHID.h"
#include "HIDTypes.h"
USBHID HID;

const int buttonPin = 0;
int previousButtonState = HIGH;
uint8_t keyboard_data[8];
uint8_t mouse_data[5];


// HIDのデバイスID
#define REPORT_KEYBOARD_ID 0x01
#define REPORT_MEDIA_KEYS_ID 0x02
#define REPORT_MOUSE_ID 0x03
#define INPUT_REP_REF_RAW_ID 0x04

#define INPUT_REPORT_RAW_MAX_LEN 32
#define OUTPUT_REPORT_RAW_MAX_LEN 32


static const uint8_t report_descriptor[] = {
  USAGE_PAGE(1),      0x01,          // USAGE_PAGE (Generic Desktop Ctrls)
  USAGE(1),           0x06,          // USAGE (Keyboard)
  COLLECTION(1),      0x01,          // COLLECTION (Application)
  // ------------------------------------------------- Keyboard
  REPORT_ID(1),       REPORT_KEYBOARD_ID,   //   REPORT_ID (1)
  USAGE_PAGE(1),      0x07,          //   USAGE_PAGE (Kbrd/Keypad)

	// モデファイヤキー(修飾キー)
	USAGE_MINIMUM(1),   0xE0,          //   USAGE_MINIMUM (0xE0)(左CTRLが0xe0)
  USAGE_MAXIMUM(1),   0xE7,          //   USAGE_MAXIMUM (0xE7)(右GUIが0xe7)
  LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1), 0x01,          //   Logical Maximum (1)
  REPORT_COUNT(1),    0x08,          //   REPORT_COUNT (8)全部で8つ(左右4つずつ)。
	REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
  HIDINPUT(1),        0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

	// 予約フィールド
	REPORT_COUNT(1),    0x01,          //   REPORT_COUNT (1) ; 1 byte (Reserved)
  REPORT_SIZE(1),     0x08,          //   REPORT_SIZE (8)1ビットが8つ。
  HIDINPUT(1),        0x01,          //   INPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

	// LED状態のアウトプット
	REPORT_COUNT(1),    0x05,          //   REPORT_COUNT (5) ; 5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)全部で5つ。
  REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)LEDにつき1ビット
  USAGE_PAGE(1),      0x08,          //   USAGE_PAGE (LEDs)
  USAGE_MINIMUM(1),   0x01,          //   USAGE_MINIMUM (0x01) ; Num Lock(NumLock LEDが1)
  USAGE_MAXIMUM(1),   0x05,          //   USAGE_MAXIMUM (0x05) ; Kana(KANA LEDが5)
  HIDOUTPUT(1),       0x02,          //   OUTPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)LED report

	// LEDレポートのパディング
	REPORT_COUNT(1),    0x01,          //   REPORT_COUNT (1) ; 3 bits (Padding)
  REPORT_SIZE(1),     0x03,          //   REPORT_SIZE (3)残りの3ビットを埋める。
  HIDOUTPUT(1),       0x03,          //   OUTPUT (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

	// 入力キーのインプット
	REPORT_COUNT(1),    0x06,          //   REPORT_COUNT (6) ; 6 bytes (Keys)全部で6つ。
  REPORT_SIZE(1),     0x08,          //   REPORT_SIZE(8)おのおの8ビットで表現
  LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM(0)キーコードの範囲 開始
  LOGICAL_MAXIMUM(1), 0x65,          //   LOGICAL_MAXIMUM(0x65) ; 101 keys キーコードの範囲 終了

	USAGE_PAGE(1),      0x07,          //   USAGE_PAGE (Kbrd/Keypad)
  USAGE_MINIMUM(1),   0x00,          //   USAGE_MINIMUM (0)
  USAGE_MAXIMUM(1),   0xEF,          //   USAGE_MAXIMUM (0x65)
  HIDINPUT(1),        0x00,          //   INPUT (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

	END_COLLECTION(0),                 // END_COLLECTION
  // ------------------------------------------------- Media Keys
  USAGE_PAGE(1),      0x0C,          // USAGE_PAGE (Consumer)
  USAGE(1),           0x01,          // USAGE (Consumer Control)
  COLLECTION(1),      0x01,          // COLLECTION (Application)
  REPORT_ID(1),       REPORT_MEDIA_KEYS_ID, //   REPORT_ID (3)
  USAGE_PAGE(1),      0x0C,          //   USAGE_PAGE (Consumer)
  LOGICAL_MINIMUM(1), 0x00,          //   LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1), 0x01,          //   LOGICAL_MAXIMUM (1)
  REPORT_SIZE(1),     0x01,          //   REPORT_SIZE (1)
  REPORT_COUNT(1),    0x10,          //   REPORT_COUNT (16)
  USAGE(1),           0xB8,          //   USAGE (Eject)     ; bit 0: 1
  USAGE(1),           0xB5,          //   USAGE (Scan Next Track)     ; bit 0: 2
  USAGE(1),           0xB6,          //   USAGE (Scan Previous Track) ; bit 1: 4
  USAGE(1),           0xB7,          //   USAGE (Stop)                ; bit 2: 8
  USAGE(1),           0xCD,          //   USAGE (Play/Pause)          ; bit 3: 16
  USAGE(1),           0xE2,          //   USAGE (Mute)                ; bit 4: 32
  USAGE(1),           0xE9,          //   USAGE (Volume Increment)    ; bit 5: 64
  USAGE(1),           0xEA,          //   USAGE (Volume Decrement)    ; bit 6: 128
  USAGE(2),           0x94, 0x01,    //   Usage (My Computer) ; bit 0: 1
  USAGE(2),           0x92, 0x01,    //   Usage (Calculator)  ; bit 1: 2
  USAGE(2),           0x2A, 0x02,    //   Usage (WWW fav)     ; bit 2: 4
  USAGE(2),           0x21, 0x02,    //   Usage (WWW search)  ; bit 3: 8
  USAGE(2),           0x26, 0x02,    //   Usage (WWW stop)    ; bit 4: 16
  USAGE(2),           0x24, 0x02,    //   Usage (WWW back)    ; bit 5: 32
  USAGE(2),           0x83, 0x01,    //   Usage (Media sel)   ; bit 6: 64
  USAGE(2),           0x8A, 0x01,    //   Usage (Mail)        ; bit 7: 128
  HIDINPUT(1),        0x02,          //   INPUT (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  END_COLLECTION(0),                 // END_COLLECTION

  // ------------------------------------------------- Mouse
  USAGE_PAGE(1),       0x01, // USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x02, // USAGE (Mouse)
  COLLECTION(1),       0x01, // COLLECTION (Application)
  USAGE(1),            0x01, //   USAGE (Pointer)
  COLLECTION(1),       0x00, //   COLLECTION (Physical)
  REPORT_ID(1),        REPORT_MOUSE_ID, //     REPORT_ID (1)
  // ------------------------------------------------- Buttons (Left, Right, Middle, Back, Forward)
  USAGE_PAGE(1),       0x09, //     USAGE_PAGE (Button)
  USAGE_MINIMUM(1),    0x01, //     USAGE_MINIMUM (Button 1)
  USAGE_MAXIMUM(1),    0x05, //     USAGE_MAXIMUM (Button 5)
  LOGICAL_MINIMUM(1),  0x00, //     LOGICAL_MINIMUM (0)
  LOGICAL_MAXIMUM(1),  0x01, //     LOGICAL_MAXIMUM (1)
  REPORT_SIZE(1),      0x01, //     REPORT_SIZE (1)
  REPORT_COUNT(1),     0x05, //     REPORT_COUNT (5)
  HIDINPUT(1),         0x02, //     INPUT (Data, Variable, Absolute) ;5 button bits
  // ------------------------------------------------- Padding
  REPORT_SIZE(1),      0x03, //     REPORT_SIZE (3)
  REPORT_COUNT(1),     0x01, //     REPORT_COUNT (1)
  HIDINPUT(1),         0x03, //     INPUT (Constant, Variable, Absolute) ;3 bit padding
  // ------------------------------------------------- X/Y position, Wheel
  USAGE_PAGE(1),       0x01, //     USAGE_PAGE (Generic Desktop)
  USAGE(1),            0x30, //     USAGE (X)
  USAGE(1),            0x31, //     USAGE (Y)
  USAGE(1),            0x38, //     USAGE (Wheel)
  LOGICAL_MINIMUM(1),  0x81, //     LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, //     LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, //     REPORT_SIZE (8)
  REPORT_COUNT(1),     0x03, //     REPORT_COUNT (3)
  HIDINPUT(1),         0x06, //     INPUT (Data, Variable, Relative) ;3 bytes (X,Y,Wheel)
  // ------------------------------------------------- Horizontal wheel
  USAGE_PAGE(1),       0x0c, //     USAGE PAGE (Consumer Devices)
  USAGE(2),      0x38, 0x02, //     USAGE (AC Pan)
  LOGICAL_MINIMUM(1),  0x81, //     LOGICAL_MINIMUM (-127)
  LOGICAL_MAXIMUM(1),  0x7f, //     LOGICAL_MAXIMUM (127)
  REPORT_SIZE(1),      0x08, //     REPORT_SIZE (8)
  REPORT_COUNT(1),     0x01, //     REPORT_COUNT (1)
  HIDINPUT(1),         0x06, //     INPUT (Data, Var, Rel)
  END_COLLECTION(0),         //   END_COLLECTION
  END_COLLECTION(0)          // END_COLLECTION

  // ------------------------------------------------- remap
        ,
        0x06, 0x60, 0xFF,
        0x09, 0x61,
        0xa1, 0x01,
        0x85, INPUT_REP_REF_RAW_ID,
        
        0x09, 0x62, 
        0x15, 0x00, 
        0x26, 0xFF, 0x00, 
        0x95, INPUT_REPORT_RAW_MAX_LEN,
        0x75, 0x08, 
        0x81, 0x06, 
      
        0x09, 0x63, 
        0x15, 0x00, 
        0x26, 0xFF, 0x00, 
        0x95, OUTPUT_REPORT_RAW_MAX_LEN, //REPORT_COUNT(32)
        0x75, 0x08, //REPORT_SIZE(8)
        0x91, 0x83, 
        0xC0             // End Collection (Application)
};

class CustomHIDDevice: public USBHIDDevice {
public:
  CustomHIDDevice(void){
    static bool initialized = false;
    if(!initialized){
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }
  
  void begin(void){
    HID.begin();
  }
  
  // HIDからreport_mapの要求
  uint16_t _onGetDescriptor(uint8_t* buffer){
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  // HIDからデータを受け取る
  void _onOutput(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    int i;
    Serial.printf("_onOutput: report_id %d / len %d\r\n", report_id, len);
    if (report_id == REPORT_KEYBOARD_ID) { // caps lockとか
      Serial.printf("Keyboard status: %x\r\n", buffer[0]);

    } else if (report_id == INPUT_REP_REF_RAW_ID) { // HID Raw
      for (i=0; i<len; i++) {
        Serial.printf("%02x", buffer[i]);
      }
      Serial.printf("\r\n");
    }
  }

  bool send(uint8_t * value){
    return HID.SendReport(REPORT_KEYBOARD_ID, value, 8);
  }

  bool mouse_send(uint8_t x) {
    mouse_data[0] = 0x00;
    mouse_data[1] = x;
    mouse_data[2] = 0x00;
    mouse_data[3] = 0x00;
    mouse_data[4] = 0x00;
    return HID.SendReport(REPORT_MOUSE_ID, mouse_data, 5);

  }

};

CustomHIDDevice Device;



static void usbEventCallback(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data){
    Serial.printf("HID: eventBase %d  eventID %d \r\n", event_base, event_id);
  if(event_base == ARDUINO_USB_EVENTS){
    arduino_usb_event_data_t * data = (arduino_usb_event_data_t*)event_data;
    Serial.printf("ARDUINO_USB_EVENTS: eventBase %d \r\n", ARDUINO_USB_EVENTS);
    switch (event_id){
      // 0 USB 刺された
      case ARDUINO_USB_STARTED_EVENT:
        Serial.println("USB PLUGGED");
        break;
      // 1 USB 外した
      case ARDUINO_USB_STOPPED_EVENT:
        Serial.println("USB UNPLUGGED");
        break;
      case ARDUINO_USB_SUSPEND_EVENT:
        Serial.printf("USB SUSPENDED: remote_wakeup_en: %u\n", data->suspend.remote_wakeup_en);
        break;
      case ARDUINO_USB_RESUME_EVENT:
        Serial.println("USB RESUMED");
        break;
      
      default:
        break;
    }
  }
};

void setup() {
  Serial.begin(115200);
  int i;
  for (i=0; i<8; i++) keyboard_data[i] = 0x00;
  pinMode(buttonPin, INPUT_PULLUP);
  // USB HID 初期化
  Device.begin();
  USB.onEvent(usbEventCallback);
  USB.begin();
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  if (HID.ready() && buttonState != previousButtonState) {
    previousButtonState = buttonState;
    if (buttonState == LOW) {
      keyboard_data[3] = 0x10;
    } else {
      keyboard_data[3] = 0x00;
    }
    // Device.send(keyboard_data);
    Device.mouse_send(keyboard_data[3]);
    delay(100);
  }
}
