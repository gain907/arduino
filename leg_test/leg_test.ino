// ==========================================
// 스파이더 로봇 교육용 1단계 : 관절 테스트 펌웨어
// ==========================================

#include <Servo.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

String FW_Version = "Joint Test Mode";
String AP_NameString = "Spider Robot_김석훈"; 
const char WiFiAPPSK[] = "12345678";

const int ALLMATRIX = 9; 
const int ALLSERVOS = 8; 
const int PWMRES_Min = 1; 
const int PWMRES_Max = 180; 
const int SERVOMIN = 400; 
const int SERVOMAX = 2400; 
const int BASEDELAYTIME = 10; 

int Running_Servo_POS[ALLMATRIX];
float globalSpeedFactor = 1.0; 

ESP8266WebServer server(80);

Servo GPIO14SERVO;
Servo GPIO12SERVO;
Servo GPIO13SERVO;
Servo GPIO15SERVO;
Servo GPIO16SERVO;
Servo GPIO5SERVO;
Servo GPIO4SERVO;
Servo GPIO2SERVO;

// =========================================================
// 관절 각도 시리얼 출력 (디버깅)
// =========================================================
void debugPrintAngles(String label) {
  Serial.println("\n[" + label + "] 현재 모터 각도 상태:");
  for (int i = 0; i < ALLSERVOS; i++) {
    Serial.print("모터 ");
    Serial.print(i);
    Serial.print("번: ");
    Serial.print(Running_Servo_POS[i]);
    Serial.print("도 | ");
    if(i == 3) Serial.println(); 
  }
  Serial.println("\n------------------------------------------------");
}

// =========================================================
// 자세 제어 시퀀스
// =========================================================
int Servo_Act_0[] = { 135, 45, 135, 45, 45, 135, 45, 135, 500 }; 

int Servo_Prg_1_Step = 2;
int Servo_Prg_1[][ALLMATRIX] = {
  { 90,  90,  90,  90,  90,  90,  90,  90,  500 }, 
  { 70,  90,  90, 110, 110,  90,  90,  70,  500 }
};

int Servo_Prg_12_Step = 2;
int Servo_Prg_12[][ALLMATRIX] = {
  { 30,  90,  90, 150, 150,  90,  90,  30,  500 }, 
  { 30,  45, 135, 150, 150, 135,  45,  30,  500 }
};

// --------------------------------------------------------------------------------
// 서보모터 제어 코어 함수
// --------------------------------------------------------------------------------
void Set_PWM_to_Servo(int iServo, int iValue)
{
  int NewPWM = iValue + (int8_t)EEPROM.read(iServo);
  NewPWM = map(NewPWM, PWMRES_Min, PWMRES_Max, SERVOMIN, SERVOMAX);

  if (iServo >= 7) { GPIO2SERVO.write(NewPWM); } 
  else if (iServo >= 6) { GPIO4SERVO.write(NewPWM); } 
  else if (iServo >= 5) { GPIO5SERVO.write(NewPWM); } 
  else if (iServo >= 4) { GPIO16SERVO.write(NewPWM); } 
  else if (iServo >= 3) { GPIO15SERVO.write(NewPWM); } 
  else if (iServo >= 2) { GPIO13SERVO.write(NewPWM); } 
  else if (iServo >= 1) { GPIO12SERVO.write(NewPWM); } 
  else if (iServo == 0) { GPIO14SERVO.write(NewPWM); }
}

void Servo_PROGRAM_Run(int iMatrix[][ALLMATRIX], int iSteps)
{
  int INT_TEMP_A, INT_TEMP_B, INT_TEMP_C;

  for (int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++) { 
    int InterTotalTime = iMatrix[MainLoopIndex][ALLMATRIX - 1] / globalSpeedFactor; 
    
    if (InterTotalTime < BASEDELAYTIME) {
      InterTotalTime = BASEDELAYTIME;
    }

    int InterDelayCounter = InterTotalTime / BASEDELAYTIME; 

    for (int InterStepLoop = 0; InterStepLoop < InterDelayCounter; InterStepLoop++) { 
      for (int ServoIndex = 0; ServoIndex < ALLSERVOS; ServoIndex++) { 
        INT_TEMP_A = Running_Servo_POS[ServoIndex]; 
        INT_TEMP_B = iMatrix[MainLoopIndex][ServoIndex]; 

        if (INT_TEMP_A == INT_TEMP_B) { 
          INT_TEMP_C = INT_TEMP_B;
        } else if (INT_TEMP_A > INT_TEMP_B) { 
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_A - INT_TEMP_B); 
          if (INT_TEMP_A - INT_TEMP_C >= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A - INT_TEMP_C);
          }
        } else if (INT_TEMP_A < INT_TEMP_B) { 
          INT_TEMP_C =  map(BASEDELAYTIME * InterStepLoop, 0, InterTotalTime, 0, INT_TEMP_B - INT_TEMP_A); 
          if (INT_TEMP_A + INT_TEMP_C <= INT_TEMP_B) {
            Set_PWM_to_Servo(ServoIndex, INT_TEMP_A + INT_TEMP_C);
          }
        }
      }
      delay(BASEDELAYTIME);
    }
    for (int Index = 0; Index < ALLMATRIX; Index++) {
      Running_Servo_POS[Index] = iMatrix[MainLoopIndex][Index];
    }
  }
}

// --------------------------------------------------------------------------------
// 웹서버 동작 라우팅
// --------------------------------------------------------------------------------
void handleTest() 
{
  String s_id = server.arg("servo");
  if (s_id != "") {
    int id = s_id.toInt();
    int currentPos = Running_Servo_POS[id];
    int tempMatrix[1][ALLMATRIX];
    
    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Running_Servo_POS[i];
    tempMatrix[0][id] = 45;
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);

    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Running_Servo_POS[i];
    tempMatrix[0][id] = 135;
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);

    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Running_Servo_POS[i];
    tempMatrix[0][id] = currentPos;
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);
  }
  server.send(200, "text/plain", "Done");
}

void handlePose() 
{
  String type = server.arg("type");
  
  if (type == "center") {
    // 전체 90도 (Servo_Act_0 사용)
    int tempMatrix[1][ALLMATRIX];
    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Servo_Act_0[i]; 
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);
  } 
  else if (type == "standby") {
    // 기본 자세 (모든 관절 90도)
    int tempMatrix[1][ALLMATRIX];
    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = 90; 
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);
  } 
  else if (type == "sleep") {
    Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
    Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
  }
  
  debugPrintAngles("자세 변경 완료 후");
  server.send(200, "text/plain", "Done");
}

// --------------------------------------------------------------------------------
// 웹페이지 UI 생성 (HTML + 자바스크립트)
// --------------------------------------------------------------------------------
void handleIndex()
{
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "<title>스파이더 관절 테스트</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body { margin: 0px; font-family: 'Malgun Gothic', sans-serif; background-color: #f4f4f9; color: #333; padding-bottom: 20px;}";
  content += "h2 { text-align: center; margin-top: 20px; color: #2c3e50; }";
  content += "p { text-align: center; font-size: 14px; color: #7f8c8d; margin-bottom: 20px; }";
  content += "table { width: 100%; height: 75%; border-collapse: separate; border-spacing: 10px; }";
  content += "td { text-align: center; width: 50%; }";
  content += "button { width: 100%; height: 100%; font-size: 18px; font-weight: bold; color: #444; ";
  content += "background: #BFDFFF; border: 2px solid #99c2ff; border-radius: 12px; transition: all 0.3s ease; }";
  content += ".global-btn { height: 60px; }";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  
  content += "<h2>스파이더 관절 테스트</h2>";
  content += "<p>버튼을 눌러 8개의 관절이 어떻게 움직이는지 관찰해보세요.</p>";
  
  content += "<table>";
  
  for(int i = 0; i < 4; i++) {
    int servo1 = i * 2;       
    int servo2 = i * 2 + 1;   
    
    String btnId1 = "btn_" + String(servo1);
    String btnName1 = String(servo1 + 1) + "번 관절";
    String btnId2 = "btn_" + String(servo2);
    String btnName2 = String(servo2 + 1) + "번 관절";

    content += "<tr height=20%>";
    content += "<td><button id='" + btnId1 + "' type='button' style='background:#BFDFFF;' onclick=\"testJoint(" + String(servo1) + ", '" + btnId1 + "', '" + btnName1 + "')\">" + btnName1 + "</button></td>";
    content += "<td><button id='" + btnId2 + "' type='button' style='background:#BFFFCF;' onclick=\"testJoint(" + String(servo2) + ", '" + btnId2 + "', '" + btnName2 + "')\">" + btnName2 + "</button></td>";
    content += "</tr>";
  }
  
  content += "<tr><td colspan=2 style='height: 10px;'></td></tr>"; 
  content += "<tr>";
  content += "<td><button id='btn_center' class='global-btn' onclick=\"setPose('center', 'btn_center', '전체 90도')\" style='background:#FFE599; border-color:#f5d061;'>전체 90도</button></td>";
  content += "<td><button id='btn_standby' class='global-btn' onclick=\"setPose('standby', 'btn_standby', '기본 자세')\" style='background:#E0E0E0; border-color:#bdbdbd;'>기본 자세</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td colspan=2><button id='btn_sleep' class='global-btn' onclick=\"setPose('sleep', 'btn_sleep', 'Sleep 모드')\" style='background:#FFBFBF; border-color:#ff9999;'>Sleep 모드</button></td>";
  content += "</tr>";

  content += "</table>";
  content += "</body>";
  
  content += "<script>";
  content += "function triggerAction(url, btnId, btnName) {";
  content += "  var btn = document.getElementById(btnId);";
  content += "  var originalBg = btn.style.background;"; 
  content += "  btn.style.background = '#FF9999';";
  content += "  btn.style.color = '#FFFFFF';";
  content += "  btn.innerHTML = '동작 중...';";
  content += "  btn.disabled = true;";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (this.readyState == 4 && this.status == 200) {";
  content += "      btn.style.background = originalBg;";
  content += "      btn.style.color = '#444';";
  content += "      btn.innerHTML = btnName;";
  content += "      btn.disabled = false;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", url, true);";
  content += "  xhttp.send();";
  content += "}";
  
  content += "function testJoint(servoId, btnId, btnName) {";
  content += "  triggerAction('test?servo=' + servoId, btnId, btnName);";
  content += "}";

  content += "function setPose(type, btnId, btnName) {";
  content += "  triggerAction('pose?type=' + type, btnId, btnName);";
  content += "}";
  
  content += "</script>";
  content += "</html>";

  server.send(200, "text/html", content);
}

// --------------------------------------------------------------------------------
// 셋업 및 루프
// --------------------------------------------------------------------------------
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Spider Robot Test Mode Start!");

  GPIO14SERVO.attach(15, SERVOMIN, SERVOMAX);
  GPIO12SERVO.attach(13, SERVOMIN, SERVOMAX);
  GPIO13SERVO.attach(12, SERVOMIN, SERVOMAX);
  GPIO15SERVO.attach(14, SERVOMIN, SERVOMAX);
  GPIO16SERVO.attach(2, SERVOMIN, SERVOMAX);
  GPIO5SERVO.attach(4, SERVOMIN, SERVOMAX);
  GPIO4SERVO.attach(5, SERVOMIN, SERVOMAX);
  GPIO2SERVO.attach(16, SERVOMIN, SERVOMAX);

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (int i = 0; i < AP_NameString.length(); i++) {
    AP_NameChar[i] = AP_NameString.charAt(i);
  }
  WiFi.softAP(AP_NameChar, WiFiAPPSK);
  
  EEPROM.begin(512);
  delay(10);

  // 무조건 모든 관절 90도로 부팅 및 유지
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, 90);
    Running_Servo_POS[i] = 90;
    delay(30);
  }
  
  debugPrintAngles("부팅 직후 초기화 (기본 자세 - 90도)");

  server.on("/", HTTP_GET, handleIndex);
  server.on("/test", HTTP_GET, handleTest); 
  server.on("/pose", HTTP_GET, handlePose); 
  
  server.begin();
}

void loop(void)
{
  server.handleClient();
}