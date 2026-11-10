// ==========================================
// 스파이더 로봇 교육용 2단계 : 움직임 제어 펌웨어 (디버그 포함)
// ==========================================

#include <Servo.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

String FW_Version = "Movement Control Mode";
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
float globalSpeedFactor = 1.0; // 기본 속도 (보통)

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
// 0. 관절 각도 시리얼 모니터 출력 함수 (디버깅용)
// =========================================================
void debugPrintAngles(String label) {
  Serial.println("\n[" + label + "] 현재 모터 각도 상태:");
  for (int i = 0; i < ALLSERVOS; i++) {
    Serial.print("모터 ");
    Serial.print(i);
    Serial.print("번: ");
    Serial.print(Running_Servo_POS[i]);
    Serial.print("도 | ");
    if(i == 3) Serial.println(); // 4개씩 줄바꿈
  }
  Serial.println("\n------------------------------------------------");
}

// =========================================================
// 1. 자세 제어 시퀀스 (전체 90도, Sleep)
// =========================================================
int Servo_Act_0[] = { 135, 45, 135, 45, 45, 135, 45, 135, 500 }; // (전체 90도 모드용)

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

// =========================================================
// 2. 방향 이동 시퀀스 (2~7)
// =========================================================
int Servo_Prg_2_Step = 11; // 전진
int Servo_Prg_2[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   90,  90,  90, 110, 110,  90,  45,  90,  200  }, 
  {   70,  90,  90, 110, 110,  90,  45,  70,  200  }, 
  {   70,  90,  90,  90,  90,  90,  45,  70,  200  }, 
  {   70,  45, 135,  90,  90,  90,  90,  70,  200  }, 
  {   70,  45, 135, 110, 110,  90,  90,  70,  200  }, 
  {   90,  90, 135, 110, 110,  90,  90,  90,  200  }, 
  {   90,  90,  90, 110, 110, 135,  90,  90,  200  }, 
  {   70,  90,  90, 110, 110, 135,  90,  70,  200  }, 
  {   70,  90,  90, 110,  90, 135,  90,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
};

int Servo_Prg_3_Step = 11; // 후진
int Servo_Prg_3[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   90,  45,  90, 110, 110,  90,  90,  90,  200  }, 
  {   70,  45,  90, 110, 110,  90,  90,  70,  200  }, 
  {   70,  45,  90,  90,  90,  90,  90,  70,  200  }, 
  {   70,  90,  90,  90,  90, 135,  45,  70,  200  }, 
  {   70,  90,  90, 110, 110, 135,  45,  70,  200  }, 
  {   90,  90,  90, 110, 110, 135,  90,  90,  200  }, 
  {   90,  90, 135, 110, 110,  90,  90,  90,  200  }, 
  {   70,  90, 135, 110, 110,  90,  90,  70,  200  }, 
  {   70,  90, 135,  90, 110,  90,  90,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
};

int Servo_Prg_4_Step = 11; // 좌로 걷기
int Servo_Prg_4[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   70,  90,  45,  90,  90,  90,  90,  70,  200  }, 
  {   70,  90,  45, 110, 110,  90,  90,  70,  200  }, 
  {   90,  90,  45, 110, 110,  90,  90,  90,  200  }, 
  {   90, 135,  90, 110, 110,  45,  90,  90,  200  }, 
  {   70, 135,  90, 110, 110,  45,  90,  70,  200  }, 
  {   70, 135,  90,  90,  90,  90,  90,  70,  200  }, 
  {   70,  90,  90,  90,  90,  90, 135,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90, 135,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90, 135,  90,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
};

int Servo_Prg_5_Step = 11; // 우로 걷기
int Servo_Prg_5[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   70,  90,  90,  90,  90,  45,  90,  70,  200  }, 
  {   70,  90,  90, 110, 110,  45,  90,  70,  200  }, 
  {   90,  90,  90, 110, 110,  45,  90,  90,  200  }, 
  {   90,  90,  45, 110, 110,  90, 135,  90,  200  }, 
  {   70,  90,  45, 110, 110,  90, 135,  70,  200  }, 
  {   70,  90,  90,  90,  90,  90, 135,  70,  200  }, 
  {   70, 135,  90,  90,  90,  90,  90,  70,  200  }, 
  {   70, 135,  90, 110, 110,  90,  90,  70,  200  }, 
  {   90, 135,  90, 110, 110,  90,  90,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
};

int Servo_Prg_6_Step = 8; // 좌회전
int Servo_Prg_6[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   90,  90,  90, 110, 110,  90,  90,  90,  200  }, 
  {   90, 135,  90, 110, 110,  90, 135,  90,  200  }, 
  {   70, 135,  90, 110, 110,  90, 135,  70,  200  }, 
  {   70, 135,  90,  90,  90,  90, 135,  70,  200  }, 
  {   70, 135, 135,  90,  90, 135, 135,  70,  200  }, 
  {   70, 135, 135, 110, 110, 135, 135,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
};

int Servo_Prg_7_Step = 8; // 우회전
int Servo_Prg_7[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
  {   70,  90,  90,  90,  90,  90,  90,  70,  200  }, 
  {   70,  90,  45,  90,  90,  45,  90,  70,  200  }, 
  {   70,  90,  45, 110, 110,  45,  90,  70,  200  }, 
  {   90,  90,  45, 110, 110,  45,  90,  90,  200  }, 
  {   90,  45,  45, 110, 110,  45,  45,  90,  200  }, 
  {   70,  45,  45, 110, 110,  45,  45,  70,  200  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  200  }, 
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
void handleMove() 
{
  String s_id = server.arg("id");
  if (s_id != "") {
    int actionId = s_id.toInt();
    switch (actionId) {
      case 2: Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step); break;
      case 3: Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step); break;
      case 4: Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step); break;
      case 5: Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step); break;
      case 6: Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step); break;
      case 7: Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step); break;
    }
  }
  
  // ★ 움직임 완료 후 시리얼 모니터에 각도 찍기
  debugPrintAngles("방향 이동 완료 후");
  
  server.send(200, "text/plain", "Done");
}

void handlePose() 
{
  String type = server.arg("type");
  
  if (type == "center") {
    int tempMatrix[1][ALLMATRIX];
    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Servo_Act_0[i]; 
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);
  } 
  else if (type == "standby") {
    int tempMatrix[1][ALLMATRIX];
    for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = 90; 
    tempMatrix[0][ALLSERVOS] = 500;
    Servo_PROGRAM_Run(tempMatrix, 1);
  } 
  else if (type == "sleep") {
    Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
    Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
  }
  
  // ★ 추가됨: 자세 제어 완료 후 시리얼 모니터에 각도 찍기
  debugPrintAngles("자세 변경 완료 후");
  
  server.send(200, "text/plain", "Done");
}

void handleSpeed() 
{
  String val = server.arg("val");
  if (val != "") {
    globalSpeedFactor = val.toFloat();
  }
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
  content += "<title>움직임 제어 모드</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body { margin: 0px; font-family: 'Malgun Gothic', sans-serif; background-color: #f4f4f9; color: #333; padding-bottom: 20px;}";
  content += "h2 { text-align: center; margin-top: 20px; color: #2c3e50; }";
  content += "p { text-align: center; font-size: 14px; color: #7f8c8d; margin-bottom: 20px; }";
  content += "table { width: 100%; height: 80%; border-collapse: separate; border-spacing: 8px; }";
  content += "td { text-align: center; width: 33.3%; height: 60px; }";
  content += "button { width: 100%; height: 100%; font-size: 16px; font-weight: bold; color: #444; ";
  content += "background: #BFDFFF; border: 2px solid #99c2ff; border-radius: 12px; transition: all 0.3s ease; }";
  content += ".global-btn { background: #FFE599; border-color: #f5d061; }";
  content += ".speed-btn { border: none; font-weight: normal; }";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  
  content += "<h2>스파이더 움직임 제어</h2>";
  content += "<p>전후좌우 방향키를 눌러 로봇을 조종해보세요.</p>";
  
  content += "<table>";
  
  // 방향 이동 버튼
  content += "<tr>";
  content += "<td><button id='btn_m6' style='background:#BFFFCF;' onclick=\"moveRobot(6, 'btn_m6', '좌회전')\">좌회전</button></td>";
  content += "<td><button id='btn_m2' onclick=\"moveRobot(2, 'btn_m2', '전진')\">전진</button></td>";
  content += "<td><button id='btn_m7' style='background:#BFFFCF;' onclick=\"moveRobot(7, 'btn_m7', '우회전')\">우회전</button></td>";
  content += "</tr>";
  content += "<tr>";
  content += "<td><button id='btn_m4' onclick=\"moveRobot(4, 'btn_m4', '좌로 걷기')\">좌로 걷기</button></td>";
  content += "<td><button id='btn_m3' onclick=\"moveRobot(3, 'btn_m3', '후진')\">후진</button></td>";
  content += "<td><button id='btn_m5' onclick=\"moveRobot(5, 'btn_m5', '우로 걷기')\">우로 걷기</button></td>";
  content += "</tr>";
  
  // 자세 제어 버튼
  content += "<tr><td colspan=3 style='height: 10px;'></td></tr>"; 
  content += "<tr>";
  content += "<td><button id='btn_center' class='global-btn' onclick=\"setPose('center', 'btn_center', '전체 90도')\">전체 90도</button></td>";
  content += "<td><button id='btn_standby' class='global-btn' style='background:#E0E0E0; border-color:#bdbdbd;' onclick=\"setPose('standby', 'btn_standby', '기본 자세')\">기본 자세</button></td>";
  content += "<td><button id='btn_sleep' class='global-btn' style='background:#FFBFBF; border-color:#ff9999;' onclick=\"setPose('sleep', 'btn_sleep', 'Sleep 모드')\">Sleep</button></td>";
  content += "</tr>";

  // 속도 제어 버튼
  content += "<tr><td colspan=3 style='height: 10px;'><span><br>Speed Control<br></span></td></tr>";
  content += "<tr>";
  content += "<td><button id='btn_s1' class='speed-btn' style='background:#E0E0E0; border: 3px solid #333; font-weight: bold;' onclick=\"setSpeed(1.0, 'btn_s1')\">보통</button></td>";
  content += "<td><button id='btn_s2' class='speed-btn' style='background:#FFFF99;' onclick=\"setSpeed(1.5, 'btn_s2')\">빠르게</button></td>";
  content += "<td><button id='btn_s3' class='speed-btn' style='background:#FF9999;' onclick=\"setSpeed(2.0, 'btn_s3')\">매우빠르게</button></td>";
  content += "</tr>";

  content += "</table>";
  content += "</body>";
  
  // 자바스크립트
  content += "<script>";
  content += "function triggerAction(url, btnId, btnName) {";
  content += "  var btn = document.getElementById(btnId);";
  content += "  var originalBg = btn.style.background;"; 
  content += "  btn.style.background = '#FF9999';";
  content += "  btn.style.color = '#FFFFFF';";
  content += "  btn.innerHTML = '이동 중...';";
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

  content += "function moveRobot(id, btnId, btnName) {";
  content += "  triggerAction('move?id=' + id, btnId, btnName);";
  content += "}";

  content += "function setPose(type, btnId, btnName) {";
  content += "  triggerAction('pose?type=' + type, btnId, btnName);";
  content += "}";

  content += "function setSpeed(val, btnId) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.open(\"GET\", \"speed?val=\"+val, true);";
  content += "  xhttp.send();";
  
  content += "  document.getElementById('btn_s1').style.border = 'none';";
  content += "  document.getElementById('btn_s2').style.border = 'none';";
  content += "  document.getElementById('btn_s3').style.border = 'none';";
  content += "  document.getElementById('btn_s1').style.fontWeight = 'normal';";
  content += "  document.getElementById('btn_s2').style.fontWeight = 'normal';";
  content += "  document.getElementById('btn_s3').style.fontWeight = 'normal';";
  
  content += "  document.getElementById(btnId).style.border = '3px solid #333';";
  content += "  document.getElementById(btnId).style.fontWeight = 'bold';";
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
  Serial.println("Spider Robot Movement Mode Start!");

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

  // 부팅 시 '무조건' 실제 모터에 90도(Stand) 신호를 먼저 쏴줍니다.
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, 90);
    Running_Servo_POS[i] = 90;
    delay(30); // 모터가 움직일 수 있는 시간 확보
  }
  
  // 부팅 직후 시리얼 모니터에 각도 찍기
  debugPrintAngles("부팅 직후 초기화 (Stand 모드)");

  // 웹서버 라우팅
  server.on("/", HTTP_GET, handleIndex);
  server.on("/move", HTTP_GET, handleMove);   
  server.on("/pose", HTTP_GET, handlePose);   
  server.on("/speed", HTTP_GET, handleSpeed); 
  
  server.begin();
}

void loop(void)
{
  server.handleClient();
}