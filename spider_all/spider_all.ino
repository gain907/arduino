// ==========================================
// 스파이더 로봇 교육용 3단계 : 최종 마스터 펌웨어 (한글화 완)
// ==========================================

#include <Servo.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

String FW_Version = "Spider Robot";
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
// 관절 각도 시리얼 모니터 출력 함수 (디버깅용)
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
// 동작 데이터 배열 (모든 모션 1~15)
// =========================================================
int Servo_Act_0[] = { 135, 45, 135, 45, 45, 135, 45, 135, 500 }; 
int Servo_Act_1[] = { 135, 45, 135, 45, 45, 135, 45, 135, 500 };

int Servo_Prg_1_Step = 2;
int Servo_Prg_1[][ALLMATRIX] = {
  { 90,  90,  90,  90,  90,  90,  90,  90,  500 }, 
  { 70,  90,  90, 110, 110,  90,  90,  70,  500 }, 
};

int Servo_Prg_2_Step = 11;
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

int Servo_Prg_3_Step = 11;
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

int Servo_Prg_4_Step = 11;
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

int Servo_Prg_5_Step = 11;
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

int Servo_Prg_6_Step = 8;
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

int Servo_Prg_7_Step = 8;
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

int Servo_Prg_8_Step = 1;
int Servo_Prg_8[][ALLMATRIX] = {
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, 
};

int Servo_Prg_9_Step = 7;
int Servo_Prg_9[][ALLMATRIX] = {
  {   70,  90,  90,  90,  90,  90,  90,  90,  400}, 
  {  170,  90,  90,  90,  90,  90,  90,  90,  400}, 
  {  170, 130,  90,  90,  90,  90,  90,  90,  400}, 
  {  170,  50,  90,  90,  90,  90,  90,  90,  400}, 
  {  170, 130,  90,  90,  90,  90,  90,  90,  400}, 
  {  170,  90,  90,  90,  90,  90,  90,  90,  400}, 
  {   70,  90,  90,  90,  90,  90,  90,  90,  400}, 
};

int Servo_Prg_10_Step = 11;
int Servo_Prg_10[][ALLMATRIX] = {
  {  120,  90,  90, 110,  60,  90,  90,  70,  500  }, 
  {  120,  70,  70, 110,  60,  70,  70,  70,  500  }, 
  {  120, 110, 110, 110,  60, 110, 110,  70,  500  }, 
  {  120,  70,  70, 110,  60,  70,  70,  70,  500  }, 
  {  120, 110, 110, 110,  60, 110, 110,  70,  500  }, 
  {   70,  90,  90,  70, 110,  90,  90, 110,  500  }, 
  {   70,  70,  70,  70, 110,  70,  70, 110,  500  }, 
  {   70, 110, 110,  70, 110, 110, 110, 110,  500  }, 
  {   70,  70,  70,  70, 110,  70,  70, 110,  500  }, 
  {   70, 110, 110,  70, 110, 110, 110, 110,  500  }, 
  {   70,  90,  90,  70, 110,  90,  90, 110,  500  }  
};

int Servo_Prg_11_Step = 11;
int Servo_Prg_11[][ALLMATRIX] = {
  {   70,  90,  90, 110, 110,  90,  90,  70,  500  }, 
  {  100,  90,  90,  80,  80,  90,  90, 100,  600  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  700  }, 
  {  100,  90,  90,  80,  80,  90,  90, 100,  800  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  900  }, 
  {  100,  90,  90,  80,  80,  90,  90, 100, 1500  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70, 2000  }, 
  {  135,  90,  90,  45,  45,  90,  90, 135,  200  }, 
  {   70,  90,  90,  45,  60,  90,  90, 135,  800  }, 
  {   70,  90,  90,  45, 110,  90,  90, 135,  800  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  800  }  
};

int Servo_Prg_12_Step = 2;
int Servo_Prg_12[][ALLMATRIX] = {
  {   30,  90,  90, 150, 150,  90,  90,  30,  500  }, 
  {   30,  45, 135, 150, 150, 135,  45,  30,  500  }, 
};

int Servo_Prg_13_Step = 10;
int Servo_Prg_13[][ALLMATRIX] = {
  {   90,  90,  90,  90,  90,  90,  90,  90,  400  }, 
  {   50,  90,  90,  90,  90,  90,  90,  90,  400  }, 
  {   90,  90,  90, 130,  90,  90,  90,  90,  400  }, 
  {   90,  90,  90,  90,  90,  90,  90,  50,  400  }, 
  {   90,  90,  90,  90, 130,  90,  90,  90,  400  }, 
  {   50,  90,  90,  90,  90,  90,  90,  90,  400  }, 
  {   90,  90,  90, 130,  90,  90,  90,  90,  400  }, 
  {   90,  90,  90,  90,  90,  90,  90,  50,  400  }, 
  {   90,  90,  90,  90, 130,  90,  90,  90,  400  }, 
  {   90,  90,  90,  90,  90,  90,  90,  90,  400  }, 
};

int Servo_Prg_14_Step = 9;
int Servo_Prg_14[][ALLMATRIX] = {
  {   70,  45, 135, 110, 110, 135,  45,  70,  400  }, 
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, 
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, 
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, 
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, 
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, 
  {   70,  45, 135, 110,  65, 135,  45, 115,  400  }, 
  {  115,  45, 135,  65, 110, 135,  45,  70,  400  }, 
  {   75,  45, 135, 105, 110, 135,  45,  70,  400  }, 
};

int Servo_Prg_15_Step = 10;
int Servo_Prg_15[][ALLMATRIX] = {
  {   70,  45,  45, 110, 110, 135, 135,  70,  400  }, 
  {  110,  45,  45,  60,  70, 135, 135,  70,  400  }, 
  {   70,  45,  45, 110, 110, 135, 135,  70,  400  }, 
  {  110,  45,  45, 110,  70, 135, 135, 120,  400  }, 
  {   70,  45,  45, 110, 110, 135, 135,  70,  400  }, 
  {  110,  45,  45,  60,  70, 135, 135,  70,  400  }, 
  {   70,  45,  45, 110, 110, 135, 135,  70,  400  }, 
  {  110,  45,  45, 110,  70, 135, 135, 120,  400  }, 
  {   70,  45,  45, 110, 110, 135, 135,  70,  400  }, 
  {   70,  90,  90, 110, 110,  90,  90,  70,  400  }, 
};

// --------------------------------------------------------------------------------
// 서보모터 제어 엔진
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
// 웹서버 라우팅
// --------------------------------------------------------------------------------
void handleController()
{
  String pm = server.arg("pm");
  String speed = server.arg("speed"); 

  if (speed != "") {
    globalSpeedFactor = speed.toFloat();
    server.send(200, "text/html", "Done");
    return;
  }

  if (pm != "") {
    int action = pm.toInt();
    int tempMatrix[1][ALLMATRIX];
    
    switch (action) {
      case 1: 
        // 기본 자세 (모든 관절 90도)
        for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = 90; 
        tempMatrix[0][ALLSERVOS] = 500;
        Servo_PROGRAM_Run(tempMatrix, 1);
        break;
      case 2: Servo_PROGRAM_Run(Servo_Prg_2, Servo_Prg_2_Step); break;
      case 3: Servo_PROGRAM_Run(Servo_Prg_3, Servo_Prg_3_Step); break;
      case 4: Servo_PROGRAM_Run(Servo_Prg_4, Servo_Prg_4_Step); break;
      case 5: Servo_PROGRAM_Run(Servo_Prg_5, Servo_Prg_5_Step); break;
      case 6: Servo_PROGRAM_Run(Servo_Prg_6, Servo_Prg_6_Step); break;
      case 7: Servo_PROGRAM_Run(Servo_Prg_7, Servo_Prg_7_Step); break;
      case 8: Servo_PROGRAM_Run(Servo_Prg_8, Servo_Prg_8_Step); break;
      case 9: 
        Servo_PROGRAM_Run(Servo_Prg_9, Servo_Prg_9_Step);
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        break;
      case 10: Servo_PROGRAM_Run(Servo_Prg_10, Servo_Prg_10_Step); break;
      case 11: Servo_PROGRAM_Run(Servo_Prg_11, Servo_Prg_11_Step); break;
      case 12: 
        // Sleep 모드
        Servo_PROGRAM_Run(Servo_Prg_1, Servo_Prg_1_Step);
        Servo_PROGRAM_Run(Servo_Prg_12, Servo_Prg_12_Step);
        break;
      case 13: Servo_PROGRAM_Run(Servo_Prg_13, Servo_Prg_13_Step); break;
      case 14: Servo_PROGRAM_Run(Servo_Prg_14, Servo_Prg_14_Step); break;
      case 15: Servo_PROGRAM_Run(Servo_Prg_15, Servo_Prg_15_Step); break;
      case 99: 
        // 전체 90도 (Servo_Act_0 사용)
        for(int i=0; i<ALLSERVOS; i++) tempMatrix[0][i] = Servo_Act_0[i]; 
        tempMatrix[0][ALLSERVOS] = 500;
        Servo_PROGRAM_Run(tempMatrix, 1);
        break;
    }
    
    debugPrintAngles("동작 " + pm + " 실행 완료 후");
  }
  
  server.send(200, "text/html", "Done");
}

// --------------------------------------------------------------------------------
// 웹페이지 UI 생성 (HTML + 자바스크립트 이벤트)
// --------------------------------------------------------------------------------
void handleIndex()
{
  String content = "";
  content += "<html>";
  content += "<head>";
  content += "<title>Spider Robot</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body { margin: 0px; font-family: 'Malgun Gothic', sans-serif; font-size: 100%; color: #555555; }";
  content += "td { text-align: center; }";
  content += "span { font-size: 70%; color: #777777; font-weight: bold; }";
  content += "button { width: 90%; height: 90%; font-size: 100%; color: #555555; background: #BFDFFF; padding: 5px; border: none; border-radius: 8px; font-weight: bold; transition: 0.3s; }";
  content += ".speed-btn { font-weight: normal; }";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  content += "<br>";
  content += "<table width=100% height=90%>";
  
  // 1열 버튼 모음 (이동)
  content += "<tr height=19%>";
  content += "<td width=33%><button id='btn_6' style='background:#BFFFCF;' onclick=\"controlPm(6, 'btn_6', '좌회전')\">좌회전</button></td>";
  content += "<td width=33%><button id='btn_2' onclick=\"controlPm(2, 'btn_2', '전진')\">전진</button></td>";
  content += "<td width=33%><button id='btn_7' style='background:#BFFFCF;' onclick=\"controlPm(7, 'btn_7', '우회전')\">우회전</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button id='btn_4' onclick=\"controlPm(4, 'btn_4', '좌로 걷기')\">좌로 걷기</button></td>";
  content += "<td><button id='btn_3' onclick=\"controlPm(3, 'btn_3', '후진')\">후진</button></td>";
  content += "<td><button id='btn_5' onclick=\"controlPm(5, 'btn_5', '우로 걷기')\">우로 걷기</button></td>";
  content += "</tr>";
  
  content += "<tr height=5%><td colspan=3><span><br></span></td></tr>";
  
  // 2열 버튼 모음 (자세 및 액션)
  content += "<tr height=20%>";
  content += "<td><button id='btn_99' style='background:#FFE599;' onclick=\"controlPm(99, 'btn_99', '전체 90도')\">전체 90도</button></td>";
  content += "<td><button id='btn_1' style='background:#E0E0E0; border: 2px solid #bdbdbd;' onclick=\"controlPm(1, 'btn_1', '기본 자세')\">기본 자세</button></td>";
  content += "<td><button id='btn_12' style='background:#FFBFBF;' onclick=\"controlPm(12, 'btn_12', '수면 모드')\">수면 모드</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button id='btn_8' style='background:#FFE599;' onclick=\"controlPm(8, 'btn_8', '엎드리기')\">엎드리기</button></td>";
  content += "<td><button id='btn_10' style='background:#FFE599;' onclick=\"controlPm(10, 'btn_10', '파이팅')\">파이팅</button></td>";
  content += "<td><button id='btn_11' style='background:#FFE599;' onclick=\"controlPm(11, 'btn_11', '팔굽혀펴기')\">팔굽혀펴기</button></td>";
  content += "</tr>";
  content += "<tr height=19%>";
  content += "<td><button id='btn_13' style='background:#CFBFFF;' onclick=\"controlPm(13, 'btn_13', '댄스 1')\">댄스 1</button></td>";
  content += "<td><button id='btn_14' style='background:#CFBFFF;' onclick=\"controlPm(14, 'btn_14', '댄스 2')\">댄스 2</button></td>";
  content += "<td><button id='btn_15' style='background:#CFBFFF;' onclick=\"controlPm(15, 'btn_15', '댄스 3')\">댄스 3</button></td>";
  content += "</tr>";
  
  // 3열 버튼 모음 (속도 조절)
  content += "<tr height=5%><td colspan=3><span><br>속도 조절<br></span></td></tr>";
  content += "<tr height=19%>";
  content += "<td><button id='btn_s1' class='speed-btn' style='background:#E0E0E0; border: 3px solid #333; font-weight: bold;' onclick=\"setSpeed(1.0, 'btn_s1')\">보통</button></td>";
  content += "<td><button id='btn_s2' class='speed-btn' style='background:#FFFF99;' onclick=\"setSpeed(1.5, 'btn_s2')\">빠르게</button></td>";
  content += "<td><button id='btn_s3' class='speed-btn' style='background:#FF9999;' onclick=\"setSpeed(2.0, 'btn_s3')\">매우빠르게</button></td>";
  content += "</tr>";

  content += "</table>";
  content += "<span><p align=center>" + FW_Version + "</p></span>";
  content += "</body>";
  
  // 자바스크립트
  content += "<script>";
  content += "function controlPm(id, btnId, btnName) {";
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
  content += "      btn.style.color = '#555555';";
  content += "      btn.innerHTML = btnName;"; 
  content += "      btn.disabled = false;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"controller?pm=\"+id, true);";
  content += "  xhttp.send();";
  content += "}";

  content += "function setSpeed(val, btnId) {";
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.open(\"GET\", \"controller?speed=\"+val, true);";
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
  Serial.println("Spider Robot Final Mode Start!");

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

  // 부팅 시 무조건 모든 관절 90도로 유지
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, 90);
    Running_Servo_POS[i] = 90;
    delay(30);
  }
  
  debugPrintAngles("부팅 직후 초기화 (기본 자세 - 90도)");

  server.on("/", HTTP_GET, handleIndex);
  server.on("/controller", HTTP_GET, handleController);
  server.begin();
}

void loop(void)
{
  server.handleClient();
}