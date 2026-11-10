// ==========================================
// 스파이더 로봇 교육용 1단계 : 관절 테스트 펌웨어
// ==========================================

#include <Servo.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// 펌웨어 버전 및 와이파이 이름
String FW_Version = "Joint Test Mode";
String AP_NameString = "Spider Robot_김석훈"; 
const char WiFiAPPSK[] = "12345678";

// 서보모터 설정
const int ALLMATRIX = 9; 
const int ALLSERVOS = 8; 
const int PWMRES_Min = 1; 
const int PWMRES_Max = 180; 
const int SERVOMIN = 400; 
const int SERVOMAX = 2400; 

ESP8266WebServer server(80);

Servo GPIO14SERVO;
Servo GPIO12SERVO;
Servo GPIO13SERVO;
Servo GPIO15SERVO;
Servo GPIO16SERVO;
Servo GPIO5SERVO;
Servo GPIO4SERVO;
Servo GPIO2SERVO;

// --------------------------------------------------------------------------------
// 서보모터 제어 함수
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

// --------------------------------------------------------------------------------
// 웹서버 요청 처리 (관절 개별 테스트)
// --------------------------------------------------------------------------------
void handleTest() 
{
  String s_id = server.arg("servo");
  if (s_id != "") {
    int id = s_id.toInt();
    
    // 원본 속도에 맞춰 안정적으로 왕복 테스트 (45도 -> 135도 -> 90도)
    Set_PWM_to_Servo(id, 45);
    delay(500); 
    Set_PWM_to_Servo(id, 135);
    delay(500);
    Set_PWM_to_Servo(id, 90);
    delay(500);
  }
  // 동작이 끝나면 클라이언트(웹)에 완료 신호 전송
  server.send(200, "text/plain", "Done");
}

// --------------------------------------------------------------------------------
// 웹서버 요청 처리 (전체 90도 센터 모드)
// --------------------------------------------------------------------------------
void handleCenter() 
{
  // 원본 코드의 초기 로딩 자세(Servo_Act_1)와 동일한 전체 90도 정렬
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, 90);
    delay(50);
  }
  server.send(200, "text/plain", "Done");
}

// --------------------------------------------------------------------------------
// 웹서버 요청 처리 (Sleep 모드)
// --------------------------------------------------------------------------------
void handleSleep() 
{
  // 원본 코드의 Sleep 자세 (Servo_Prg_12 참고)
  int sleepPos[8] = {30, 90, 90, 150, 150, 90, 90, 30};
  
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, sleepPos[i]);
    delay(50);
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
  content += "<title>스파이더 관절 테스트</title>";
  content += "<meta charset=UTF-8>";
  content += "<meta name=viewport content=width=device-width>";
  content += "<style type=text/css>";
  content += "body { margin: 0px; font-family: 'Malgun Gothic', sans-serif; background-color: #f4f4f9; color: #333; padding-bottom: 20px;}";
  content += "h2 { text-align: center; margin-top: 20px; color: #2c3e50; }";
  content += "p { text-align: center; font-size: 14px; color: #7f8c8d; margin-bottom: 20px; }";
  content += "table { width: 100%; height: 75%; border-collapse: separate; border-spacing: 10px; }";
  content += "td { text-align: center; width: 50%; }";
  
  // 기본 버튼 스타일
  content += "button { width: 100%; height: 100%; font-size: 18px; font-weight: bold; color: #444; ";
  content += "background: #BFDFFF; border: 2px solid #99c2ff; border-radius: 12px; transition: all 0.3s ease; }";
  content += "</style>";
  content += "</head>";
  content += "<body>";
  
  content += "<h2>스파이더 관절 테스트</h2>";
  content += "<p>버튼을 눌러 8개의 관절이 어떻게 움직이는지 관찰해보세요.</p>";
  
  content += "<table>";
  
  // 8개의 개별 관절 버튼 생성 (2칸씩 4줄)
  for(int i = 0; i < 4; i++) {
    int servo1 = i * 2;       // 0, 2, 4, 6
    int servo2 = i * 2 + 1;   // 1, 3, 5, 7
    
    String btnId1 = "btn_" + String(servo1);
    String btnName1 = String(servo1 + 1) + "번 관절";
    String btnId2 = "btn_" + String(servo2);
    String btnName2 = String(servo2 + 1) + "번 관절";

    content += "<tr height=20%>";
    content += "<td><button id='" + btnId1 + "' type='button' style='background:#BFDFFF;' onclick=\"testJoint(" + String(servo1) + ", '" + btnId1 + "', '" + btnName1 + "')\">" + btnName1 + "</button></td>";
    content += "<td><button id='" + btnId2 + "' type='button' style='background:#BFFFCF;' onclick=\"testJoint(" + String(servo2) + ", '" + btnId2 + "', '" + btnName2 + "')\">" + btnName2 + "</button></td>";
    content += "</tr>";
  }
  
  // 전체 제어 모드 버튼 2개 추가 (전체 90도, Sleep 모드)
  content += "<tr><td colspan=2 style='height: 15px;'></td></tr>"; // 여백
  content += "<tr height=20%>";
  content += "<td><button id='btn_center' type='button' style='background:#FFE599; border-color:#f5d061;' onclick=\"testGlobal('center', 'btn_center', '전체 90도')\">전체 90도</button></td>";
  content += "<td><button id='btn_sleep' type='button' style='background:#FFBFBF; border-color:#ff9999;' onclick=\"testGlobal('sleep', 'btn_sleep', 'Sleep 모드')\">Sleep 모드</button></td>";
  content += "</tr>";

  content += "</table>";
  content += "</body>";
  
  // 자바스크립트: 버튼 클릭 시 시각적 이벤트 처리
  content += "<script>";
  
  // 개별 관절 테스트용 자바스크립트
  content += "function testJoint(servoId, btnId, btnName) {";
  content += "  var btn = document.getElementById(btnId);";
  content += "  var originalBg = btn.style.background;"; // 원래 색상 기억
  
  // 1. 클릭 즉시 버튼 상태 변경 (빨간색, 텍스트 변경, 터치 잠금)
  content += "  btn.style.background = '#FF9999';";
  content += "  btn.style.color = '#FFFFFF';";
  content += "  btn.innerHTML = '움직이는 중...';";
  content += "  btn.disabled = true;";
  
  // 2. 아두이노 서버로 모터 제어 명령 전송
  content += "  var xhttp = new XMLHttpRequest();";
  content += "  xhttp.onreadystatechange = function() {";
  content += "    if (this.readyState == 4 && this.status == 200) {";
  // 3. 서버 동작 완료(Done) 응답 시 버튼 원상복구
  content += "      btn.style.background = originalBg;";
  content += "      btn.style.color = '#444';";
  content += "      btn.innerHTML = btnName;";
  content += "      btn.disabled = false;";
  content += "    }";
  content += "  };";
  content += "  xhttp.open(\"GET\", \"test?servo=\"+servoId, true);";
  content += "  xhttp.send();";
  content += "}";

  // 전체 제어(Center, Sleep)용 자바스크립트
  content += "function testGlobal(action, btnId, btnName) {";
  content += "  var btn = document.getElementById(btnId);";
  content += "  var originalBg = btn.style.background;"; 
  content += "  btn.style.background = '#FF9999';";
  content += "  btn.style.color = '#FFFFFF';";
  content += "  btn.innerHTML = '움직이는 중...';";
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
  content += "  xhttp.open(\"GET\", \"/\" + action, true);";
  content += "  xhttp.send();";
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

  // 모터 핀 연결
  GPIO14SERVO.attach(15, SERVOMIN, SERVOMAX);
  GPIO12SERVO.attach(13, SERVOMIN, SERVOMAX);
  GPIO13SERVO.attach(12, SERVOMIN, SERVOMAX);
  GPIO15SERVO.attach(14, SERVOMIN, SERVOMAX);
  GPIO16SERVO.attach(2, SERVOMIN, SERVOMAX);
  GPIO5SERVO.attach(4, SERVOMIN, SERVOMAX);
  GPIO4SERVO.attach(5, SERVOMIN, SERVOMAX);
  GPIO2SERVO.attach(16, SERVOMIN, SERVOMAX);

  // 와이파이 AP 모드 실행
  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);
  for (int i = 0; i < AP_NameString.length(); i++) {
    AP_NameChar[i] = AP_NameString.charAt(i);
  }
  WiFi.softAP(AP_NameChar, WiFiAPPSK);
  
  EEPROM.begin(512);
  delay(10);

  // 로봇 켜질 때 모든 관절을 중앙(90도)으로 초기화
  for (int i = 0; i < ALLSERVOS; i++) {
    Set_PWM_to_Servo(i, 90);
    delay(50);
  }

  // 웹서버 주소 라우팅 연결
  server.on("/", HTTP_GET, handleIndex);
  server.on("/test", HTTP_GET, handleTest); 
  server.on("/center", HTTP_GET, handleCenter); 
  server.on("/sleep", HTTP_GET, handleSleep);   
  
  server.begin();
}

void loop(void)
{
  server.handleClient();
}