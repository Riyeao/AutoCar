#include <MsTimer2.h>
#include<Servo.h>
#include <stdlib.h>
#define LENCODER_A 2 // 左编码器A相
#define LENCODER_B 4 // 左编码器B相
#define RENCODER_A 3 // 右编码器A相
#define RENCODER_B 5 // 右编码器B相
#define PWMR 10      // 左轮控制
#define INT3 32
#define INT4 34
#define PWML 9 // 右轮控制
#define INT1 33
#define INT2 35
#define L1 22 // 左红外
#define L2 24
#define L3 26
#define R1 23 // 右红外
#define R2 25
#define R3 27
#define M 28                                // 中红外
#define SA 11
#define SB 12
#define SC 13
Servo servoA, servoB, servoC;
int TARGETL = 30;                            // 设定左轮转速
int TARGETR = 30;                            // 设定右轮转速
#define PERIOD 20                            // 测速周期为20ms
float Kp = 5, Ti = 140, Td = 80, T = PERIOD; // PID控制的三个参数
float V_target_L, V_target_R;                // 小车左轮、右轮在红外传感器信号变化下的期望速度
float q0 = Kp * (1 + T / Ti + Td / T);
float q1 = -Kp * (1 + 2 * Td / T);
float q2 = Kp * Td / T;
float uL, uR;                           // 分别输出左轮，右轮PWM波
float vL, vR;                           // 对应左轮，右轮速度
volatile long LencoderVal, RencoderVal; // 对应左编码器、右编码器取值
float LeI, LeII = 0, LeIII = 0;         // 左轮此时刻，上一时刻，上上时刻与给定值的误差
float ReI, ReII = 0, ReIII = 0;         // 右轮此时刻，上一时刻，上上时刻与给定值的误差
char parameter, num[5] = {};
int i = -1;
float Num = 0;
void Car_speed_in_PID(void)
{
    if (digitalRead(M) == LOW)
    {
        V_target_L = TARGETL;
        V_target_R = TARGETR;
    }
    if (digitalRead(L1) == HIGH) // 左边第一个红外传感器偏离，往右转
    {
        V_target_L = 0.9 * TARGETL;
        V_target_R = 0.5 * TARGETR;
    }
    if (digitalRead(R1) == HIGH) // 右边第一个红外传感器偏离，往左转
    {
        V_target_L = 0.5 * TARGETL;
        V_target_R = 0.9 * TARGETR;
    }
    if (digitalRead(L2) == HIGH) // 左边第二个红外传感器偏离，往右转
    {
        V_target_L = 0.75 * TARGETL;
        V_target_R = 0.15 * TARGETR;
    }
    if (digitalRead(R2) == HIGH) // 右边第二个红外传感器偏离，往左转
    {
        V_target_L = 0.15 * TARGETL;
        V_target_R = 0.75 * TARGETR;
    }
    if (digitalRead(L3) == HIGH) // 左边第三个红外传感器偏离，往右转
    {
        V_target_L = 0.65 * TARGETL;
        V_target_R = 0;
    }
    if (digitalRead(R3) == HIGH) // 右边第三个红外传感器偏离，往左转
    {
        V_target_L = 0;
        V_target_R = 0.65 * TARGETR;
    }
    if (digitalRead(L1) == HIGH && digitalRead(L2) == HIGH && digitalRead(L3) == HIGH && digitalRead(M) == HIGH && digitalRead(R1) == HIGH && digitalRead(R2) == HIGH && digitalRead(R3) == HIGH)
    {
        V_target_L = 0;
        V_target_R = 0;
    }
}
void GetEncoderL(void) // 左编码器在一个周期内的取值
{
    if (digitalRead(LENCODER_A) == LOW)
    {
        if (digitalRead(LENCODER_B) == LOW)
        {
            LencoderVal--;
        }
        else
        {
            LencoderVal++;
        }
    }
    else
    {
        if (digitalRead(LENCODER_B) == HIGH)
        {
            LencoderVal--;
        }
        else
        {
            LencoderVal++;
        }
    }
}

void GetEncoderR(void) // 右编码器在一个周期内的取值
{
    if (digitalRead(RENCODER_A) == LOW)
    {
        if (digitalRead(RENCODER_B) == LOW)
        {
            RencoderVal--;
        }
        else
        {
            RencoderVal++;
        }
    }
    else
    {
        if (digitalRead(RENCODER_B) == HIGH)
        {
            RencoderVal--;
        }
        else
        {
            RencoderVal++;
        }
    }
}
void CONTROL() // 小车转速增量式PID控制
{
    Car_speed_in_PID();
    vL = (LencoderVal / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD); // 测左轮转速
    vR = (RencoderVal / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD); // 测右轮转速
    LencoderVal = 0;                                             // 两编码器值清零，以便下个周期测速
    RencoderVal = 0;
    LeI = V_target_L - vL;                       // 左轮转速误差
    ReI = V_target_R - vR;                       // 右轮转速误差
    uL = uL + q0 * LeI + q1 * LeII + q2 * LeIII; // 左轮PID控制
    uR = uR + q0 * ReI + q1 * ReII + q2 * ReIII; // 右轮PID控制
    LeIII = LeII;                                // 左轮误差迭代
    LeII = LeI;
    ReIII = ReII; // 右轮误差迭代
    ReII = ReI;
    if (uL > 255) // 速度在[-255,255]之内
    {
        uL = 255;
    }
    if (uL < -255)
    {
        uL = -255;
    }
    if (uR > 255)
    {
        uR = 255;
    }
    if (uR < -255)
    {
        uR = -255;
    }
    uL = (int)uL;
    uR = (int)uR;
    if (uL > 0)
    {
        digitalWrite(INT1, LOW);
        digitalWrite(INT2, HIGH);
        analogWrite(PWML, uL);
    }
    else
    {
        digitalWrite(INT1, HIGH);
        digitalWrite(INT2, LOW);
        analogWrite(PWML, abs(uL));
    }
    if (uR > 0)
    {
        digitalWrite(INT3, LOW);
        digitalWrite(INT4, HIGH);
        analogWrite(PWMR, uR);
    }
    else
    {
        digitalWrite(INT3, HIGH);
        digitalWrite(INT4, LOW);
        analogWrite(PWMR, abs(uR));
    }
}
void setup()
{
    TCCR1B = TCCR1B & B11111000 | B00000001;
    servoA.attach(SA);
    servoB.attach(SB);
    servoC.attach(SC);
    pinMode(SA, OUTPUT);
    pinMode(SB, OUTPUT);
    pinMode(SC, OUTPUT);
    pinMode(L1, INPUT); // 红外传感器信号输入
    pinMode(L2, INPUT);
    pinMode(L3, INPUT);
    pinMode(R1, INPUT);
    pinMode(R2, INPUT);
    pinMode(R3, INPUT);
    pinMode(M, INPUT);
    pinMode(LENCODER_A, INPUT); // 左轮、右轮编码器值输入
    pinMode(LENCODER_B, INPUT);
    pinMode(RENCODER_A, INPUT);
    pinMode(RENCODER_B, INPUT);
    pinMode(PWML, OUTPUT); // 左轮PWM信号输出以输出小车左轮转速
    pinMode(INT1, OUTPUT);
    pinMode(INT2, OUTPUT);
    pinMode(PWMR, OUTPUT); // 右轮PWM信号输出以输出小车右轮转速
    pinMode(INT3, OUTPUT);
    pinMode(INT4, OUTPUT);
    attachInterrupt(0, GetEncoderL, CHANGE);
    attachInterrupt(1, GetEncoderR, CHANGE);
    Serial.begin(9600);
    MsTimer2::set(PERIOD, CONTROL);
    MsTimer2::start();
}

void loop()
{
    while(Serial.available())
    {
        if(i==-1)
            parameter = (char)Serial.read();
        else
            num[i] = (char)Serial.read();
        i++;
    }
    i = -1;
    Num = atof(num);
    if(parameter=='l')
        TARGETL = Num;
    else if(parameter=='r')
        TARGETR = Num;
    else if(parameter=='p')
        Kp = Num;
    else if(parameter=='i')
        Ti = Num;
    else if(parameter=='d')
        Td = Num;
    Serial.print("vL:");
    Serial.print(vL);
    Serial.print("/n/r");
    Serial.print("vR:");
    Serial.print(vR);
    servoA.write(0);
    servoC.write(60);
    servoA.write(90);
    servoA.write(0);
    servoC.write(90);
    servoA.write(90);
}
