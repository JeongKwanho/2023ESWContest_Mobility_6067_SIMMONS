#include "mbed.h"
#include "QEncoder2.h"
#include "MD30C.h"
#include <cstdint>
#include <cmath>
#include <utility>
#include "math.h"


#define PwmA PB_7
#define DIRAA PC_9
#define DIRAB PB_9
#define MOTORA_PINA D6
#define MOTORA_PINB D5

#define PwmB PA_15
#define DIRBA PA_13
#define DIRBB PA_14
#define MOTORB_PINA D1 //원래 D9
#define MOTORB_PINB D0 //원래 D7

#define PwmC PA_11
#define DIRCA PA_12
#define DIRCB PB_12
#define MOTORC_PINA D11
#define MOTORC_PINB D12

#define PwmD PB_2
#define DIRDA PB_15
#define DIRDB PB_1
#define MOTORD_PINA D3
#define MOTORD_PINB D4

#define unity_TX PC_12
#define unity_RX PD_2

#define ALPHA 0.9
#define WRITING_TIME_MOTOR 10

///////////////////////////////communication/////////////
#define DATA_UPDATE_RATE 80
#define MAIN_LOOP 60
#define PRINT_INTERVAL 200
#define SBUF_SIZE 400


/////////////////////parameter of motor control////////////
PwmOut pwmA(PwmA);
DigitalOut dirAA(DIRAA);
DigitalOut dirAB(DIRAB);

PwmOut pwmB(PwmB);
DigitalOut dirBA(DIRBA);
DigitalOut dirBB(DIRBB);


PwmOut pwmC(PwmC);
DigitalOut dirCA(DIRCA);
DigitalOut dirCB(DIRCB);


PwmOut pwmD(PwmD);
DigitalOut dirDA(DIRDA);
DigitalOut dirDB(DIRDB);

QEncoder encoA(MOTORA_PINA,MOTORA_PINB,ALPHA,WRITING_TIME_MOTOR,'R');
QEncoder encoB(MOTORB_PINA,MOTORB_PINB,ALPHA,WRITING_TIME_MOTOR,'R');
QEncoder encoC(MOTORC_PINA,MOTORC_PINB,ALPHA,WRITING_TIME_MOTOR,'L');
QEncoder encoD(MOTORD_PINA,MOTORD_PINB,ALPHA,WRITING_TIME_MOTOR,'L');

///////////////////////parameter of communication//////////////////
RawSerial pc(USBTX, USBRX, 115200);
RawSerial Unity_Co(unity_TX, unity_RX, 115200);

Thread DataRead_thread(osPriorityHigh);  
Thread Print_thread(osPriorityNormal); 

volatile float PC_data[100];
volatile float UN_data[50];
 // 데이터 저장 몇개나 받을지
bool gotPacket = false;
bool gotUnity = false;

void DataRead_loop();
void PcParser();
void UnityParser();

void PRINT_thread_loop();
int my_strcmp(char* str1, char* str2);
float Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle);
float reverse_Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle);
float six1_Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle);

float Motor_Linear_Regression(float input);
float Motor_Linear_Regression2(float input);

float angle = 0;
float output = 0;

float A_input;
float B_input;
float C_input;
float D_input;
float pitch_input;

float A_current = 0;
float B_current = 0;
float C_current = 0;
float D_current = 0;

float Final_A_input;
float Final_B_input;
float Final_C_input;
float Final_D_input;

void control_motor_vel();

volatile float motorA_rpm, motorA_rate;
volatile float motorB_rpm, motorB_rate;
volatile float motorC_rpm, motorC_rate;
volatile float motorD_rpm, motorD_rate;

int main()
{
    uint16_t Now, Work;

    osThreadSetPriority(osThreadGetId(),osPriorityRealtime7);

    Unity_Co.attach(&UnityParser);
    pc.attach(&PcParser); // uart 인터럽트 콜백
    DataRead_thread.start(&DataRead_loop); // 파싱 데이터 읽어오기 쓰레드
    Print_thread.start(&PRINT_thread_loop);


    pwmA.period_us(66); // 15kHz
    pwmB.period_us(66); // 15kHz
    pwmC.period_us(66); // 15kHz    
    pwmD.period_us(66); // 15kHz


    encoA.init();
    encoB.init();
    encoC.init();
    encoD.init();

    while (true)
    {
        Now=rtos::Kernel::get_ms_count();
        
        float A_angle_Error;
        float B_angle_Error;
        float C_angle_Error;
        float D_angle_Error;

        A_current = (A_current + encoA.CalRATE()*(Work-Now)/1000);
        B_current = (B_current + encoB.CalRATE()*(Work-Now)/1000);
        C_current = (C_current + encoC.CalRATE()*(Work-Now)/1000);
        D_current = (D_current + encoD.CalRATE()*(Work-Now)/1000);

        if(A_current >= 150)
        {
            A_current = 150;
        }
        else if(A_current <= -150)
        {
            A_current = -150;
        }

        if(B_current >= 150)
        {
            B_current = 150;
        }
        else if(B_current <= -150)
        {
            B_current = -150;
        }

        if(C_current >= 100)
        {
            C_current = 100;
        }
        else if(C_current <= -100)
        {
            C_current = -100;
        }

        if(D_current >= 150)
        {
            D_current = 150;
        }
        else if(D_current <= -150)
        {
            D_current = -150;
        }

        A_angle_Error = Motor_Linear_Regression(A_input) + A_current;
        B_angle_Error = Motor_Linear_Regression(B_input) - B_current;
        C_angle_Error = Motor_Linear_Regression2(C_input) - C_current;
        D_angle_Error = Motor_Linear_Regression(D_input) + D_current;

        float tmp_A = ((A_angle_Error < 1 && A_angle_Error > - 1) || A_angle_Error > 150 || A_angle_Error < -150)?0:Motor_Control(A_angle_Error, dirAA, dirAB, A_current);
        float tmp_B = ((B_angle_Error < 1 && B_angle_Error > - 1) || B_current >= 150 || B_current <= -150)?0:Motor_Control(B_angle_Error, dirBA, dirBB, B_current);
        float tmp_C = ((C_angle_Error < 1 && C_angle_Error > - 1) || C_current > 100 || C_current < -100)?0:six1_Motor_Control(C_angle_Error, dirCA, dirCB, C_current);
        float tmp_D = ((D_angle_Error < 1 && D_angle_Error > - 1) || D_angle_Error > 150 || D_angle_Error < -150)?0:Motor_Control(D_angle_Error, dirDA, dirDB, D_current);

        pwmA = tmp_A;
        pwmB = tmp_B;
        pwmC = tmp_C;
        pwmD = tmp_D;

        control_motor_vel();

        Unity_Co.printf("*%f\n", pitch_input);
        
        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(MAIN_LOOP-(Work-Now)));
    }
}

void control_motor_vel(){
        
        motorA_rpm = encoA.CalRPM();
        motorA_rate = motorA_rpm*2*3.14159265358979323846/60;


        motorB_rpm = encoB.CalRPM();
        motorB_rate = motorB_rpm*2*3.14159265358979323846/60;


        motorC_rpm = encoC.CalRPM();
        motorC_rate = motorC_rpm*2*3.14159265358979323846/60;


        motorD_rpm = encoD.CalRPM();
        motorD_rate = motorD_rpm*2*3.14159265358979323846/60;
        

        // pc.printf("A : %f, B : %f, C : %f, D : %f\n",motorA_rpm, motorB_rpm, motorC_rpm, motorD_rpm);
        // pc.printf("A : %f\n",motorA_rpm);
        //motorR.run(1.0);

}

void DataRead_loop()
{
    uint32_t Now_M,Work_M;

    pc.printf("DataRead_start\n");

    while(1)
    {
        Now_M=rtos::Kernel::get_ms_count();
        // pc.printf("%d %f p\n",API_weather,PC_data[0]);

        if(gotPacket)
        {
            gotPacket = false;

            A_input = PC_data[0];
            B_input = PC_data[1];
            C_input = PC_data[2];
            D_input = PC_data[3];
            pitch_input = PC_data[4];
        }

        if(gotUnity)
        {
            gotUnity = false;
        }

        Work_M=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(DATA_UPDATE_RATE-(Work_M-Now_M)));
    }
}

void PcParser() //Data 시리얼 파싱
{
    static char pc_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
    static int pc_data_cnt=0,pc_buff_cnt=0;

    if(pc.readable())
    { // 통신 가능하면
        char byteIn_pc= pc.getc(); // 시리얼 통신 문자
        // pc.printf("%c",byteIn_pc);
        if (byteIn_pc=='*') // 시작문자 -> 초기화
        {
            pc_buff_cnt=0;
            pc_data_cnt=0;
        }

        else if(byteIn_pc==',') // weather(첫번쨰) 경우 외엔 모두 float 데이터로 변환해서 저장
        {
            pc_serialInBuffer[pc_buff_cnt]='\0';
            PC_data[pc_data_cnt++]=atof(pc_serialInBuffer);
            pc_buff_cnt=0;
        }

        else if(byteIn_pc=='\n'){ //끝문자
                // pc.printf("hi\n");
                pc_serialInBuffer[pc_buff_cnt]='\0';
                PC_data[pc_data_cnt]=atof(pc_serialInBuffer);

                gotPacket= true;
        }

        else
        { // 데이터 들어오는중
                pc_serialInBuffer[pc_buff_cnt++]=byteIn_pc;
        }
        
        if(pc_buff_cnt>=SBUF_SIZE) pc_buff_cnt=0; // 버퍼 넘치기 방지
    }
}

void UnityParser() //Data 시리얼 파싱
{
    static char un_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
    static int un_data_cnt=0,un_buff_cnt=0;

    if(Unity_Co.readable())
    { // 통신 가능하면
        char byteIn_un= Unity_Co.getc(); // 시리얼 통신 문자
        // pc.printf("%c",byteIn_pc);
        if (byteIn_un=='*') // 시작문자 -> 초기화
        {
            un_buff_cnt=0;
            un_data_cnt=0;
        }

        else if(byteIn_un==',') // weather(첫번쨰) 경우 외엔 모두 float 데이터로 변환해서 저장
        {
            un_serialInBuffer[un_buff_cnt]='\0';
            UN_data[un_data_cnt++]=atof(un_serialInBuffer);
            un_buff_cnt=0;
        }

        else if(byteIn_un=='\n'){ //끝문자
                // pc.printf("hi\n");
                un_serialInBuffer[un_buff_cnt]='\0';
                UN_data[un_data_cnt]=atof(un_serialInBuffer);

                gotUnity= true;
        }

        else
        { // 데이터 들어오는중
                un_serialInBuffer[un_buff_cnt++]=byteIn_un;
        }
        
        if(un_buff_cnt>=SBUF_SIZE) un_buff_cnt=0; // 버퍼 넘치기 방지
    }
}

void PRINT_thread_loop()// 디버깅 PRINT
{ 
    uint32_t Now_P,Work_P;

    pc.printf("Print_start\n");

    while(true)
    {
        Now_P=rtos::Kernel::get_ms_count();

        Work_P=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(PRINT_INTERVAL-(Work_P-Now_P)));
    }
}

int my_strcmp(char* str1, char* str2)
{
   int i = 0;
   // 한쪽 문자열이 끝날때까지 비교
   while (str1[i] != '\0' || str2[i] != '\0') {
   
      // 문자열 같으면 계속
      if (str1[i] == str2[i])
      {
           i++;   
             continue;
        }

      // 앞에 문자쪽 1
      else if (str1[i] > str2[i])
         return 1;

      // 뒤에 문자쪽 -1
      else if (str1[i] < str2[i])
         return -1;
      i++;
   }

   // 어느 한쪽 문자열이 끝났고 i - 1 까지 모두 같음
   if (str1[i] == str2[i])      // str1[i] == str2[i] == '\0' 문자열 끝남
       return 0;
   else if (str1[i] != '\0')   // str1에 글자가 남아있으면 1 리턴
      return 1;
   else return -1;            // str2에 글자가 남아있으면 -1 리턴
}

float Motor_Linear_Regression(float input)
{
    float result = (300/0.9)*input;

    if(result >= 150)
    {
        result = 150;
    }
    else if(result <= -150)
    {
        result = -150;
    }

    return result;
}

float Motor_Linear_Regression2(float input)
{
    float result = (300/0.9)*input;

    if(result >= 100)
    {
        result = 100;
    }
    else if(result <= -100)
    {
        result = -100;
    }

    return result;
}

float Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle)
{
    if(Error >= 0)
    {
        A = 0;
        B = 1;
    }
    else
    {
        A = 1;
        B = 0;
    }

    output = abs(Error)/50;

    if(output >= 0.7)
    {
    output = 0.7;
    }
    else if(output <= 0.4)
    {
        output = 0.4;
    }

    return output;
}

float reverse_Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle)
{
    if(Error >= 0)
    {
        A = 0;
        B = 1;
    }
    else
    {
        A = 1;
        B = 0;
    }

    output = abs(Error)/50;

    if(output >= 0.7)
    {
    output = 0.7;
    }
    else if(output <= 0.4)
    {
        output = 0.4;
    }

    return output;
}

float six1_Motor_Control(float Error, DigitalOut A, DigitalOut B, float current_angle)
{
    if(Error >= 0)
    {
        A = 0;
        B = 1;
    }
    else
    {
        A = 1;
        B = 0;
    }

    output = abs(Error)/50;

    if(output >= 0.8)
    {
        output = 0.8;
    }
    else if(output <= 0.4)
    {
        output = 0.4;
    }

    return output;
}
