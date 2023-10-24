#include "mbed.h"
#include <cmath>
#include <cstdint>
#include <utility>
#include "math.h"
#include "Si7020.h"
#include "MD30C.h"

#define DATA_UPDATE_RATE 80
#define MAIN_LOOP 60
#define PRINT_INTERVAL 200
#define SBUF_SIZE 400 // 시리얼 버퍼 크기

#define SIT_SERIAL_TX PC_12
#define SIT_SERIAL_RX PD_2

#define WINDIR D4
#define WINPWM D3

//--------------------------//
RawSerial pc(USBTX, USBRX, 115200);
RawSerial SIT_MCU(SIT_SERIAL_TX,SIT_SERIAL_RX,115200);

Thread DataRead_thread(osPriorityHigh); 
Thread Print_thread(osPriorityNormal); 

volatile float PC_data[100]; // 데이터 저장 몇개나 받을지
volatile float SIT_data[50];

bool gotPacket = false;
bool sit_gotPacket = false;

volatile int Chabum = 0;
volatile int Chabum_finish = 0;
volatile float User_Temp = 0;
volatile float User_Hum = 0;
volatile float API_Temp = 0;
volatile float API_Hum = 0;
volatile float API_wind_speed = 0;
volatile float API_weather = 0;

void DataRead_loop();
void PcParser();
void PRINT_thread_loop();
int my_strcmp(char* str1, char* str2);
//--------------------------//
I2C i2c(I2C_SDA, I2C_SCL);
Si7020 si(&i2c);

float Sensor_Temp = 0;
float Sensor_Hum = 0;

void Call_SI7020();
void SIT_Parser();

//------------------------//
MD30C win(WINDIR, WINPWM);
Timer test;

int main()
{   
    test.start();

    enum window
    {
        closed = 0,
        opened
    };

    int previousWindowState = closed;
    float disComfortIndex = 0.81*Sensor_Temp + 0.01*Sensor_Hum*(0.99*Sensor_Temp - 14.3) + 46.3; 

    uint32_t Now,Work;

    osThreadSetPriority(osThreadGetId(),osPriorityRealtime7);
    SIT_MCU.attach(&SIT_Parser);
    pc.attach(&PcParser); // uart 인터럽트 콜백
    DataRead_thread.start(&DataRead_loop); // 파싱 데이터 읽어오기 쓰레드
    Print_thread.start(&PRINT_thread_loop);

    while(1)
    {
        Now =rtos::Kernel::get_ms_count();

        Call_SI7020();
        
        if(API_weather == 1)
        {
            if(Sensor_Temp < User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed < 10)  //모든 조건 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp < User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed > 10)  //모든 조건 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed > 10)  //모든 조건 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed > 10)  //모든 조건 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp == User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed > 10)  //기온 만족하지만 불쾌 지수, 풍속 조건 불만족
            {
                if(API_wind_speed < 20)
                {
                    test.reset();
                    win.run(0);
                    while(true) {if(test.read_ms()>=4000)break;}
                    previousWindowState = opened;
                }
                else
                {
                    test.reset();
                    win.run(0.3);
                    while(true) {if(test.read_ms()>=4000)break;}
                    previousWindowState = closed;
                }
            }
            else if (Sensor_Temp == User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed > 10)  //기온 만족하지만 불쾌 지수, 풍속 조건 불만족
            {
                if(API_wind_speed < 20)
                {
                    test.reset();
                    win.run(-0.3);
                    while(true) {if(test.read_ms()>=4000)break;}
                    previousWindowState = opened;
                }
                else
                {
                    test.reset();
                    win.run(0);
                    while(true) {if(test.read_ms()>=4000)break;}
                    previousWindowState = closed;
                }
            }


            else if (Sensor_Temp < User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed > 10)  //불쾌 지수 만족하지만 기온, 풍속 조건 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp < User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed > 10)  //불쾌 지수 만족하지만 기온, 풍속 조건 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed > 10)  //불쾌 지수 만족하지만 기온, 풍속 조건 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed > 10)  //불쾌 지수 만족하지만 기온, 풍속 조건 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp < User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed < 10)  //풍속 조건 만족하지만 기온, 불쾌 지수 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp < User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed < 10)  //풍속 조건 만족하지만 기온, 불쾌 지수 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed < 10)  //풍속 조건 만족하지만 기온, 불쾌 지수 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = opened;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed < 10)  //풍속 조건 만족하지만 기온, 불쾌 지수 불만족
            {
                test.reset();
                win.run(-0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = opened;
            }


            else if (Sensor_Temp == User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed > 10)  //기온과 불쾌지수 만족하지만 풍속 조건 불만족
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp == User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed > 10)  //기온과 불쾌지수 만족하지만 풍속 조건 불만족
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp < User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed < 10)  //불쾌지수와 풍속 조건 만족하지만 실내온도 낮을 경우
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp < User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed < 10)  //불쾌지수와 풍속 조건 만족하지만 실내온도 낮을 경우
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp > User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed < 10)  //불쾌지수와 풍속 조건 만족하지만 실내온도 높을 경우
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp > User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed < 10)  //불쾌지수와 풍속 조건 만족하지만 실내온도 높을 경우
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp == User_Temp && disComfortIndex > 68 && previousWindowState == closed && API_wind_speed < 10)   //기온과 풍속조건 만족하지만 불쾌지수 높을 경우
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp == User_Temp && disComfortIndex > 68 && previousWindowState == opened && API_wind_speed < 10)  //기온과 풍속조건 만족하지만 불쾌지수 높을 경우
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }


            else if (Sensor_Temp == User_Temp && disComfortIndex < 68 && previousWindowState == closed && API_wind_speed < 10)  //모든 지표 완벽시 창문 닫음
            {
                test.reset();
                win.run(0);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
            else if (Sensor_Temp == User_Temp && disComfortIndex < 68 && previousWindowState == opened && API_wind_speed < 10)  //모든 지표 완벽시 창문 닫음
            {
                test.reset();
                win.run(0.3);
                while(true) {if(test.read_ms()>=4000)break;}
                previousWindowState = closed;
            }
        }

        else 
        {
            previousWindowState = closed;
        }

        if(Chabum){
            if(!Chabum_finish) SIT_MCU.printf("*%d\n",Chabum);
            else SIT_MCU.printf("*0\n");
        }
        
        pc.printf("%d,%f,%f\n",Chabum_finish ,Sensor_Temp, Sensor_Hum);
        Work =rtos::Kernel::get_ms_count();
        //pc.printf("%d\n", Work-Now);
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(MAIN_LOOP-(Work-Now)));
    }
}

void Call_SI7020()
{
    if(si.getHumidity(&Sensor_Hum) != 0)
    {
        //printf("Error getting humidity\n");
        Sensor_Hum = -1;
    }

    if(si.getTemperature(&Sensor_Temp) != 0)
    {
        //printf("Error getting temperature");
        Sensor_Temp = -1;
    }
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

            Chabum = PC_data[0];
            User_Temp = PC_data[1];
            User_Hum = PC_data[2];
            API_Temp = PC_data[3];
            API_wind_speed = PC_data[4];
            API_weather = PC_data[5];
        }

        if(sit_gotPacket)
        {
                
            sit_gotPacket = false;

            Chabum_finish = (int)SIT_data[0];

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

void SIT_Parser() //Data 시리얼 파싱
{
    static char sit_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
    static int sit_data_cnt=0,sit_buff_cnt=0;

    if(SIT_MCU.readable())
    { // 통신 가능하면
        char byteIn_sit= SIT_MCU.getc(); // 시리얼 통신 문자
        //pc.printf("%c",byteIn_sit);
        if (byteIn_sit=='*') // 시작문자 -> 초기화
        {
            sit_buff_cnt=0;
            sit_data_cnt=0;
        }

        else if(byteIn_sit==',') // weather(첫번쨰) 경우 외엔 모두 float 데이터로 변환해서 저장
        {
            sit_serialInBuffer[sit_buff_cnt]='\0';
            SIT_data[sit_data_cnt++]=atof(sit_serialInBuffer);
            sit_buff_cnt=0;
        }

        else if(byteIn_sit=='\n'){ //끝문자
                // pc.printf("hi\n");
                sit_serialInBuffer[sit_buff_cnt]='\0';
                SIT_data[sit_data_cnt]=atof(sit_serialInBuffer);

                sit_gotPacket= true;
        }

        else
        { // 데이터 들어오는중
                sit_serialInBuffer[sit_buff_cnt++]=byteIn_sit;
        }
        
        if(sit_buff_cnt>=SBUF_SIZE) sit_buff_cnt=0; // 버퍼 넘치기 방지
    }
}



void PRINT_thread_loop()// 디버깅 PRINT
{ 
    uint32_t Now_P,Work_P;

    pc.printf("Print_start\n");

    while(true)
    {
        Now_P=rtos::Kernel::get_ms_count();

        //printf("DEBUG: %f\n", API_humidity);
        //pc.printf("%f, %f, %d, %f, %f, %f, %f, %f\n", Sensor_Temp, sensor_humid, API_weather, API_humidity, API_temperature, API_wind_speed, input_temp, input_hum);
        //pc.printf("%d %f p\n",API_weather,PC_data[0]);

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
