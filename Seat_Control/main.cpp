#include "mbed.h"
#include "DUALMOTOR.h"
#include "MD30C.h"
#include "PID.h"
#include "QEncoder2.h"

#define alpha 0.8
//#define alpha2 0.9

#define ROT_MOTOR_PWM D10
#define ROT_MOTOR_DIR D9
#define VER_MOTOR_PWM D11
#define VER_MOTOR_DIR D12
#define DISP_MOTOR_PWM D8
#define DISP_MOTOR_DIR D7
#define MOTOR_DISP_ENC_PINA D4 // white = a
#define MOTOR_DISP_ENC_PINB D3 // yellow = b
#define HALL_sENSOR A4


#define DISP_MAX_VAL 120
#define DISP_MIN_VAL 0

#define PI 3.14159265358979323846

#define ENCODER_SERIAL_TX A0
#define ENCODER_SERIAL_RX A1

#define UI_SERIAL_TX PC_12
#define UI_SERIAL_RX PD_2

#define UNITY_SERIAL_TX PC_10
#define UNITY_SERIAL_RX PC_11

#define MAX_ANGLE_ENCO_VAL 56.97

#define MIN_ANGLE_ENCO_VAL -8

#define MAX_HEIGHT_ENCO_VAL 164
#define MIN_HEIGHT_ENCO_VAL 2

#define MAX_DISP 121
#define ROT_ANGLE_TO_DISP 10


#define DATA_UPDATE_RATE 8

#define MAIN_LOOP 8

#define PRINT_INTERVAL 100

DigitalIn but(BUTTON1, PullUp);

RawSerial pc(USBTX, USBRX, 115200);
RawSerial ENCODER_SER(ENCODER_SERIAL_TX, ENCODER_SERIAL_RX, 115200);

RawSerial unity(UNITY_SERIAL_TX,UNITY_SERIAL_RX,115200);
RawSerial ui(UI_SERIAL_TX, UI_SERIAL_RX, 115200);


// InterruptIn test(MOTOR_DISP_ENC_PINA);
QEncoder enco_disp(MOTOR_DISP_ENC_PINA,MOTOR_DISP_ENC_PINB,alpha,MAIN_LOOP,'R');
MD30C ver_motor(VER_MOTOR_DIR,VER_MOTOR_PWM);
MD30C rot_motor(ROT_MOTOR_DIR,ROT_MOTOR_PWM);
MD30C disp_motor(DISP_MOTOR_DIR,DISP_MOTOR_PWM);

DigitalIn hall_sensor(HALL_sENSOR,PullUp);

#define SBUF_SIZE 100 // 시리얼 버퍼 크기
#define RATE_CONTROLLER_KP 1
#define RATE_CONTROLLER_KI 0.1
#define RATE_CONTROLLER_KD 0


//--------------------------//


volatile float PC_data[50]; // 데이터 저장 몇개나 받을지
volatile int enco_data[50]; // 데이터 저장 몇개나 받을지
volatile float ui_data[20];
volatile float UNITY_data[20];


Timer initial_timer;
Thread DataRead_thread(osPriorityHigh); 
Thread Print_thread(osPriorityNormal); 


PwmOut Servo(PC_9);

bool initialize_flag = true;
bool gotPacket = false;
bool enco_gotPacket = false;

volatile bool ui_gotPacket = false;
volatile bool unity_gotPacket = false;
volatile bool Car_camping_finsh = false;


int Car_camping = 0;



float vel_disp = 0,vel_ver = 0,vel_rot = 0;

int enco_val_ver=0,enco_val_rot=0;

int las_enco_val_disp = 0,las_enco_val_ver = 0,las_enco_val_rot = 0;

int filtered_enco_val_ver=0,filtered_enco_val_rot=0;

// float enco_val_disp=0;
// int filtered_enco_val_disp=0;
// int las_filtered_enco_val_disp=0;

float enco_disp_rate = 0;
float enco_disp_rpm = 0;
float last_ang_sum = 0;




int las_filtered_enco_val_ver=0;
int las_filtered_enco_val_rot=0;

float angle_trans_motor_ver = 0;
float angle_trans_motor_rot = 0;
float measure_height = 0;
float measure_disp = 0;

float target_angle_rot_rad =0; 
float target_angle_rot = 0;
float target_disp = 60;


// float target_height_ver = 63;
float target_height_ver = 63.7;

float pid_target_ver = 0;
float pid_target_rot = 0;
float pid_target_disp = 0;

void DataRead_loop();

void PcParser();
void EncoParser();
void Ui_Serial_Parser();
void Unity_Serial_Parser();


void PRINT_thread_loop();
float ang_to_height(float);



bool emergency_stop = false;
bool enco_zero_point = true;


int tmp = 0;
int counttt = 0;
void flip(){
    counttt +=1;
}

int LPF(int prev_data, int current_data);
template <class T> T map(T x, T in_min, T in_max, T out_min, T out_max);
float constrain(float x, float x_min, float x_max);

PIDBasic pid_ver(RATE_CONTROLLER_KP, RATE_CONTROLLER_KI, RATE_CONTROLLER_KD, 0);// kp,ki,kd,imax
PIDBasic pid_rot(RATE_CONTROLLER_KP, RATE_CONTROLLER_KI, RATE_CONTROLLER_KD, 0);// kp,ki,kd,imax
PIDBasic pid_disp(RATE_CONTROLLER_KP, RATE_CONTROLLER_KI, RATE_CONTROLLER_KD, 0);// kp,ki,kd,imax


// DigitalIn but(BUTTON1);
int cnt = 0;
int sen_val= 1;
bool set_position_zero = true;

//////////////서보모터//////////////////

void servo_set(PwmOut &rc);

volatile float servo_ang = 90.0;



int main()
{   
    
    uint32_t Now,Work;
    enco_data[0] = -1;
    enco_data[1] = -1;
    enco_data[2] = -1;


    ENCODER_SER.attach(&EncoParser);
    pc.attach(&PcParser);
    ui.attach(&Ui_Serial_Parser);
    unity.attach(&Unity_Serial_Parser);



    DataRead_thread.start(&DataRead_loop);
    Print_thread.start(&PRINT_thread_loop);
    enco_disp.init();
    

    initial_timer.start();
    initial_timer.reset();
    

    servo_set(Servo);

    while(true){
            if(initial_timer.read_ms()>=500) break;
    }

    while (true){
        
        Now=rtos::Kernel::get_ms_count();

        sen_val = hall_sensor.read();
        if(!but) {

            ver_motor.run(0);
            rot_motor.run(0);
            emergency_stop=false;
            
            // vel = 0;
            // ROTATE_MOTOR.run(0);
        }        
        

        ///////rot////////
        if(enco_val_rot>=0) filtered_enco_val_rot=LPF(las_filtered_enco_val_rot,enco_val_rot);

        else filtered_enco_val_rot=-LPF(las_filtered_enco_val_rot,enco_val_rot);

        if (filtered_enco_val_rot>=0) angle_trans_motor_rot = float((510-filtered_enco_val_rot))/1024*360;
        else angle_trans_motor_rot = float((-filtered_enco_val_rot-510))/1024*360;
        angle_trans_motor_rot += 95.3;


        pid_target_rot=pid_rot.computePID(target_angle_rot,angle_trans_motor_rot,DATA_UPDATE_RATE,MAX_ANGLE_ENCO_VAL/8);
        if (pid_target_rot>=0)
            {
                if (angle_trans_motor_rot<=-4){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.515, 0.525);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                else if (angle_trans_motor_rot<=-2){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.50, 0.515);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                else if (angle_trans_motor_rot<=0){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.485, 0.50);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                else if (angle_trans_motor_rot<=4){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.475, 0.485);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                else if(angle_trans_motor_rot<=10){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.46, 0.475);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                else if(angle_trans_motor_rot<=20){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.45, 0.46);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.25;
                }
                else if(angle_trans_motor_rot<=23){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.42, 0.435);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.25;
                }
                else if(angle_trans_motor_rot<=27){
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL/8, 0.415, 0.42);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.25;
                }
                else{
                    vel_rot = map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL, 0.15, 0.3);
                    if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
                }
                //if (abs(pid_target_rot)<=0.35) pid_target_rot=0;
            }

        else {

            if(angle_trans_motor_rot>=27) vel_rot = 0.05;
            else if (angle_trans_motor_rot>=23) vel_rot = 0.08;
            else if (angle_trans_motor_rot>=18) vel_rot = 0.09;
            else if (angle_trans_motor_rot>=12) vel_rot = 0.1;
            
            else vel_rot=0.12;
            if (abs(pid_target_rot)<=0.4) vel_rot=0.3;
            // vel_rot = -map<float>(abs(pid_target_rot), 0, MAX_ANGLE_ENCO_VAL, 0.1, 0.15);
            // if (abs(pid_target_rot)<=0.4){
            //     if(angle_trans_motor_rot<=20)
            //     vel_rot=0.3;
            // }
            
        }
        //if (abs(pid_target_rot)<=0.35) pid_target_rot=0;
        //if(pid_target_rot<0) vel_rot = -vel_rot;   
        //if (abs(pid_target_rot)<=3) vel_rot=0;
        //if (abs(vel_rot)<=map<float>(abs(0.503), 0, 1024.0/2, 0.503, 0.512)) vel_rot = 0;





        /////////ver/////////
        if(enco_val_ver>=0) filtered_enco_val_ver=LPF(las_filtered_enco_val_ver,enco_val_ver);

        else filtered_enco_val_ver=-LPF(las_filtered_enco_val_ver,enco_val_ver);

        if (filtered_enco_val_ver>=0) angle_trans_motor_ver = float((510-filtered_enco_val_ver))/1024*360;
        else angle_trans_motor_ver = float((-filtered_enco_val_ver-510))/1024*360;

        measure_height = ang_to_height(angle_trans_motor_ver);
        
        

        target_height_ver=constrain(target_height_ver, 62.4, 123);

        
        pid_target_ver=pid_ver.computePID(target_height_ver,measure_height,DATA_UPDATE_RATE,60.0);

        vel_ver = map<float>(abs(pid_target_ver), 0, 60, 0.1, 0.15);
        if(pid_target_ver<0) vel_ver = -vel_ver;
        if (abs(vel_ver)<=map<float>(abs(0.4), 0, 60, 0.1, 0.15)) vel_ver =0.01;


        


        //if (abs(vel)<=0.05) vel = 0;

        // if(filtered_enco_val*las_filtered_enco_val<=0&&enco_zero_point){
        //     enco_zero_point = false;
        //     vel = 0;
        //     enco_data[0]=0;
        //     ROTATE_MOTOR.stop();
        // }
        // else{
        //     ROTATE_MOTOR.run(vel);
        // }




        

        ///////////disp
        //enco_disp_rpm = 0;
        enco_disp_rpm = enco_disp.CalRPM();
        // if (abs(enco_disp_rpm)<=1){
        //     enco_disp_rpm =0;
        // }
        // if (abs(enco_disp_rpm) <= 1){
        //     enco_disp_rpm = 0;
        // }
        //enco_disp_rate = enco_disp_rpm*2*3.14159265358979323846/60;
        enco_disp_rate = enco_disp_rpm*60; //각속도(degree)
        
        last_ang_sum += enco_disp_rate*0.006;
        measure_disp = last_ang_sum*ROT_ANGLE_TO_DISP/360;
        
        target_disp=constrain(target_disp, 0, DISP_MAX_VAL);

        pid_target_disp=pid_disp.computePID(target_disp,measure_disp,DATA_UPDATE_RATE,DISP_MAX_VAL); 
        if(abs(pid_target_disp)<=0.3) pid_target_disp = 0;
        vel_disp = map<float>(abs(pid_target_disp), 0, DISP_MAX_VAL, 0.5, 1.0);
        if(pid_target_disp<0) vel_disp = -vel_disp;
        if (abs(vel_disp)<=map<float>(abs(0.4), 0, DISP_MAX_VAL, 0.5, 1.0)) vel_disp =0;



        if(Car_camping){
            //문턱값 0.5
            //0.5로 들리는 순간 -> 10도
            servo_ang = 90-target_angle_rot;
            rot_motor.run(vel_rot);
            ver_motor.run(vel_ver);

            uint16_t pulseW = map<float>(servo_ang, 180., 0., 600., 2600.);
            Servo.pulsewidth_us(pulseW);

            //0일때 자석이 있고, 1일때 자석이 없고.
            if(set_position_zero && !sen_val){
                last_ang_sum = 0;
                disp_motor.run(0);
            }
            //if(set_position_zero) last_ang_sum = 0;
            if(!sen_val) set_position_zero = false;

            if(set_position_zero) disp_motor.run(-0.7);
            else{
                disp_motor.run(vel_disp);
            }

            if(abs(pid_target_disp)<=0.3 && abs(pid_target_ver)<=0.4 && abs(pid_target_rot)<= 0.4) {
                ui.printf("*1\n");
                Car_camping_finsh = true;
                //pc.printf("*1\n");
                
            }

        }

        else{
            rot_motor.run(0.3);
            ver_motor.run(0);
            disp_motor.run(0);
        }

        

        //if(!sen_val) set_position_zero = false;



        // if(set_position_zero && sen_val){
        //     disp_motor.run(0);
        // }
        // else {
        //     disp_motor.run(0);
        // }



        // if(sen_val) disp_motor.run(0.6);
        // disp_motor.run(0.6);
        //else disp_motor.run(0);


        //disp_motor.run(vel_disp);

        //홀센서 부분 추가 + 엔코더 각속도로 포지션 제어

    

        //pc.printf("%d\n",enco_disp.get_pulse_Count());
        // 0일때 자석이 있고, 1일때 자석이 없고.
        //int coun_t = test.read();
        //pc.printf("A : %f %f H: %f %f D %f %f\n",angle_trans_motor_rot,target_angle_rot,measure_height,target_height_ver,measure_height,target_disp);
        //pc.printf("%d %f %f %f %f\n",sen_val, enco_disp_rate, measure_disp, pid_target_disp, vel_disp);
        //pc.printf("%f %f %f %f\n",angle_trans_motor_rot,target_angle_rot,pid_target_rot,vel_rot);

        las_filtered_enco_val_ver = filtered_enco_val_ver;
        las_filtered_enco_val_rot = filtered_enco_val_rot;

        Work=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(DATA_UPDATE_RATE-(Work-Now)));


    }
}




void DataRead_loop(){

    uint32_t Now_M,Work_M;

    pc.printf("DataRead_start\n");


    while(1){

        Now_M=rtos::Kernel::get_ms_count();
        
        if(gotPacket){
                gotPacket = false;
                
                // target_height_ver = PC_data[0];
                // //target_angle_rot = PC_data[1];
                // target_disp = PC_data[1];
                //vel = PC_data[0]/10;
                
                enco_zero_point = true;
                // else emergency_stop = false;

        }

        if(unity_gotPacket){
                unity_gotPacket = false;
                
                if(Car_camping && !Car_camping_finsh){
                    
                    Car_camping_finsh=false;
                    target_angle_rot = UNITY_data[0];
                    
                    target_disp = 323.2-222.173*cos((target_angle_rot+8.02)/180.0f*PI)-150.752*cos((70.734-target_angle_rot)/180.0f*PI);
                    
                    target_height_ver = 84+23+ 94.51+222.173*sin((target_angle_rot+8.02)/180.0f*PI)-150.752*sin((70.734-target_angle_rot)/180.0f*PI);


                }
                
                //vel = PC_data[0]/10;
                //if (abs(vel)>1) emergency_stop=true;
                
                // else emergency_stop = false;

        }

        if(enco_gotPacket){
                enco_gotPacket = false;
                enco_val_ver = 1024/2-enco_data[0];
                enco_val_rot = 1024/2-enco_data[1];
        }


        if(ui_gotPacket){
                ui_gotPacket = false;
                Car_camping = int(ui_data[0]);
        }


        Work_M=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(DATA_UPDATE_RATE-(Work_M-Now_M)));

    }
}




void EncoParser() //Data 시리얼 파싱
{
  static char enco_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
  static int enco_data_cnt=0,enco_buff_cnt=0;

  if(ENCODER_SER.readable()){ // 통신 가능하면

		char byteIn_enco= ENCODER_SER.getc(); // 시리얼 통신 문자
        //pc.printf("%c",byteIn_enco);
        if (byteIn_enco=='*') // 시작문자 -> 초기화
        {
            enco_buff_cnt=0;
            enco_data_cnt=0;
        }
        else if(byteIn_enco==','){

            enco_serialInBuffer[enco_buff_cnt]='\0';
            enco_data[enco_data_cnt++]=atoi(enco_serialInBuffer);
            enco_buff_cnt=0;

        }

        else if(byteIn_enco=='\n'){ //끝문자
                // pc.printf("hi\n");
                enco_serialInBuffer[enco_buff_cnt]='\0';
                enco_data[enco_data_cnt]=atoi(enco_serialInBuffer);
                if(initialize_flag && enco_data[0] != -1 && enco_data[1] != -1) {
                    las_filtered_enco_val_ver = 1024/2-enco_data[0];
                    las_filtered_enco_val_rot = 1024/2-enco_data[1];
                    initialize_flag=false;
                }
                if(!initialize_flag) enco_gotPacket= true;
        }

        else{ // 데이터 들어오는중
                enco_serialInBuffer[enco_buff_cnt++]=byteIn_enco;
        }
        
        if(enco_buff_cnt>=SBUF_SIZE) enco_buff_cnt=0; // 버퍼 넘치기 방지
    }
}


void Ui_Serial_Parser() //Data 시리얼 파싱
{
  static char ui_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
  static int ui_data_cnt=0,ui_buff_cnt=0;

  if(ui.readable()){ // 통신 가능하면

		char byteIn_ui= ui.getc(); // 시리얼 통신 문자
        // pc.printf("%c",byteIn_pc);
        if (byteIn_ui=='*') // 시작문자 -> 초기화
        {
            ui_buff_cnt=0;
            ui_data_cnt=0;
        }
        else if(byteIn_ui==','){
            ui_serialInBuffer[ui_buff_cnt]='\0';
            ui_data[ui_data_cnt++]=atof(ui_serialInBuffer);
            ui_buff_cnt=0;
        }

        else if(byteIn_ui=='\n'){ //끝문자
                // pc.printf("hi\n");
                ui_serialInBuffer[ui_buff_cnt]='\0';
                ui_data[ui_data_cnt]=atof(ui_serialInBuffer);
                
                ui_gotPacket= true;
        }

        else{ // 데이터 들어오는중
                ui_serialInBuffer[ui_buff_cnt++]=byteIn_ui;
        }
        
        if(ui_buff_cnt>=SBUF_SIZE) ui_buff_cnt=0; // 버퍼 넘치기 방지
    }
}

void Unity_Serial_Parser() //Data 시리얼 파싱
{
  static char unity_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
  static int unity_data_cnt=0,unity_buff_cnt=0;

  if(unity.readable()){ // 통신 가능하면

		char byteIn_unity= unity.getc(); // 시리얼 통신 문자
        // pc.printf("%c",byteIn_pc);
        if (byteIn_unity=='*') // 시작문자 -> 초기화
        {
            unity_buff_cnt=0;
            unity_data_cnt=0;
        }
        else if(byteIn_unity==','){

            unity_serialInBuffer[unity_buff_cnt]='\0';
            UNITY_data[unity_data_cnt++]=atof(unity_serialInBuffer);
            unity_buff_cnt=0;

        }

        else if(byteIn_unity=='\n'){ //끝문자
                // pc.printf("hi\n");
                unity_serialInBuffer[unity_buff_cnt]='\0';
                UNITY_data[unity_data_cnt]=atof(unity_serialInBuffer);
                
                unity_gotPacket= true;
        }

        else{ // 데이터 들어오는중
                unity_serialInBuffer[unity_buff_cnt++]=byteIn_unity;
        }
        
        if(unity_buff_cnt>=SBUF_SIZE) unity_buff_cnt=0; // 버퍼 넘치기 방지
    }
}


void PcParser() //Data 시리얼 파싱
{
  static char pc_serialInBuffer[SBUF_SIZE];//시리얼 버퍼 배열 만들기
  static int pc_data_cnt=0,pc_buff_cnt=0;

  if(pc.readable()){ // 통신 가능하면

		char byteIn_pc= pc.getc(); // 시리얼 통신 문자
        // pc.printf("%c",byteIn_pc);
        if (byteIn_pc=='*') // 시작문자 -> 초기화
        {
            pc_buff_cnt=0;
            pc_data_cnt=0;
        }
        else if(byteIn_pc==','){

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

        else{ // 데이터 들어오는중
                pc_serialInBuffer[pc_buff_cnt++]=byteIn_pc;
        }
        
        if(pc_buff_cnt>=SBUF_SIZE) pc_buff_cnt=0; // 버퍼 넘치기 방지
    }
}


int LPF(int prev_data, int current_data){
    if(isnan(current_data)){
        return 0;
    }
    else{
        int fil_data;
        fil_data = alpha*(abs(prev_data))+(1-alpha)*abs(current_data);
        // pc.printf("%d %d %d\n", *prev_data, fil_data,current_data);
        return fil_data;
    }
}

template <class T> T map(T x, T in_min, T in_max, T out_min, T out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float x, float x_min, float x_max){
        if(x>x_max) return x_max;
        else if (x_max>=x && x>x_min) return x;
        else return x_min;
}

float ang_to_height(float theta){
    theta = theta/180.0f*PI;
    return 50*sin(theta)+79.94*sqrt((1-((50/79.94)*cos(theta))*((50/79.94)*cos(theta))));
}

void PRINT_thread_loop(){
    uint32_t Now_P,Work_P;
    while(true){
        Now_P=rtos::Kernel::get_ms_count();
        //pc.printf("t m p v %f %f %f %f\n",target_height_ver,measure_height,pid_target_ver,vel_ver);
        pc.printf("A : %f %f H: %f %f D %f %f\n",angle_trans_motor_rot,target_angle_rot,measure_height,target_height_ver,measure_disp,target_disp);
        //ui.printf("*3\n");
        Work_P=rtos::Kernel::get_ms_count();
        ThisThread::sleep_until(rtos::Kernel::get_ms_count()+(PRINT_INTERVAL-(Work_P-Now_P)));
    }
}





void servo_set(PwmOut &rc){
    rc.period_ms(20);
    uint16_t pulseW = map<float>(servo_ang, 180., 0., 600., 2600.);
    rc.pulsewidth_us(pulseW);
}



// void servo_chk(PwmOut &rc){
//     ang+=inc;
//     if (ang > 180.f || ang < 0.f){
//         inc = -inc;
//     }
    
//     ang = 90;

//     uint16_t pulseW = map<float>(ang, 0., 180., 500., 2600.);
//     // pc.printf("%f %d\n",ang, pulseW);
//     rc.pulsewidth_us(pulseW);
// }
