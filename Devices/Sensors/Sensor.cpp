/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Sensor.h"
#include <cmath>
#include "Matix.h"

extern I2C_HandleTypeDef hi2c2;

//PressureSensor PressureSensor::pressure_sensor(B02_IIC_ADDRESS);


void PressureSensor::Handle(){
    static uint32_t cnt = 0;
    if (++cnt < 20)return;
    cnt = 0;//20分频
        //flag_Busy = 1;
        for(int i=0;i<SENSOR_NUM;++i){
            TCA_SelectSingleChannel(i+1);
            Handle_single(i);//收集每个传感器数据
        }
        Update_plane();//计算机器人参考系下水面方程
        data_depth=-data_plane[3];//计算深度
        pitch=atan(-data_plane[0]/(sqrt(data_plane[1]*data_plane[1]+data_plane[2]*data_plane[2])));//计算俯仰角
        roll=atan(data_plane[1]/data_plane[2]);//计算横滚角

}


void PressureSensor::Delay_us(uint32_t us)
{
    uint32_t i;
    for (i = 0; i < us; i++ )
    {
        int a = 10;  //delay based on main clock, 168Mhz
        while (a-- );
    }
}

unsigned char PressureSensor::MS5837_30BA_Crc4(int id)
{
    int cnt;
    int t;
    unsigned int n_rem=0;
    unsigned char n_bit;
    unsigned char  a=0;
    unsigned char  b=0;
    unsigned short  int n_prom[8];

    for( t=0;t<7;t++)
    {
        n_prom[t]=Cal_C[id][t];
    }
    n_prom[0]=((n_prom[0]) & 0x0FFF);
    n_prom[7]=0;
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem = (n_rem << 1);
        }

    }
    n_rem= ((n_rem >> 12) & 0x000F);
    a=(n_rem ^ 0x00);
    b=Cal_C[id][0]>>12;
    if (a==b)
    {
        return 1;
    }
    else return 0;

}

void PressureSensor::MS5837_30BA_ReSet(void)
{
    uint8_t data[1];
    data[0] = MS5837_30BA_ResetCommand;
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, data, 1, 0xffff);
}

uint8_t PressureSensor::MS5837_30BA_PROM(int id)
{
    uint8_t memaddr[1];
    uint8_t data[2];
    int i;

    MS5837_30BA_ReSet();	                                             //��λMS5837
    //HAL_Delay(20);
    for (i=0;i<7;i++)
    {
        memaddr[0] = MS5837_30BA_PROM_RD + (i*2);
        HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, memaddr, 1, 0xffff);
        HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, data, 2, 0xffff);
        Cal_C[id][i] = (((uint16_t)data[0] << 8) | data[1]);
    }
    return !Cal_C[id][0];
}



unsigned long PressureSensor::MS5837_30BA_GetConversion(uint8_t command)
{
    unsigned long conversion = 0;
    uint8_t data = MS5837_30BA_ADC_RD;
    uint8_t temp[3];
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &command, 1, 0xffff);
    HAL_Delay(10);
    //Delay_us(15000);
    HAL_I2C_Master_Transmit(&hi2c2, B02_IIC_ADDRESS, &data, 1, 0xffff);

    HAL_I2C_Master_Receive(&hi2c2, B02_IIC_ADDRESS, temp, 3, 0xffff);
    conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
    return conversion;
}

/*signed int PressureSensor::MS5837_30BA_GetTemp(int id)
{
    if(flag_ok[id]) {
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
        HAL_Delay(20);
        dT = D2_Temp - (((uint32_t) Cal_C[id][5]) * 256l);
        return dT;
    }
    else return -1;
}*/

float PressureSensor::MS5837_30BA_GetData(int id)
{
    if(flag_ok[id]){
        D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
        HAL_Delay(20);
        //Delay_us(30000);
        D1_Pres = MS5837_30BA_GetConversion(MS5837_30BA_D1_OSR_8192);
        HAL_Delay(20);
        //Delay_us(30000);
        dT = D2_Temp - (((uint32_t) Cal_C[id][5]) * 256l);
        SENS = (int64_t) Cal_C[id][1] * 65536l + ((int64_t) Cal_C[id][3] * dT) / 128l;
        OFF_ = (int64_t) Cal_C[id][2] * 131072l + ((int64_t) Cal_C[id][4] * dT) / 64l;

        //TEMP = 2000l + (int64_t)(dT) * Cal_C[id][6] / 8388608LL;
        TEMP = 2000l + (int64_t)(dT) * Cal_C[id][6] / 8388608LL;

//�����¶Ȳ���
        if (TEMP < 2000)  // low temp
        {

            Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        } else {         // high temp
            Ti = 2 * (dT * dT) / (137438953472LL);
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
        OFF2 = OFF_ - OFFi;
        SENS2 = SENS - SENSi;
        /*if(D1_Pres==0||D2_Temp==0) return last_measure[id];
        else{
            float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
            last_measure[id]=pressure;
            return pressure;

        }*/
        float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
        return pressure;

        //data_pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;          //У׼��ѹ������

        //Temperature = (TEMP - Ti) / 100.0;                                //У׼���¶�����
    }
    else return -1;
}

void PressureSensor::Init_single(int id)
{
    data_pressure_offset[id]=0;
    if(!flag_ok[id]) {
        MS5837_30BA_PROM(id);
        flag_ok[id] = MS5837_30BA_Crc4(id);
    }
    int32_t i=0;
    while(i<50){
        //data_temp[id] += MS5837_30BA_GetTemp(id) / 50;
        data_pressure_raw[id] = MS5837_30BA_GetData(id);
        float temp = data_pressure_raw[id] - data_pressure_offset[id];
        if(temp<100&&temp>-10){
            data_pressure_offset[id]=temp/50;
            ++i;
        }
        //data_pressure_offset[id] += MS5837_30BA_GetData(id) / 50;


    }
}


void PressureSensor::Handle_single(int id)
{

    data_pressure_raw[id] = MS5837_30BA_GetData(id);
    float temp = data_pressure_raw[id] - data_pressure_offset[id];
    if(temp<100&&temp>-10){
        data_pressure[id]=temp;
    }
    //if(data_pressure[id]>200||data_pressure[id]<-10) data_pressure[id]=last_measure[id]- data_pressure_offset[id];


}



void PressureSensor::Solve_plane_3(float* data,float* h,float* x,float* y,float* z){
    float pos_data[9];
    for(int i=0;i<3;++i){
        pos_data[i*3]=x[i];
        pos_data[i*3+1]=y[i];
        pos_data[i*3+2]=z[i];
    }

    double a=0,b=0,c=0,d=0;
    Matrix pos(3,3,pos_data);

    Matrix pos_inv;
    if(pos_inv.inv(pos)){

        float a1=0,a2=0,b1=0,b2=0,c1=0,c2=0,ax=0,bx=0,cx=0;

        for(int i=1;i<4;++i){
            a1-=pos_inv.mat[1][i];
            b1-=pos_inv.mat[2][i];
            c1-=pos_inv.mat[3][i];
            a2-=pos_inv.mat[1][i]*h[i-1];
            b2-=pos_inv.mat[2][i]*h[i-1];
            c2-=pos_inv.mat[3][i]*h[i-1];
        }

        ax=a1*a1+b1*b1+c1*c1;
        bx=2*(a1*a2+b1*b2+c1*c2);
        cx=a2*a2+b2*b2+c2*c2-1;
        data[0]=(-bx-sqrt(bx*bx-4*ax*cx))/(2*ax);
        data[1]=a1*d+a2;
        data[2]=b1*d+b2;
        data[3]=c1*d+c2;
    }
}


void PressureSensor::Solve_plane(float* data,float* h,float* x,float* y,float* z,int num){
    float xi=0,yi=0,zi=0,hi=0,xi2=0,yi2=0,zi2=0,xiyi=0,xizi=0,yizi=0,xihi=0,yihi=0,zihi=0;
    for(int i=0;i<num;++i){
        xi+=x[i];
        yi+=y[i];
        zi+=z[i];
        hi+=h[i];
        xi2+=x[i]*x[i];
        yi2+=y[i]*y[i];
        zi2+=z[i]*z[i];
        xiyi+=x[i]*y[i];
        xizi+=x[i]*z[i];
        yizi+=y[i]*z[i];
        xihi+=x[i]*h[i];
        yihi+=y[i]*h[i];
        zihi+=z[i]*h[i];
    }
    float gradient[5]={0};
    gradient[0] = xi2*data[0]  + xiyi*data[1] + xizi*data[2] + xi*data[3] + xihi + data[0]*data[4];
    gradient[1] = xiyi*data[0] + yi2*data[1]  + yizi*data[2] + yi*data[3] + yihi + data[1]*data[4];
    gradient[2] = xizi*data[0] + yizi*data[1] + zi2*data[2]  + zi*data[3] + zihi + data[2]*data[4];
    gradient[3] = xi*data[0]   + yi*data[1]   + zi*data[2]   + num*data[3]    + hi;
    gradient[4] = data[0]*data[0] + data[1]*data[1] + data[2]*data[2] -1;

    float jacobi_data[25]={xi2+data[4],xiyi,xizi,xi,data[0],
                           xiyi,yi2+data[4],yizi,yi,data[1],
                           xizi,yizi,zi2+data[4],zi,data[2],
                           xi,yi,zi,(float)num,0,
                           data[0],data[1],data[2],0,0};
    Matrix jacobi(5,5,jacobi_data);
    Matrix jacobi_inv;
    if(jacobi_inv.inv(jacobi)){
        float data_dx[5]={0};
        for(int i=0;i<5;++i){
            for(int j=0;j<5;++j){
                data_dx[i]+=jacobi_inv.mat[i+1][j+1]*gradient[j];
            }
            data[i]-=0.5*data_dx[i];
        }
    }
}


void PressureSensor::Update_plane(){
    Solve_plane(data_plane,data_pressure,site.x,site.y,site.z,SENSOR_NUM);

    for(int i=0;i<3;++i){
        data_level[i]=data_plane[i];
    }
}


/*
void PressureSensor::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}
*/


void Sonar::Handle(){
    static uint8_t cnt = 0;
    if (++cnt < 20)return;
    cnt=0;
    uint8_t rqst[1] = {' '};
    HAL_UART_Transmit_IT(&huart6, rqst ,1);

}

void Sonar::Receive(){
    if(RxBuffer[0]== 0xFF) {
        data = RxBuffer[1]*256 + RxBuffer[2] - data_offset;
    }
    else data= -1;

    //HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX); TODO 多设备共用串口资源，需要重构
}

Sonar::Sonar() {
    data_offset=0;
    for(int i=0;i<100;++i){
        Handle();
        HAL_Delay(20);
        if(RxBuffer[0]== 0xFF)   data_offset += (RxBuffer[1]*256 + RxBuffer[2])/100;
    }
}
