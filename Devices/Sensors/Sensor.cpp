/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Sensor.h"
#include <cmath>
#include "Matix.h"

extern I2C_HandleTypeDef hi2c2;

SensorGroup SensorGroup::sensorGroup;


void SensorGroup::Handle(){
    //for (int i = 0; i < SENSOR_NUM; ++i) {
    //    if (! pressureSensors.at(i).IsReady()) return;//如有任一传感器数据未就位，则返回
    //}
    //for (int i = 0; i < SENSOR_NUM; ++i) {
    //    data_pressure[i] = pressureSensors.at(i).GetData();
    //}
    if (!(pressureSensor0.IsReady() && pressureSensor1.IsReady() && pressureSensor2.IsReady() && pressureSensor3.IsReady())){
        return;
    }
    data_pressure[0] = pressureSensor0.GetData();
    data_pressure[1] = pressureSensor1.GetData();
    data_pressure[2] = pressureSensor2.GetData();
    data_pressure[3] = pressureSensor3.GetData();

    Update_plane();//计算机器人参考系下水面方程
    data_depth=-data_plane[3];//计算深度
    pitch=atan(-data_plane[0]/(sqrt(data_plane[1]*data_plane[1]+data_plane[2]*data_plane[2])));//计算俯仰角
    roll=atan(data_plane[1]/data_plane[2]);//计算横滚角
}

void SensorGroup::Solve_plane_3(float* data, float* h, float* x, float* y, float* z){
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


void SensorGroup::Solve_plane(float* data, float* h, float* x, float* y, float* z, int num){
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


void SensorGroup::Update_plane(){
    Solve_plane(data_plane,data_pressure,site.x,site.y,site.z,SENSOR_NUM);

    for(int i=0;i<3;++i){
        data_level[i]=data_plane[i];
    }
}
