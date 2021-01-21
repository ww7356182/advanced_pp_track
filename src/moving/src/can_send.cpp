#include<canlib.h>
#include<stdio.h>
#include <ros/ros.h>
#include <endian.h>

int  main(int argc,char **argv)
{
    ros::init(argc, argv, "can_send");
    ros::NodeHandle n_; 
    
    canHandle h;
    canInitializeLibrary();
    h=canOpenChannel(0,canWANT_EXCLUSIVE);
    if(h != canOK)
    {
        char msg[64];
        canGetErrorText((canStatus)h,msg,sizeof(msg));
        fprintf(stderr,"canopenchannel fialied(%s)\n",msg);
        exit(1);

    }
    canSetBusParams(h,BAUD_1M,0,0,0,0,0);
    canBusOn(h);

    long id;
    id=0x121;
    long int msg,msg_1,msg_2,msg_3;
    int16_t angle;
    angle=500;
    angle=htobe16(angle);
    char buffer[8];
    unsigned int dlc;
    dlc=8;
    unsigned int flag;
    flag=0;
    for(int i=0;i<8;i++)
    {
    buffer[i]=i;
    }
    buffer[0]=10;
    buffer[1]=12;
    buffer[2]=15;
    buffer[3]=19;
    buffer[4]=24;
    buffer[5]=30; 
    buffer[6]=37;
    buffer[7]=01;
    
    canWrite(h,id,&buffer,dlc,flag);
    
    ROS_INFO("send");
    canWriteSync(h,500);
    canBusOff(h);   
    canClose(h); 
    return 0;
}
