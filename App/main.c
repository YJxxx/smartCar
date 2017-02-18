#include "common.h"
#include "include.h"

//������
uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint8 img[CAMERA_H][CAMERA_W];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��

void vcan_sendimg(uint8 *imgaddr, uint32 imgsize);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();

//�Զ���
void camera_pro();
void black_line();
void duoji();
void init();
void duoji_zj();
void black_line_zj();
void duoji_l();
void duoji_r();
void black_line_zx();
void duoji_zx();
void black_line_zx_new();
void PIT_IRQHandler(void);
void PORTA_A_IRQHandler(void);
void proess();
void duoji_init();
void speed();

#define H_MAX_COLUMN 65   //��������������Χ
#define H_START_LINE 50   //ͼ������ĵ�һ��
#define H_END_LINE 30   //ͼ��Ҫ���������һ��   35
#define H_MID_COLUMN 40  //ÿ�е��м�ֵ����Ҫ����
#define ZJ_CHECK   15     //ֱ�Ǹ��м��

#define MOTOR_TPM   TPM0      //�����ͨ��
#define MOTOR1_PWM  TPM_CH0
#define MOTOR2_PWM  TPM_CH1
#define MOTOR3_PWM  TPM_CH2
#define MOTOR4_PWM  TPM_CH3
#define MOTOR_HZ    (20*1000) //���Ƶ��

#define S3010_TPM   TPM1     //�����ͨ��       ����Ѹ�Ϊ1000����
#define S3010_CH    TPM_CH0
#define S3010_HZ    (100)    //���Ƶ��
#define ZYCZ 30        //���Ҳ�ֵ������������ߺ����ұߵ����߲�ֵ,��Ҫ������
#define DJ_ZZ 110             //�����ֵ��152 �����170���ұ���134     201 183 165      150 168 186

#define ZX_NUM (H_START_LINE-H_END_LINE+1-3)
#define ZX_JG 4

int16 left_line[H_MAX_COLUMN];       //���������洢
int16 right_line[H_MAX_COLUMN];      //�ұ�������洢
int16 center_line[H_MAX_COLUMN];     //��������洢
int16 error_mid[H_MAX_COLUMN];       //ÿ����ֵ��ȥʵ������ֵ�ó�ƫ�������֮��
uint8 left[60],right[60],black[60];
uint8 SPEED; 
uint8 SPEED_C; 
uint8 SPEED_Z; 
uint8 SPEED_ZX;

int16 error_mid[H_MAX_COLUMN];       //ÿ����ֵ��ȥʵ������ֵ�ó�ƫ�������֮��
float error_dj=0;                      //һ��ͼ������ƫ���ƽ��ֵ
int16 steer;                        //�������PWMֵ
float kp=14.0/ZYCZ;    //����   36�Ǳ�׼44 30 24   22
uint8 flag_zj=0;
uint8 flag=0;
uint8 flag_change=0;
uint8 black_x;
uint8 flag_dx_l=0,flag_dx_r=0;
uint8 last_left=10;
uint8 last_right=70;
uint8 flag_zx=0;
uint8 flag_zx_d=0,flag_zx_e=0;
uint8 test;
uint8 last_zx;
int16 error_test;
uint8 center_test;
uint8 last_wb,last_bw;
uint8 car_start;
uint8 pulse=1;
float kk=0.58;
int16 test_k;

void main(void)
{   gpio_init (PTC8, GPI,0);
    gpio_init (PTC9, GPI,0);
    gpio_init (PTC10, GPI,0);
    gpio_init (PTC11, GPI,0);
    speed();
    duoji_init();
    disable_irq (PIT_IRQn);
    pit_init_ms(PIT0, 50);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 50ms
    set_vector_handler(PIT_VECTORn ,PIT_IRQHandler);       //����PIT0���жϷ�����Ϊ PIT_IRQHandler
    port_init (PTA14, IRQ_RISING | PF | ALT1 | PULLUP );
    set_vector_handler(PORTA_VECTORn ,PORTA_A_IRQHandler);
    enable_irq (PORTA_IRQn);
    enable_irq (PIT_IRQn);                                 //ʹ��PIT0�ж�                                 
    while(car_start==0)
    {
      
    }
    camera_init(imgbuff);
    //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
    
    init();   
    camera_pro();
}
void speed()
    {
    if(gpio_get (PTC8)==0x00)
     {  
      SPEED=25;
      SPEED_C=26; 
      SPEED_Z=24;                   
      SPEED_ZX=25;

     }
      if(gpio_get (PTC9)==0x00)
      {
      SPEED=30;
      SPEED_C=31; 
      SPEED_Z=22;                   
      SPEED_ZX=30;
      }
     else if(gpio_get (PTC10)==0x00)
      {
      SPEED=33;
      SPEED_C=34; 
      SPEED_Z=21;                   
      SPEED_ZX=33;
      }
   else if(gpio_get (PTC11)==0x00)
    {
      SPEED=36;
      SPEED_C=37; 
      SPEED_Z=22;                   
      SPEED_ZX=36;
      
    }
    }

void vcan_sendimg(uint8 *imgaddr, uint32 imgsize)
{
#define CMD_IMG     1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //ɽ����λ�� ʹ�õ�����
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //ɽ����λ�� ʹ�õ�����

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��

    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
}



void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}


void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 6;                                              //���ж�
    if(flag & (1 << n))                                 //PTA6�����ж�
    {
        camera_vsync();
    }
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}

void camera_pro()
{
    while(1)
    {
      uint8 line ,column;
      camera_get_img();                                   //����ͷ��ȡͼ��
      img_extract((uint8 *)img,(uint8 *) imgbuff, CAMERA_W*CAMERA_H/8);
      if(flag==0)                   
      {
        black_line();
      }
      else
      {
        if(flag==1)
        {
          black_line_zj();
        }
        else
        {
          if(flag==2)
          {
            black_line_zx_new();
          }
        }
      }
      
      for(line=H_START_LINE;line>=H_END_LINE;line--)
      {
        uint8 white;
        if(black[line]>76)
        {
          for(column=2;column<79;column++)
          {
            if(img[line-ZJ_CHECK][column]==255)
              white++;
          }
          if(white>76)
          {
            black_x++;
          }
        }
        black[line]=0;
      }
      
      if(black_x>=1)
      {
        flag_zj=1;       //���ڹ�����
        black_x=0;
      }
      else              
      {
        if(flag==0)         //û����ֱ��
        {
          if(flag_zj==1)    //���ڹ�����
          {
            flag_zj=0;     //�������
            flag=1;            //����ֱ��
          }
        }
        else
        { 
          if(flag==1)
          {
            if(flag_zj==1)
            {
              flag_zj=0;     //�������
              flag=0;            //��ֱ��
            }
          }
        }
      }
      
      if(flag_change==0)
      {
        duoji();
      }
      else
      {
        if(flag_change==1)
        {
          duoji_l();
        }
        else
        {
          if(flag_change==2)
          {
           duoji_r(); 
          }
          else
            if(flag_change==3)
            {
              duoji_zx();
            }
        }       
      }
      

     //vcan_sendimg((uint8 *)img,CAMERA_W*CAMERA_H);
    }
}




void black_line()
{
  uint8 line;
  uint8 column; 
  center_line[H_START_LINE+1]=H_MID_COLUMN;                //����һ�������ұ߽�����ṩѰ�һ���  
  for(line=H_START_LINE;line>=H_END_LINE;line--)          //����Ѱ����߽�
  {
    for(column=8;column<72;column++)
    {
      if(img[line][column]-img[line][column+1]==255)
      {
        left[line]=column;
        break;
      }
    }
    
    for(column=2;column<79;column++)
    {
      if(img[line][column]==0)
      {
        black[line]++;
      }
      if(img[line][column]-img[line][column+1]==-255)
      {
        right[line]=column;
      }
    }
        
    if((left[line]!=0) && (right[line]!=0) && (left[line]-right[line]<0) && (left[line]-right[line]>-10))
    {
      flag_zx++;
      center_line[line]=(left[line]+right[line]+1)/2;
      left[line]=0;
      right[line]=0;
    }
    else
    {
      for(column=center_line[line+1];column>0;column--)
      {
        if(img[line][column]-img[line][column-1]==255)
        {
          left_line[line]=column-1;
          last_left=left_line[line];
          break;                                             //�ҵ���߽�
        } 
        else
        {
          left_line[line]=222;                             //�Ҳ����������־λ
        }
      }
    
      for(column=center_line[line+1];column<79;column++)  //����Ѱ���ұ߽�
      {
        if(img[line][column]-img[line][column+1]==255)
        {
          right_line[line]=column+1;
          last_right=right_line[line];
          break;                                            //�ҵ���߽�
        } 
        else
        {
          right_line[line]=222;                            //�Ҳ����������־λ
        }
      }
       
      if((left_line[line]==222) || (right_line[line]==222))            //��߻����ұ߶���
      {
        if((left_line[line]==222) && (right_line[line]!=222))        //��߶��� 
        {
          left_line[line]=0+(right_line[line]-last_right);
          last_left=left_line[line];
          center_line[line]=(right_line[line]+left_line[line]+1)/2;                  
        }
        else
          if((left_line[line]!=222) && (right_line[line]==222))    //�ұ߶���
          {
            right_line[line]=79+(left_line[line]-last_left);
            last_right=right_line[line];
            center_line[line]=(right_line[line]+left_line[line]+1)/2;                
          }
        else                                           //ȫ����������ʮ��
        {
          center_line[line]=40;                               //ÿ�е��м�ֵ����Ҫ����
        }
      }
      else                                             //ȫ����
      {
        center_line[line]=(right_line[line]+left_line[line]+1)/2;
      }
    }
    img[line][center_line[line]]=0;                           //����λ����������
  }
  
  if(flag_zx>=5)
  {
    flag=2;
    flag_change=3;
  }
  flag_zx=0;
  tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED);
  tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED);
}


void init()
{
    
    tpm_pwm_init(MOTOR_TPM, MOTOR1_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM   ��ͨ�� Сͨ�� Ƶ�� ռ�ձ�
    tpm_pwm_init(MOTOR_TPM, MOTOR2_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
    tpm_pwm_init(MOTOR_TPM, MOTOR3_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
    tpm_pwm_init(MOTOR_TPM, MOTOR4_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
	
    tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED);
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,0);
    tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED);
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,0);
}

void duoji()
{
	uint8 line;
	
	for(line=H_START_LINE;line>=H_END_LINE;line--)
	{
          error_mid[line]=40-center_line[line];                   //���ÿ��ƫ��
          error_dj=error_mid[line]+error_dj;                                //���ƫ���ۼ�
	} 
        error_dj=error_dj/(H_START_LINE-H_END_LINE+1);                      //���һ��ͼ�Ķ��ƫ��
        error_dj=0.0015*error_dj*error_dj*error_dj-0.0154*error_dj+error_dj;   //y=0.0015*x^3-0.0154*x
        //error_dj=(int16)(0.0015*(error_dj/(H_START_LINE-H_END_LINE+1))*(error_dj/(H_START_LINE-H_END_LINE+1))*(error_dj/(H_START_LINE-H_END_LINE+1))-0.0428*(error_dj/(H_START_LINE-H_END_LINE+1))+(error_dj/(H_START_LINE-H_END_LINE+1)));
        test_k=(int16)(kk*(center_line[H_START_LINE]-center_line[H_END_LINE]));
        if(center_line[H_END_LINE]==40)
        {
          test_k=0;
        }
        steer=(uint16)(DJ_ZZ+kp*error_dj);                           //���Ҫ�������PWMֵ
        steer=(uint16)(DJ_ZZ+kp*error_dj+test_k);
        tpm_pwm_duty(S3010_TPM, S3010_CH,steer);                            //�������ֵ


}





void black_line_zj()
{
  uint8 line;
  uint8 column;
  
  center_line[H_START_LINE+1]=H_MID_COLUMN;                //����һ�������ұ߽�����ṩѰ�һ���  
  for(line=H_START_LINE;line>=H_END_LINE;line--)          //����Ѱ����߽�
  {
    for(column=8;column<72;column++)
    {
      if(img[line][column]-img[line][column+1]==255)
      {
        left[line]=column;
        break;
      }
    }
    
    for(column=2;column<79;column++)
    {
      if(img[line][column]==0)
      {
        black[line]++;
      }
      if(img[line][column]-img[line][column+1]==-255)
      {
        right[line]=column;
      }
    }
        
    if((left[line]!=0) && (right[line]!=0) && (left[line]-right[line]<0) && (left[line]-right[line]>-8))
    {
      center_line[line]=(left[line]+right[line]+1)/2;
      left[line]=0;
      right[line]=0;
    }
    else
    {
      for(column=center_line[line+1];column>0;column--)
      {
        if(img[line][column]-img[line][column-1]==255)
        {
          left_line[line]=column-1;
          last_left=left_line[line];
          break;                                             //�ҵ���߽�
        } 
        else
        {
          left_line[line]=222;                             //�Ҳ����������־λ
        }
      }
    
      for(column=center_line[line+1];column<79;column++)  //����Ѱ���ұ߽�
      {
        if(img[line][column]-img[line][column+1]==255)
        {
          right_line[line]=column+1;
          last_right=right_line[line];
          break;                                            //�ҵ��ұ߽�
        } 
        else
        {
          right_line[line]=222;                            //�Ҳ����������־λ
        }
      }
       
      if((left_line[line]==222) || (right_line[line]==222))            //��߻����ұ߶���
      {
        if((left_line[line]==222) && (right_line[line]!=222))        //��߶��� 
        {
          flag_dx_r=0;
          flag_dx_l++;
          left_line[line]=0+(right_line[line]-last_right);
          last_left=left_line[line];
          center_line[line]=(right_line[line]+left_line[line]+1)/2;
        }
        else
          if((left_line[line]!=222) && (right_line[line]==222))    //�ұ߶���
          {
            flag_dx_r++;
            flag_dx_l=0;
            right_line[line]=79+(left_line[line]-last_left);
            last_right=right_line[line];
            center_line[line]=(right_line[line]+left_line[line]+1)/2;                         
          }
        else                                           //ȫ����������ʮ��
        {
          flag_dx_r=0;
          flag_dx_l=0;
          center_line[line]=40;                               //ÿ�е��м�ֵ����Ҫ����
        }
      }
      else                                             //ȫ����
      {
        flag_dx_r=0;
        flag_dx_l=0;
        center_line[line]=(right_line[line]+left_line[line]+1)/2;
      }
    }
    img[line][center_line[line]]=0;                           //����λ����������
  }
  
  if(flag_dx_l>=8)
  {
    if(img[H_END_LINE-15][40]==0)
    {
      flag_change=1;
    }
  }
  else
    if(flag_dx_r>=8)
    {
      if(img[H_END_LINE-15][40]==0)
      {
        flag_change=2;
      }
    }
  else
  {
    flag_change=0;
  }
  flag_dx_l=0;
  flag_dx_r=0;
  tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED_Z);
  tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED_Z);
}


void duoji_zj()
{
	uint8 line;	
	for(line=H_START_LINE;line>H_END_LINE;line--)
	{
          error_mid[line]=H_MID_COLUMN-center_line[line];                   //���ÿ��ƫ��
          error_dj=error_mid[line]+error_dj;                                //���ƫ���ۼ�
	} 
        error_dj=error_dj/(H_START_LINE-H_END_LINE+1);                      //���һ��ͼ�Ķ��ƫ��
        steer=(uint16)(DJ_ZZ+kp*error_dj);                           //���Ҫ�������PWMֵ
        tpm_pwm_duty(S3010_TPM, S3010_CH,steer);                            //�������ֵ
}
 

void duoji_l()
{
  steer=135;
  tpm_pwm_duty(S3010_TPM, S3010_CH,steer);
  
  tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,0);
  tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED_C);
}

void duoji_r()
{
  steer=85;
  tpm_pwm_duty(S3010_TPM, S3010_CH,steer);
  
  tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED_C);
  tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,0);
}

void black_line_zx()
{
  uint8 line;
  uint8 column;
  
  center_line[H_START_LINE+1]=H_MID_COLUMN;                //����һ�������ұ߽�����ṩѰ�һ���  
  for(line=H_START_LINE;line>=H_END_LINE;line--)          //����Ѱ����߽�
  {
    for(column=2;column<79;column++)
    {
      if(img[line][column]-img[line][column+1]==255)
      {
        left[line]=column;
        break;
      }
    }
    
    for(column=2;column<79;column++)
    {
      if(img[line][column]==0)
      {
        black[line]++;
      }
      if(img[line][column]-img[line][column+1]==-255)
      {
        right[line]=column;
      }
    }
        
    if((left[line]!=0) && (right[line]!=0) && (left[line]-right[line]<0) && (left[line]-right[line]>-8))
    {
      flag_zx++;
      center_line[line]=(left[line]+right[line]+1)/2;
      left[line]=0;
      right[line]=0;
    }
    else
    {
      for(column=center_line[line+1];column>0;column--)
      {
        if(img[line][column]-img[line][column-1]==255)
        {
          left_line[line]=column-1;
          last_left=left_line[line];
          break;                                             //�ҵ���߽�
        } 
        else
        {
          left_line[line]=222;                             //�Ҳ����������־λ
        }
      }
    
      for(column=center_line[line+1];column<79;column++)  //����Ѱ���ұ߽�
      {
        if(img[line][column]-img[line][column+1]==255)
        {
          right_line[line]=column+1;
          last_right=right_line[line];
          break;                                            //�ҵ���߽�
        } 
        else
        {
          right_line[line]=222;                            //�Ҳ����������־λ
        }
      }
       
      if((left_line[line]==222) || (right_line[line]==222))            //��߻����ұ߶���
      {
        if((left_line[line]==222) && (right_line[line]!=222))        //��߶��� 
        {
          left_line[line]=0+(right_line[line]-last_right);
          last_left=left_line[line];
          center_line[line]=(right_line[line]+left_line[line]+1)/2;                  
        }
        else
          if((left_line[line]!=222) && (right_line[line]==222))    //�ұ߶���
          {
            right_line[line]=79+(left_line[line]-last_left);
            last_right=right_line[line];
            center_line[line]=(right_line[line]+left_line[line]+1)/2;                
          }
        else                                           //ȫ����������ʮ��
        {
          flag_zx_d++;
          center_line[line]=40;                               //ÿ�е��м�ֵ����Ҫ����
        }
      }
      else                                             //ȫ����
      {
        center_line[line]=(right_line[line]+left_line[line]+1)/2;
      }
    }
    img[line][center_line[line]]=0;                           //����λ����������
  }
  
  if(flag_zx==0)
  {
    flag=0;
    flag_change=0;
  }
  flag_zx=0;
  //tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED_ZX);
  //tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED_ZX); 
}

void duoji_zx()
{
  	uint8 line;
	
	for(line=H_START_LINE;line>=H_END_LINE;line--)
	{
          error_mid[line]=40-center_line[line];                   //���ÿ��ƫ��
          error_dj=error_mid[line]+error_dj;                                //���ƫ���ۼ�
	} 
        //error_dj=error_dj/(H_START_LINE-H_END_LINE+1);
        error_dj=(int16)(error_dj/(H_START_LINE-H_END_LINE+1-flag_zx_d));                      //���һ��ͼ�Ķ��ƫ��
        //error_test=error_dj;
        //test=H_START_LINE-H_END_LINE+1-flag_zx_d;
        flag_zx_d=0;
        if(error_dj>6 || error_dj<-6)
        {
          tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED_ZX);
          tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED_ZX);
        }
            
        /*
        if(error_dj<=3 && error_dj>=-3)
        {
          tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,SPEED);
          tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,SPEED); 
        }
        else
          if(error_dj>=18 || error_dj<=-18)
        {
          tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,12);
          tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,12);
        }
        else
          if(error_dj>=8 || error_dj<=-8)
        {
          tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,18);
          tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,18);        
        }
        else
           if(error_dj>3 || error_dj<-3)
        {
          tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,22);
          tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,22);
        }  
        if(steer>128)
        {
          steer=128;
        }
        if(steer<92)
        {
          steer=92;
        }
        */
        steer=(uint16)(DJ_ZZ+kp*error_dj);                           //���Ҫ�������PWMֵ
        tpm_pwm_duty(S3010_TPM, S3010_CH,steer);                            //�������ֵ

}

void black_line_zx_new()
{
  uint8 line;
  uint8 column;
  center_line[H_START_LINE+1]=H_MID_COLUMN;
  for(column=20;column<60;column++)
  {
    if(img[H_START_LINE+2][column]-img[H_START_LINE+2][column+1]==255)
    {
      last_wb=column;
      break;
    }
  }
  for(column=20;column<60;column++)
  {
    if(img[H_START_LINE+2][column]-img[H_START_LINE+2][column+1]==-255)
    {
      last_bw=column+1;
      break;
    }
  }
  
  for(line=H_START_LINE;line>=H_END_LINE;line--)
  {
    for(column=last_wb-ZX_JG;column<last_bw+ZX_JG;column++)
    {
      if(img[line][column]-img[line][column+1]==255)
      {
        left[line]=column;
        last_wb=left[line];
        break;
      }
    }
    
    for(column=last_wb-ZX_JG;column<last_bw+ZX_JG;column++)
    {
      if(img[line][column]-img[line][column+1]==-255)
      {
        right[line]=column;
        last_bw=right[line]+1;
        break;
      }
    }
    
    if((left[line]!=0) && (right[line]!=0) && (left[line]-right[line]<0) && (left[line]-right[line]>-8))
    {
      center_line[line]=(left[line]+right[line]+1)/2;
      left[line]=0;
      right[line]=0;
    }
    else
    {
      for(column=center_line[line+1];column>0;column--)
      {
        if(img[line][column]-img[line][column-1]==255)
        {
          left_line[line]=column-1;
          last_left=left_line[line];
          break;                                             //�ҵ���߽�
        } 
        else
        {
          left_line[line]=222;                             //�Ҳ����������־λ
        }        
      }
      
      for(column=center_line[line+1];column<79;column++)  //����Ѱ���ұ߽�
      {
        if(img[line][column]-img[line][column+1]==255)
        {
          right_line[line]=column+1;
          last_right=right_line[line];
          break;                                            //�ҵ���߽�
        } 
        else
        {
          right_line[line]=222;                            //�Ҳ����������־λ
        }
      }
      
      if((left_line[line]==222) || (right_line[line]==222))            //��߻����ұ߶���
      {
        if((left_line[line]==222) && (right_line[line]!=222))        //��߶��� 
        {
          flag_zx_e++;
          left_line[line]=0+(right_line[line]-last_right);
          last_left=left_line[line];
          center_line[line]=(right_line[line]+left_line[line]+1)/2;                  
        }
        else
          if((left_line[line]!=222) && (right_line[line]==222))    //�ұ߶���
          {
            flag_zx_e++;
            right_line[line]=79+(left_line[line]-last_left);
            last_right=right_line[line];
            center_line[line]=(right_line[line]+left_line[line]+1)/2;                
          }
        else                                           //ȫ����������ʮ��
        {
          flag_zx_e=0;
          flag_zx_d++;
          center_line[line]=40;                               //ÿ�е��м�ֵ����Ҫ����
        }
      }
      else                                             //ȫ����
      {
        flag_zx_e++;
        center_line[line]=(right_line[line]+left_line[line]+1)/2;
      }
    }
    img[line][center_line[line]]=0;
  }
  
  //һ��ͼ��
  if(flag_zx_e>=3)
  {
    flag=0;
    flag_change=0;
  }
  flag_zx_e=0;
}

void PIT_IRQHandler(void)
{

    if(PIT_TFLG(PIT0) == 1 )        //�ж��Ƿ� PIT0 �����ж�
    {
        if(pulse==0)
        {
          car_start=1;
        }
        pulse=0;

        PIT_Flag_Clear(PIT0);       //���жϱ�־λ
    }
}

void PORTA_A_IRQHandler(void)
{

      // �������룬���ַ����ɹ�ѡ��

    uint8  n = 0;    //���ź�
    n = 14;
    if(PORTA_ISFR & (1 << n))           //PTC9 �����ж�
    {
        PORTA_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */

        proess();

        /*  ����Ϊ�û�����  */
    }
}

void proess()
{
  pulse++;
}

void duoji_init()
{
  tpm_pwm_init(S3010_TPM, S3010_CH,S3010_HZ,DJ_ZZ);      //��ʼ�� ��� PWM
}