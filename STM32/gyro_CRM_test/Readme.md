# gyro_CRM_test
Silicon Sensing�А���CRM100�����CRM200�̃e�X�g�v���O�����ł��B    

## ����m�F��
 - IDE CubeIDE v1.0.1  
 - Board NUCLEO-F446RE  

## �z��
Arduino-compatible headers���g�p  
�W���C���Ƃ̒ʐM��SPI�o�X��SS��GPIO��3�s��  
�W���C���̃��Z�b�g��GPIO��3�s��
 - SPI3  
	- D3 (PB3)   SCLK  
    - D4 (PB5)   MOSI  
    - D5 (PB4)   MISO  
    - D8 (PA9)   SS_P  
    - D9 (PC7)   SS_R  
    - D10(PB6)   SS_Y  

- RESET  
  - D11 (PA7)  reset_P  
  - D12 (PA6)  reset_R  
  - D13 (PA5)  reset_Y  

��P:pitch R:roll Y:yaw  

USART2��W���o�͂Ɋ��蓖��(=���ɉ������Ȃ��Ă�print�f�o�b�O���\)
 
## ����  
### SPI �ʐM���x  
�N���b�N:100KHz�`8MHz (�W��:1MHz)  
�����703.125KBits/s�Ŏ���

���[�h�F1  
CPOL:LOW  
CPHA:1Edge  

1�̃��b�Z�[�W��6Byte48Bit�Ŋ�������B  
�e�o�C�g��command�� DATA0���c��DATA3��CHECKSUM�̏�  

### ���C�u�����̎g����  
 1. ```CRMx00_t``` �\���̂��f�o�C�X�̐������錾
 1. ```SetCRM_port``` ��SS��RESET�ASPI�̃|�[�g�ƃs�����w��
 1. ```SetCRM_range``` �Ń����W���w��
 1. ```UpdateCRM``` �Ńf�[�^���X�V
 1. ```CRMx00_t.temperature_f``` �Ȃǂ̒l��ǂ�

### �֐����t�@�����X
(��ŏ���)