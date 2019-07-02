# gyro_CRM_test
Silicon Sensing社製のCRM100およびCRM200のテストプログラムです。    

## 動作確認環境
 - IDE CubeIDE v1.0.1  
 - Board NUCLEO-F446RE  

## 配線
Arduino-compatible headersを使用  
ジャイロとの通信はSPIバスとSSにGPIOを3ピン  
ジャイロのリセットにGPIOを3ピン
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

※P:pitch R:roll Y:yaw  

USART2を標準出力に割り当て(=特に何もしなくてもprintデバッグが可能)
 
## メモ  
### SPI 通信速度  
クロック:100KHz～8MHz (標準:1MHz)  
今回は656.25KBits/sで実装

モード：1  
CPOL:LOW  
CPHA:1Edge  

1つのメッセージは6Byte48Bitで完結する。  
各バイトはcommand→ DATA0→…→DATA3→CHECKSUMの順  

### 関数 