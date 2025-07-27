#include <ch32v00x.h>
#include <ch32v00x_gpio.h>
#include <stdio.h> // Include stdio.h for sprintf

// Defining the SDA and SCL Pins for I2C Communication
#define SDA_PIN GPIO_Pin_1
#define SCL_PIN GPIO_Pin_2

// Defining the LCD_Address 
#define LCD_Address 0x27

void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_str(signed char *str);
void lcd_init(void);
void delay_ms(unsigned int ms);

// Function to produce a delay
void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; i++) {
        for (unsigned int j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

void USARTx_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

// Function to initialize GPIO pins
void GPIO_INIT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

    // Initialize SDA and SCL pins for I2C
    GPIO_InitStructure.GPIO_Pin = SDA_PIN | SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// Function to write a byte of data to the I2C bus
void i2c_write(unsigned char dat) {
    for (unsigned char i = 0; i < 8; i++) {
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
        if (dat & (0x80 >> i)) {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
        } else {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
        }
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    }
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to start I2C communication
void i2c_start(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to stop I2C communication
void i2c_stop(void) {
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
}

// Function to wait for an acknowledgment bit
void i2c_ACK(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    while(GPIO_ReadInputDataBit(GPIOC, SDA_PIN));
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to send a command to the LCD
void lcd_send_cmd(unsigned char cmd) {
    unsigned char cmd_l = (cmd << 4) & 0xf0;
    unsigned char cmd_u = cmd & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(cmd_u | 0x0C);
    i2c_ACK();
    i2c_write(cmd_u | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_write(cmd_l | 0x0C);
    i2c_ACK();
    i2c_write(cmd_l | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send data to the LCD
void lcd_send_data(unsigned char data) {
    unsigned char data_l = (data << 4) & 0xf0;
    unsigned char data_u = data & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(data_u | 0x0D);
    i2c_ACK();
    i2c_write(data_u | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_write(data_l | 0x0D);
    i2c_ACK();
    i2c_write(data_l | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send a string to the LCD
void lcd_send_str(signed char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Function to initialize the LCD
void lcd_init(void) {
    lcd_send_cmd(0x02); // Return home
    lcd_send_cmd(0x28); // 4-bit mode, 2 lines, 5x7 dots
    lcd_send_cmd(0x0C); // Display On, cursor off
    lcd_send_cmd(0x06); // Increment cursor (shift cursor to right)
    lcd_send_cmd(0x01); // Clear display
    delay_ms(20); // Wait for the LCD to process the clear command
}

// Function to set pin mode dynamically
void set_pin_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void receive() {
    char buffer[20]; // Buffer to store received data
    uint8_t index = 0;
    int temperature = 0, emg_val = 0, emg_env = 0;
    uint8_t startParsing = 0; // Flag to indicate when to start storing data

    while (1) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // Wait for data

        char receivedChar = USART_ReceiveData(USART1); // Read received character

        if (receivedChar == 'x') { 
            startParsing = 1; // Start storing data once 'x' is found
            index = 0; // Reset index to store only meaningful data
        }
        
        if (receivedChar == 'o') { // End of data (carriage return)
            buffer[index] = '\0'; // Null-terminate the string
            break;
        }

        if (startParsing) { // Store only after 'x' is found
            if (index < sizeof(buffer) - 1) {
                buffer[index++] = receivedChar;
            }
        }
    }

    // Extract values from the cleaned buffer (Expected format: x30y59z80)
    sscanf(buffer, "x%d y%d z%d", &temperature, &emg_val, &emg_env);

    lcd_send_cmd(0x01); // Clear display
    delay_ms(20);

    // Display Temperature
    lcd_send_cmd(0x80); // Move cursor to first row, first column
    lcd_send_str((signed char *)"****Scanning****");
    delay_ms(1000);

    // Display Temperature
    lcd_send_cmd(0x01);
    delay_ms(20);
    lcd_send_cmd(0x80); // Move cursor to first row, first column
    lcd_send_str((signed char *)"Temperature: ");
    char tempStr[5];
    sprintf(tempStr, "%d", temperature);
    lcd_send_str((signed char *)tempStr);
    delay_ms(1000);

    // Display Pressure
    lcd_send_cmd(0x01);
    delay_ms(20);
    lcd_send_cmd(0x80); // Move cursor to second row, first column
    lcd_send_str((signed char *)"EMG_Val: ");
    char emg_val_Str[5];
    sprintf(emg_val_Str, "%d", emg_val);
    lcd_send_str((signed char *)emg_val_Str);
    delay_ms(1000);

    // Clear display and show Altitude
    lcd_send_cmd(0x01);
    delay_ms(20);
    lcd_send_cmd(0x80);
    lcd_send_str((signed char *)"EMG_Env: ");
    char emg_env_Str[5];
    sprintf(emg_env_Str, "%d", emg_env);
    lcd_send_str((signed char *)emg_env_Str);
    //delay_ms(1000);
}

int main(void) {
    SystemCoreClockUpdate();
    USARTx_CFG();

    GPIO_INIT(); // Initialize the GPIO pins
    delay_ms(20);

    // Initialize the LCD Display
    lcd_init();
    delay_ms(20);

    while (1) {
        receive();
    }
}
