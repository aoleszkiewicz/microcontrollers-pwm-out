/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// rozmiar bufora na ramke
#define ADDRESS_LENGTH 3
#define PARAM_LENGTH_DIGITS 3
#define COMMAND_LENGTH 4
#define MAX_PARAM_LENGTH 700
#define CRC_MAX_LENGTH 4

// maksymalna liczba wpisow w buforze wzorcow
#define MAX_PATTERN_COUNT 100
// ile cyfr ma czestotliwosc (0000-9999)
#define FREQ_DIGITS 4
// ile cyfr ma wypelnienie (000-100 %)
#define DUTY_DIGITS 3
// ile cyfr ma okres (0000-9999)
#define PERIOD_DIGITS 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// bufory na odbior i wysylanie przez UART
#define USART_TXBUF_LEN 719 //Liczba bajtów bufora nadawczego
#define USART_RXBUF_LEN 1432 //Liczba bajtów bufora odbiorczego
uint8_t USART_TxBuf[USART_TXBUF_LEN];//tworzenie bufora nadawczego
uint8_t USART_RxBuf[USART_RXBUF_LEN];//tworzenie bufora odbiorczego

__IO int USART_TX_Empty=0; //wskaźniki zapisu (empty) i odczytu(busy)
__IO int USART_TX_Busy=0;
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;

// stany maszyny stanow odbierajacej ramke
typedef enum
{
    WAIT_FOR_HEADER,    // czekamy na znak '{'
    READ_SENDER_ADDR,   // czytamy adres nadawcy (np. PC1)
    READ_RECEIVER_ADDR, // czytamy adres odbiorcy (STM)
    READ_PARAM_LENGTH,  // czytamy dlugosc parametrow (np. 007)
    READ_COMMAND,       // czytamy komende (np. CPWM)
    READ_PARAMETERS,    // czytamy parametry
    READ_CRC,           // czytamy sume kontrolna
    WAIT_FOR_FOOTER     // czekamy na znak '}'
} FrameState;

// aktualny stan maszyny stanow
FrameState state = WAIT_FOR_HEADER;
// czy nastepny znak jest "escaped" (byte stuffing)
uint8_t escape_next = 0;
// zbieramy tu sume kontrolna CRC
uint16_t crc_collector = 0xFFFF;
// ile bajtow ma cala ramka
uint16_t frame_length = 0;

// pomocnicze zmienne do parsowania
uint16_t param_length = 0; // dlugosc parametrow jako liczba
uint16_t data_index = 0;   // indeks w aktualnie czytanym polu

// bufor na "odkodowane" dane (bez byte stuffing)
uint8_t raw_data[719];
uint16_t raw_length = 0;

// bufory na pola ramki
uint8_t sender_address[ADDRESS_LENGTH];                 // adres nadawcy
uint8_t receiver_address[ADDRESS_LENGTH];               // adres odbiorcy
uint8_t param_length_digits[PARAM_LENGTH_DIGITS]; // dlugosc parametrow
uint8_t command[COMMAND_LENGTH];                  // komenda
uint8_t parameters[MAX_PARAM_LENGTH];             // parametry
uint8_t crc_bytes[CRC_MAX_LENGTH];                // suma kontrolna

// TODO: zmienic?? tablica znakow np. "ACKN" dla zwracanej komendy
char return_command[COMMAND_LENGTH];

// struktura na konfiguracje PWM
typedef struct
{
    uint16_t frequency; // czestotliwosc w Hz
    uint8_t duty;       // wypelnienie 0-100%
} PWM_Config;

// struktura na jeden wpis wzorca
typedef struct
{
    uint16_t period; // czestotliwosc
    uint8_t duty;    // wypelnienie 0-100%
} Pattern_Entry;

// aktualna konfiguracja PWM
PWM_Config current_pwm = {0, 0};
// bufor na wzorce
Pattern_Entry pattern_buffer[MAX_PATTERN_COUNT];
// ile wzorcow jest w buforze
uint16_t pattern_count = 0;
// czy DMA dziala
uint8_t dma_running = 0;

// bufor na wartosci PWM dla DMA
uint32_t pwm_buffer[MAX_PATTERN_COUNT];
// okres timera
uint32_t timer_period = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//uint8_t USART_kbhit() { // Sprawdzenie czy tablica przypadkiem nie jest pusta
//    if (USART_RX_Empty == USART_RX_Busy) {
//        return 0;
//    } else {
//        return 1;
//    }
//} // USART_kbhit

int16_t USART_getchar() { // Pobranie znaku
    int16_t tmp;
    if (USART_RX_Empty != USART_RX_Busy) { // Sprawdzenie czy tablica przypadkiem nie jest pusta
        tmp = USART_RxBuf[USART_RX_Busy]; // Odczyt
        USART_RX_Busy++; // Przejście na kolejny indeks odczytu
        if (USART_RX_Busy >= USART_RXBUF_LEN) {
            USART_RX_Busy = 0; // Sprawdzenie czy nie przekroczyło długości bufora odczytu
        }
        return tmp;
    } else {
        return -1;
    }
} // USART_getchar
//
//uint8_t USART_getline(char *buf) {
//    static uint8_t bf[128];
//    static uint8_t idx = 0;
//    int i;
//    uint8_t ret;
//
//    while (USART_kbhit()) { // Sprawdza czy wskaźnik odczytu i zapisu są na tym samym miejscu
//        bf[idx] = USART_getchar();
//        if ((bf[idx] == 10) || (bf[idx] == 13)) {
//            bf[idx] = 0;
//            for (i = 0; i <= idx; i++) {
//                buf[i] = bf[i]; // Przekopiuj do bufora
//            }
//            ret = idx;
//            idx = 0;
//            return ret; // Odebrano linię
//        } else { // Jeśli tekst dłuższy, to zawijamy - trudno
//            idx++;
//            if (idx >= 128) {
//                idx = 0;
//            }
//        }
//    }
//    return 0;
//} // USART_getline

void USART_fsend(char *format, ...) {
    char tmp_rs[719];
    int i;
    __IO int idx;
    va_list arglist;

    va_start(arglist, format);
    vsprintf(tmp_rs, format, arglist);
    va_end(arglist);

    idx = USART_TX_Empty;
    for (i = 0; i < strlen(tmp_rs); i++) {
    	USART_TxBuf[idx] = tmp_rs[i];
		idx++;
		if (idx >= USART_TXBUF_LEN) {
			idx = 0;
		}
    }

    __disable_irq(); // Wyłączamy przerwania
    if ((USART_TX_Empty == USART_TX_Busy) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
        // Sprawdzić dodatkowo zajętość bufora nadajnika
        USART_TX_Empty = idx;
        uint8_t tmp = USART_TxBuf[USART_TX_Busy];
        USART_TX_Busy++;
        if (USART_TX_Busy >= USART_TXBUF_LEN) {
            USART_TX_Busy = 0;
        }
        HAL_UART_Transmit_IT(&huart2, &tmp, 1);
    } else {
        USART_TX_Empty = idx;
    }
    __enable_irq();
} // USART_fsend

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        if (USART_TX_Empty != USART_TX_Busy) {
            uint8_t tmp = USART_TxBuf[USART_TX_Busy];
            USART_TX_Busy++;
            if (USART_TX_Busy >= USART_TXBUF_LEN) {
                USART_TX_Busy = 0;
            }
            HAL_UART_Transmit_IT(&huart2, &tmp, 1);
            // Informacja zwrotna - informuję o tym, że dany bajt został już wysłany
            // i że można odebrać następny
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        USART_RX_Empty++;
        if (USART_RX_Empty >= USART_RXBUF_LEN) {
            USART_RX_Empty = 0;
        }
        HAL_UART_Receive_IT(&huart2, &USART_RxBuf[USART_RX_Empty], 1);
    }
}

void reset_state();
uint8_t get_decoded_character(uint8_t character);
void FrameRead();
void ProcessCommand(uint8_t *sender_addr, uint8_t *command, uint8_t *parameters, uint16_t param_length);
uint16_t crc16_ccitt_false(uint16_t crc, uint8_t *data, uint16_t length);
bool StartPWMGeneration();
bool StopPWMGeneration();
bool ConfigurePWM(PWM_Config *config);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

//    HAL_UART_Receive_IT(&huart2, &BUF_RX[0], 1);
    HAL_UART_Receive_IT(&huart2,&USART_RxBuf[0], 1);
    USART_fsend("\n\rREADY\n\r");

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    	FrameRead();
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_TIM2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void reset_state()
{
	state = WAIT_FOR_HEADER;
	escape_next = 0;
	crc_collector = 0xFFFF;
	frame_length = 0;
	param_length = 0;
	data_index = 0;
	raw_length = 0;
}

uint8_t get_decoded_character(uint8_t character)
{
	switch (character)
	{
		case '[':
			return '{';
			break;
		case ']':
			return '}';
			break;
		case '^':
			return '~';
			break;
		default:
			return 0;
	}
}

void FrameRead()
{
    int16_t temp_char;
    while ((temp_char = USART_getchar()) >= 0)
    {
        uint8_t character = (uint8_t) temp_char;

        if (character == 0)
		{
			USART_fsend("NULL DETECTED: \\x%02X\n\r", character);
			return;
		}

        if (frame_length == 0 && character != '{') {
			return;
		}

        if (frame_length > 0 && character == '{')
		{
			reset_state();
		}

        if (frame_length > 1432)
        {
        	reset_state();
        	return;
        }

        if (frame_length > 0 && frame_length < 18 && character == '}')
        {
			USART_fsend("est");
        	reset_state();
        	return;
        }

        if (frame_length > 0 && character == '~')
        {
        	escape_next = 1;
        	return;
        }

        if (escape_next)
        {
        	uint8_t decoded_character = get_decoded_character(character);

        	if (decoded_character == 0)
        	{
        		USART_fsend("{STM%s000BTER}", sender_address);
        		reset_state();
        		return;
        	}

        	escape_next = 0;
        	character = decoded_character;
        }

		if (state != WAIT_FOR_HEADER && state != READ_CRC && state != WAIT_FOR_FOOTER)
		{
			raw_data[raw_length++] = character;
		}

        switch (state)
        {
			case WAIT_FOR_HEADER:
				frame_length++;
				state = READ_SENDER_ADDR;
				break;

			case READ_SENDER_ADDR:
				if (data_index >= ADDRESS_LENGTH)
				{
					return;
				}

				frame_length++;
				sender_address[data_index++] = character;

				if (data_index == ADDRESS_LENGTH)
				{
					data_index = 0;
					state = READ_RECEIVER_ADDR;
				}

				break;

			case READ_RECEIVER_ADDR:
				if (data_index >= ADDRESS_LENGTH)
				{
					return;
				}

				frame_length++;
				receiver_address[data_index++] = character;

				if (data_index == ADDRESS_LENGTH)
				{
					if (memcmp(receiver_address, "STM", ADDRESS_LENGTH) != 0)
					{
						reset_state();
						return;
					}

					data_index = 0;
					state = READ_PARAM_LENGTH;
				}

				break;

			case READ_PARAM_LENGTH:
				if (data_index >= PARAM_LENGTH_DIGITS)
				{
					return;
				}

				if (character < '0' || character > '9')
				{
					reset_state();
					return;
				}

				frame_length++;
				param_length_digits[data_index++] = character;

				if (data_index == PARAM_LENGTH_DIGITS)
				{
					param_length = atoi(param_length_digits);

					if (param_length > MAX_PARAM_LENGTH)
					{
						USART_fsend("{STM%s000DLNE}", sender_address);
						reset_state();
						return;
					}

					data_index = 0;
					state = READ_COMMAND;
				}

				break;

			case READ_COMMAND:
				if (data_index >= COMMAND_LENGTH)
				{
					return;
				}

				frame_length++;
				command[data_index++] = character;

				if (data_index == COMMAND_LENGTH)
				{
					data_index = 0;
					state = param_length == 0 ? READ_CRC : READ_PARAMETERS;
				}
				break;

			case READ_PARAMETERS:
				if (data_index >= param_length)
				{
					USART_fsend("{STM%s000DLNE}", sender_address);
					reset_state();
					return;
				}

				if (character < '0' || character > '9')
				{
					reset_state();
					return;
				}

				frame_length++;
				parameters[data_index++] = character;

				if (data_index == param_length)
				{
					data_index = 0;
					state = READ_CRC;
				}

				break;

			case READ_CRC:
				if (data_index >= CRC_MAX_LENGTH)
				{
					reset_state();
					return;
				}

				if (!((character >= '0' && character <= '9') || (character >= 'A' && character <= 'F') ||
					  (character >= 'a' && character <= 'f')))
				{
					reset_state();
					return;
				}

				frame_length++;
				crc_bytes[data_index++] = character;

				if (data_index == CRC_MAX_LENGTH)
				{
					data_index = 0;
					state = WAIT_FOR_FOOTER;
				}

				break;

			case WAIT_FOR_FOOTER:
				if (character != '}')
				{
					reset_state();
					return;
				}

				frame_length++;

				crc_collector = crc16_ccitt_false(0xFFFF, raw_data, raw_length);
				uint16_t received_crc = (uint16_t)strtol(crc_bytes, NULL, 16);

				if (crc_collector != received_crc)
				{
					USART_fsend("{STM%s000CRCE}", sender_address);
					// debug fsend
					USART_fsend("\n\rCRC %04X\n\r", crc_collector);
					reset_state();
					return;
				}

				ProcessCommand(sender_address, command, parameters, param_length);
				reset_state();

				break;

			default:
				reset_state();
				break;
        }
    }
}

void ProcessCommand(uint8_t *sender_addr, uint8_t *command, uint8_t *parameters, uint16_t param_length)
{
    if (strncmp(command, "CPWM", COMMAND_LENGTH) == 0)
    {
        // sprawdz czy mamy dokladnie 7 znakow (4 freq + 3 duty)
        if (param_length != 7)
        {
        	USART_fsend("{STM%s000PWME}", sender_address);
            return;
        }

        char freq_str[FREQ_DIGITS + 1];
        memcpy(freq_str, parameters, FREQ_DIGITS);
        freq_str[FREQ_DIGITS] = '\0'; // null-terminator

        char duty_str[DUTY_DIGITS + 1];
        memcpy(duty_str, parameters + FREQ_DIGITS, DUTY_DIGITS);
        duty_str[DUTY_DIGITS] = '\0'; // null-terminator

        uint16_t freq = atoi(freq_str); // np. "2000" -> 2000
        uint8_t duty = atoi(duty_str);  // np. "050" -> 50

        if (freq < 1 || freq > 9999 || duty > 100 || duty < 0)
        {
        	USART_fsend("{STM%s000PWME}", sender_address);
            return;
        }

        // zapisz nowa konfiguracje
        current_pwm.frequency = freq;
        current_pwm.duty = duty;

        // sprobuj skonfigurowac timer
        if (!ConfigurePWM(&current_pwm))
        {
        	USART_fsend("{STM%s000PWME}", sender_address);
            return;
        }

        USART_fsend("{STM%s000ACKN}", sender_address);
    }
    else if (strncmp(command, "SPTR", COMMAND_LENGTH) == 0)
    {
        // sprawdz czy liczba znakow jest ok (wielokrotnosc 7)
        if (param_length % 7 != 0 ||
            param_length / 7 > 100)
        {
        	USART_fsend("{STM%s000PTNE}", sender_address);
            return;
        }

        // policz ile wzorcow przyszlo
        pattern_count = param_length / 7;

        // dla kazdego wzorca
        for (uint16_t i = 0; i < pattern_count; i++)
        {
            // wez 4 znaki na okres
            char period_str[PERIOD_DIGITS + 1];
            memcpy(period_str, &parameters[i * 7], PERIOD_DIGITS);
            period_str[PERIOD_DIGITS] = '\0';

            // wez 3 znaki na wypelnienie
            char duty_str[DUTY_DIGITS + 1];
            memcpy(duty_str, &parameters[i * 7 + PERIOD_DIGITS], DUTY_DIGITS);
            duty_str[DUTY_DIGITS] = '\0';

            // przerob na liczby
            uint16_t period = atoi(period_str);
            uint8_t duty = atoi(duty_str);

            // sprawdz wartosci
            if (period < 1 || period > 9999 || duty > 100 || duty < 0)
            {
            	USART_fsend("{STM%s000PTNE}", sender_address);
                return;
            }

            // zapisz wzorzec
            pattern_buffer[i].period = period;
            pattern_buffer[i].duty = duty;
        }

        USART_fsend("{STM%s000ACKN}", sender_address);
    }
    else if (strncmp(command, "GCPW", COMMAND_LENGTH) == 0)
    {
        if (param_length != 0)
        {
        	USART_fsend("{STM%s000PARE}", sender_address);
            return;
        }

        if (current_pwm.frequency == 0 && current_pwm.duty == 0)
        {
        	USART_fsend("{STM%s000NSDT}", sender_address);
        	return;
        }

        USART_fsend("{STM%s007ACKN%04lu%03u}", sender_address, current_pwm.frequency, current_pwm.duty);
    }
    else if (strncmp(command, "STRT", COMMAND_LENGTH) == 0)
    {
    	if (dma_running == 1)
		{
			return;
		}

    	if (param_length != 0)
		{
			USART_fsend("{STM%s000PARE}", sender_address);
			return;
		}

        if (StartPWMGeneration())
        {
            dma_running = 1;
            USART_fsend("{STM%s000ACKN}", sender_address);
        }
        else
        {
        	USART_fsend("{STM%s000DMER}", sender_address);
        }
    }
    else if (strncmp(command, "STOP", COMMAND_LENGTH) == 0)
    {
    	if (param_length != 0)
		{
			USART_fsend("{STM%s000PARE}", sender_address);
			return;
		}

    	if (dma_running == 0)
    	{
    		USART_fsend("{STM%s000DMSE}", sender_address);
    		return;
    	}

        if (StopPWMGeneration())
        {
            dma_running = 0;
            USART_fsend("{STM%s000ACKN}", sender_address);
        }
        else
        {
        	USART_fsend("{STM%s000DMSE}", sender_address);
        }
    }
    else if (strncmp(command, "STAT", COMMAND_LENGTH) == 0)
    {
        if (param_length != 0)
        {
        	USART_fsend("{STM%s000PARE}", sender_address);
            return;
        }

        if (dma_running == 1)
        {
        	USART_fsend("{STM%s000ACTV}", sender_address);
        }
        else
        {
        	USART_fsend("{STM%s000NOAC}", sender_address);
        }
    }
    else if (strncmp(command, "GPTR", COMMAND_LENGTH) == 0)
    {
        if (param_length != 0)
        {
        	USART_fsend("{STM%s000PARE}", sender_address);
            return;
        }

        // jesli nie ma zadnych wzorcow
        if (pattern_count == 0)
        {
        	USART_fsend("{STM%s000NSDT}", sender_address);
            return;
        }

        char patterns[700] = {0};

        for (uint16_t i = 0; i < pattern_count; i++)
        {
            // Tymczasowy bufor na pojedynczy wzorzec (7 znaków + \0)
            char pattern_str[8];
            // Formatuj wzorzec: 4 cyfry okres + 3 cyfry wypełnienie
            snprintf(pattern_str, sizeof(pattern_str), "%04lu%03u", pattern_buffer[i].period, pattern_buffer[i].duty);

            // Dopisuj do głównego bufora
            strncat(patterns, pattern_str, 7);
        }

        USART_fsend("{STM%s%03uACKN%s}", sender_address, pattern_count * 7, patterns);
    }
    else
    {
    	USART_fsend("{STM%s000CMNF}", sender_address);
    }
}

bool ConfigurePWM(PWM_Config *config)
{
    uint32_t timer_clock = SystemCoreClock / 2; // APB1 zegar - 4 000 000
    uint32_t desired_period = timer_clock / config->frequency; // dziele zegar systemowy przez czestotliwosc z konfiguracji

    uint32_t prescaler = 1; // inicjalizacja prescalera
    while (desired_period > 65535) // dopoki wartosc jest wieksza od 16 bitow
    {
        prescaler++;
        desired_period = timer_clock / (config->frequency * prescaler);
    }

    timer_period = desired_period;

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

//    USART_fsend("\n\rprescaler %d || timer period %d\n\r", prescaler, timer_period);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler - 1;
    htim2.Init.Period = timer_period - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        return false;
    }

    // konfiguracja PWM
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (timer_period * config->duty) / 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    return true;
}

bool StartPWMGeneration()
{
    if (pattern_count > 0)
    {
        // konfiguracja DMA
        for (uint16_t i = 0; i < pattern_count; i++)
        {
            pwm_buffer[i] = (timer_period * pattern_buffer[i].duty) / 100;
        }

        // start DMA
        if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, pwm_buffer, pattern_count) != HAL_OK)
        {
            return false;
        }
    }
    else
    {
        // pojeczynczy PWM
        if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
        {
            return false;
        }
    }

    return true;
}

bool StopPWMGeneration()
{
    if (pattern_count > 0)
    {
        if (HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1) != HAL_OK)
        {
            return false;
        }
    }
    else
    {
        if (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1) != HAL_OK)
        {
            return false;
        }
    }

    return true;
}

uint16_t crc16_ccitt_false(uint16_t crc, uint8_t *data, uint16_t length)
{
    while (length--)
    {
        crc ^= ((uint16_t)(*data++) << 8);
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
