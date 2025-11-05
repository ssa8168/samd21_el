/**
 * \file
 *
 * \brief  This file contains the SAMD QTouch library sample user application.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SAMD20/D21 QTouch Example
 *
 * \section Purpose
 *
 * This example demonstrates how to use QTouch library on SAMD20/D21 devices.
 *
 * \section Requirements
 *
 * This example can be used on SAMD20/D21 xplained pro and .
 *
 * \section Description
 *
 * The program configures the necessary modules for using QTouch library. After
 * it started, users can turn on and off the LED by touching the button and slider,
 * change the color of RGB LED by touching wheel on QT1 Xplained Pro board.
 */

#include <asf.h>
#include <stdio.h>
#include <touch_config_samd.h>
#include <QDebug_samd.h>

/* Macros */

/**
 * \def GET_SENSOR_STATE(SENSOR_NUMBER)
 * \brief To get the sensor state that it is in detect or not
 * \param SENSOR_NUMBER for which the state to be detected
 * \return Returns either 0 or 1
 * If the bit value is 0, it is not in detect
 * If the bit value is 1, it is in detect
 * Alternatively, the individual sensor state can be directly accessed using
 * p_qm_measure_data->p_sensor_states[(SENSOR_NUMBER/8)] variable.
 */
 #define GET_SELFCAP_SENSOR_STATE(SENSOR_NUMBER) p_selfcap_measure_data-> \
	p_sensor_states[(SENSOR_NUMBER / \
	8)] & (1 << (SENSOR_NUMBER % 8))

/**
 * \def GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER)
 * \brief To get the rotor angle or slider position.
 * These values are valid only when the sensor state for
 * corresponding rotor or slider shows in detect.
 * \param ROTOR_SLIDER_NUMBER for which the position to be known
 * \return Returns rotor angle or sensor position
 */
#define GET_SELFCAP_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) \
	p_selfcap_measure_data->p_rotor_slider_values[ \
		ROTOR_SLIDER_NUMBER]


/**
 * Discrete LED output pin definitions
 */
//#ifdef LED_0_PIN
//#undef LED_0_PIN
//#define LED_0_PIN  PIN_PB02
//#endif

#define LED_1_PIN   PIN_PA10
#define LED_2_PIN   PIN_PA11
#define LED_3_PIN   PIN_PA20
#define LED_4_PIN   PIN_PA21
#define LED_5_PIN   PIN_PB12

#define LED_6_PIN   PIN_PB13
#define LED_7_PIN   PIN_PB14
#define LED_8_PIN   PIN_PB15
#define LED_9_PIN   PIN_PA08
#define LED_10_PIN  PIN_PA09

#define LED_11_PIN  PIN_PB11
#define LED_12_PIN  PIN_PB10
#define LED_13_PIN  PIN_PA17
#define LED_14_PIN  PIN_PA18

#define INIT_SW              PIN_PA19
#define INIT_SW_EIC_PIN      PIN_PA19A_EIC_EXTINT3    // PIN_PA15A_EIC_EXTINT15
#define INIT_SW_EIC_MUX      MUX_PA19A_EIC_EXTINT3    // MUX_PA15A_EIC_EXTINT15
#define INIT_SW_EIC_PINMUX   PINMUX_PA19A_EIC_EXTINT3 // PINMUX_PA15A_EIC_EXTINT15 
#define INIT_SW_EIC_LINE     3


#define RSEV_SW     PIN_PA16

#define LED_ON       1
#define LED_OFF      0

#define MAX_TEST_PORTS  10		//14
uint32_t LED_PORT[MAX_TEST_PORTS] = { 
    LED_1_PIN, 
    LED_2_PIN, 
    LED_3_PIN, 
    LED_4_PIN, 
    LED_5_PIN, 
    LED_6_PIN, 
    LED_7_PIN, 
    LED_8_PIN, 
    LED_9_PIN, 
    LED_10_PIN  //, 
//    LED_11_PIN, 
//    LED_12_PIN, 
//    LED_13_PIN, 
//    LED_14_PIN
};

uint8_t LED_result[MAX_TEST_PORTS];

/**
 * RTC Interrupt timing definition
 */
#define TIME_PERIOD_1MSEC 33u
/**
 * Variables
 */
uint8_t rotor_state;
uint8_t rotor_position;
volatile uint16_t touch_time_counter = 0u;
struct rtc_module rtc_instance;

uint8_t current_Button_status[MAX_TEST_PORTS];

uint8_t current_Button_01_state = 0;
uint8_t current_Button_02_state = 0;
uint8_t current_Button_03_state = 0;
uint8_t current_Button_04_state = 0;
uint8_t current_Button_05_state = 0;
uint8_t current_Button_06_state = 0;
uint8_t current_Button_07_state = 0;
uint8_t current_Button_08_state = 0;
uint8_t current_Button_09_state = 0;
uint8_t current_Button_10_state = 0;
uint8_t current_Button_11_state = 0;
uint8_t current_Button_12_state = 0;
uint8_t current_Button_13_state = 0;
uint8_t current_Button_14_state = 0;


////////////////////////////////////////////////////////////////////////////
#define USART_RECV_LENGTH      1
#define RX_BUFFER_LENGTH       128

volatile uint8_t rx_buffer[USART_RECV_LENGTH+1];

#define NO_SIG     0
#define TIMER_SIG      0x0001   // 1ms timer

volatile uint16_t  process_sig = 0x0000;

void usart_read_callback(struct usart_module *const usart_module);
void usart_write_callback(struct usart_module *const usart_module);

void configure_usart(void);
void configure_usart_callbacks(void);

struct usart_module usart_instance;

#define  APPLICATION_START    0x80
#define  OPERATION_START_CMD  0x81
#define  READY_RESPONSE       0x82

#define  PROTOCOL_VERSION     0x01
#define  HEADER_LENGTH         6
#define  MAGIC_LENGTH          4
#define  BLOCK_START_NUMBER    1

typedef enum
{
    TYPE_STRING    = 0,
    TYPE_FULL_INFO = 1,
    TYPE_SIGNAL    = 2,
    TYPE_REFERENCE = 3,
    TYPE_DELTA     = 4,
    TYPE_SETTING   = 5
} information_data_type;

uint8_t MAGIC[4] = { 0x20, 0x19, 0x12, 0x01 };
uint8_t usart_recv_header[HEADER_LENGTH];
uint8_t usart_recv_buffer[RX_BUFFER_LENGTH];
uint16_t data_receive_index = 0;
uint16_t usart_receive_data_length = 0;

uint8_t TX_buffer[128];
uint8_t TX_buffer_index = 0;


typedef enum
{
    USART_WAIT_TO_START   = 0,
    USART_RECV_START      = 1,
    USART_DATA_RECEIVING  = 3,
    USART_DATA_OPERATING  = 4,
    USART_STATUS_MAX
} USART_status_type;

USART_status_type USART_status = USART_WAIT_TO_START;

uint16_t RoofCounter = 0;
uint16_t RoofCounter2 = 0;

uint16_t QTouch_Signals[16];
uint16_t QTouch_References[16];
int16_t QTouch_Delta[16];

uint8_t nvm_page_buffer[NVMCTRL_PAGE_SIZE];

uint8_t test_LED_toggle_value = 0;
uint8_t test_LED_Blinking = 0;
uint8_t board_start = 0;
uint8_t ready_to_start = 0;

uint8_t USART_TX_READY = 0;

////////////////////////////////////////////////////////////////////////////
int16_t Delta_Average[TEST_TOUCH_NUMBER]   = {DELTA_AVERAGE};
int16_t Delta_Threshold[TEST_TOUCH_NUMBER] = {DELTA_THRESHOLD};

/**
 * Prototypes
 */
/*! \brief Configure Port pins
 *
 */
void configure_port_pins(void);

/*! \brief Initialize timer
 *
 */
void timer_init( void );

/*! \brief RTC timer overflow callback
 *
 */
void rtc_overflow_callback(void);

/*! \brief Configure the RTC timer callback
 *
 */
void configure_rtc_callbacks(void);

/*! \brief Configure the RTC timer count after which interrupts comes
 *
 */
void configure_rtc_count(void);

void configure_nvm(void);


void USART_data_send( uint8_t * buff, uint8_t length);
void Get_QTouch_Signals(void);
void Get_QTouch_Reference(void);
void Get_QTouch_Delta(void);
void QTouch_get_measureement_data(void);
void Transmit_full_information(void);
void Transmit_QTouch_Signals(void);
void Transmit_QTouch_Reference(void);
void Transmit_QTouch_Delta(void);

void Transmit_QTouch_string( uint8_t * buff, uint8_t length, uint8_t on_off );
void Transmit_QTouch_Infomation(void);
void Transmit_QTouch_Setting(void);

void turn_on_LEDs( uint8_t num_leds );
void turn_off_LEDs(void);
void check_test_condition(void);
uint8_t get_touch_result( uint8_t touch_index );
void touch_check(void);
void button_int_callback(void);
void button_init(void);
void QTouch_sensor_init(void);

void QTouch_nvm_buffer_clear(void);
void QTouch_get_config(void);
void QTouch_set_config( uint8_t * data, uint8_t length );

void USART_data_receive( uint8_t data );

void QTouch_init_buffer(void);
void QTouch_send_init_message(void);

void test_LED_toggle(void);

void configure_extint_channel(void);
void extint_detection_callback(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////
void rtc_overflow_callback(void)
{
	/* Do something on RTC overflow here */
	if(touch_time_counter == touch_time.measurement_period_ms){
		touch_time.time_to_measure_touch = 1;
		touch_time.current_time_ms = touch_time.current_time_ms + touch_time.measurement_period_ms;
		touch_time_counter = 0u;
	}
	else{
		touch_time_counter++;
	}
	process_sig = TIMER_SIG;
}

void configure_rtc_callbacks(void)
{
	/* register callback */
	rtc_count_register_callback( &rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW );
	/* Enable callback */
	rtc_count_enable_callback( &rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW );
}

void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update = true;
	/* initialize rtc */
	rtc_count_init(&rtc_instance,RTC,&config_rtc_count);

	/* enable rtc */
	rtc_count_enable(&rtc_instance);
}

void timer_init(void)
{
	/* Configure and enable RTC */
	configure_rtc_count();

	/* Configure and enable callback */
	configure_rtc_callbacks();

	/* Set Timer Period */
	rtc_count_set_period(&rtc_instance,TIME_PERIOD_1MSEC);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_LEDs( uint8_t num_leds )
{
    uint8_t i;

    // LED Toggle action ...
    if( num_leds > MAX_TEST_PORTS ){
        //num_leds = MAX_TEST_PORTS;
        for( i=0 ; i<MAX_TEST_PORTS ; i++ ){
            if( LED_result[i] ){
                port_pin_set_output_level( LED_PORT[i], LED_ON );
            }
            else{
                port_pin_set_output_level( LED_PORT[i], LED_OFF );
            }
        }
    }
    else{
        for( i=0 ; i<num_leds ; i++ ){
            port_pin_set_output_level( LED_PORT[i], LED_ON );
        }
    }
}

void turn_off_LEDs(void)
{
    uint8_t i;

    for( i=0 ; i<MAX_TEST_PORTS ; i++ ){
        port_pin_set_output_level( LED_PORT[i], LED_OFF );
    }
}

void configure_port_pins(void)
{
    uint8_t i;

	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);

	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;

	port_pin_set_config(LED_0_PIN, &config_port_pin);

    for( i=0 ; i<MAX_TEST_PORTS ; i++ ){
        port_pin_set_config( LED_PORT[i],  &config_port_pin );
    }
	turn_off_LEDs();
}

void check_test_condition(void)
{
    uint8_t i;

    if( !ready_to_start ){
        ready_to_start = 1;
        for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
            // Channel is opened ... 
            if( Delta_Average[i] > (QTouch_Delta[i]+OPEN_DELTA_VALUE) ){
                ready_to_start = 0;
                LED_result[i] = 1;
            }
            else if( Delta_Average[i] < (QTouch_Delta[i]-OPEN_DELTA_VALUE) ){
                ready_to_start = 0;
                LED_result[i] = 1;
            }
			else{
                LED_result[i] = 0;
            }
        }
        if(ready_to_start){
            test_LED_Blinking = 0;
            port_pin_set_output_level(LED_0_PIN, 0);  // LED_ON
			turn_on_LEDs( TEST_TOUCH_NUMBER );
        }
        else{
            test_LED_Blinking = 1;
        }
    }
}

uint8_t get_touch_result( uint8_t touch_index )
{
    uint8_t result = 0;
    int16_t Calculation = 0;

    Calculation = (int16_t)((int16_t)QTouch_Signals[touch_index] - (int16_t)QTouch_References[touch_index]);
    if( Calculation >= (int16_t)(Delta_Average[touch_index] + Delta_Threshold[touch_index]) ){
        Calculation = 0;		
        result =  1;
    }
    return result;
}

void touch_check(void)
{
    uint8_t i;
    uint8_t on_off = 0;
    uint8_t buttons[TEST_TOUCH_NUMBER];

    for( i=0 ; i<TEST_TOUCH_NUMBER ; i++ ){
        #if 0
        buttons[i] = GET_SELFCAP_SENSOR_STATE(i);
        #else
        buttons[i] = get_touch_result(i);
        #endif
    }
    /////////////////////////////////////////////////////////////////////////////////////////
	for( i=0 ; i<TEST_TOUCH_NUMBER ; i++ ){
        if( buttons[i] ){
            port_pin_set_output_level( LED_PORT[i], LED_OFF );
            if( current_Button_status[i] != buttons[i] ){
                if( on_off ){
                    uint8_t temp_string[21] = "Pressed Button  -  1";// 20 + 1
                    if( i>=10 ){
                        temp_string[18] = 0x30 + (i/10);
                    }
                    temp_string[19] = 0x30 + (i%10);
                    Transmit_QTouch_string( temp_string, 20, on_off );
                }
                Transmit_QTouch_Infomation();
            }
        }
        else{
            if( current_Button_status[i] != buttons[i] ){
                if( on_off ){
                    uint8_t temp_string[21] = "Released Button -  1";// 20 + 1
                    if( i>=10 ){
                        temp_string[18] = 0x30 + (i/10);
                    }
                    temp_string[19] = 0x30 + (i%10);
                    Transmit_QTouch_string( temp_string, 20, on_off );
                }
                Transmit_QTouch_Infomation();
            }
        }
        current_Button_status[i] = buttons[i];
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void QTouch_sensor_init(void)
{
    port_pin_set_output_level(LED_0_PIN, 1);  // LED_0 Off
	turn_off_LEDs();
    board_start = 1;
	ready_to_start = 0;
	test_LED_Blinking = 0;
	//QTouch_send_init_message();
	//QTouch_get_config();
}

void button_int_callback(void)
{
    QTouch_sensor_init();
    //NVIC_SystemReset();	
}

void button_init(void)
{
	#if 1
    struct extint_chan_conf config_extint_chan;
    extint_chan_get_config_defaults(&config_extint_chan);

    config_extint_chan.gpio_pin           = INIT_SW_EIC_PIN;
    config_extint_chan.gpio_pin_mux       = INIT_SW_EIC_MUX;
    config_extint_chan.gpio_pin_pull      = EXTINT_PULL_DOWN;
    config_extint_chan.detection_criteria = EXTINT_DETECT_RISING;

    extint_chan_set_config( INIT_SW_EIC_LINE, &config_extint_chan );

	extint_register_callback( button_int_callback, INIT_SW_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT );
	extint_chan_enable_callback( INIT_SW_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT );
	#endif
	
    //struct port_config config_port_pin;
    //port_get_config_defaults(&config_port_pin);

    //config_port_pin.direction = PORT_PIN_DIR_INPUT;
}

void extint_detection_callback(void)
{
}

void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = BUTTON_0_EIC_PIN;
	config_extint_chan.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING; //EXTINT_DETECT_BOTH;
	extint_chan_set_config( BUTTON_0_EIC_LINE, &config_extint_chan );

	extint_register_callback( extint_detection_callback, BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT );
	extint_chan_enable_callback( BUTTON_0_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void QTouch_nvm_buffer_clear(void)
{
    uint8_t i;
    
    for( i=0 ; i<NVMCTRL_PAGE_SIZE ; i++ ){
        nvm_page_buffer[i] = 0;
    }
}

void QTouch_get_config(void)
{
    #if 0
    enum status_code error_code;
    do{
        error_code = nvm_read_buffer( 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE, nvm_page_buffer, NVMCTRL_PAGE_SIZE );
    } while( error_code == STATUS_BUSY );
    // nvm_page_buffer[0] : Sensor Number
    // nvm_page_buffer[1] : Log Speed
    // nvm_page_buffer[2] : Threshold
    // nvm_page_buffer[3] : Hysterisys
    Transmit_QTouch_Setting( nvm_page_buffer, 4 );
    #endif

    Transmit_QTouch_Setting();
}

void QTouch_set_config( uint8_t * data, uint8_t length )
{
    uint8_t i;
    enum status_code error_code;

    QTouch_nvm_buffer_clear();

    for( i=0 ; i<length ; i++ ){
        nvm_page_buffer[i] = data[i];
    }
	/////////////////////////////////////////////////////////////////////////////////
    do {
        error_code = nvm_erase_row( 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE );
    } while( error_code == STATUS_BUSY );
	/////////////////////////////////////////////////////////////////////////////////
    do {
        error_code = nvm_write_buffer( 100 * NVMCTRL_ROW_PAGES * NVMCTRL_PAGE_SIZE, nvm_page_buffer, NVMCTRL_PAGE_SIZE );
	} while( error_code == STATUS_BUSY );
	/////////////////////////////////////////////////////////////////////////////////
}

void configure_nvm(void)
{
	struct nvm_config config_nvm;

	nvm_get_config_defaults(&config_nvm);
	config_nvm.manual_page_write = false;
	nvm_set_config(&config_nvm);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void USART_data_send( uint8_t * buff, uint8_t length)
{
    usart_write_buffer_wait( &usart_instance, buff, length );
}

void Get_QTouch_Signals(void)
{
	uint8_t i;
	uint16_t *p_signals = (uint16_t *)QDEBUG_SIGNALS_PTR;

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        QTouch_Signals[i] = *p_signals;
        p_signals++;
    }
}

void Get_QTouch_Reference(void)
{
	uint8_t i;
	uint16_t *p_references = (uint16_t *)QDEBUG_REFERENCES_PTR;

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        QTouch_References[i] = *p_references;
        p_references++;
    }
}

void Get_QTouch_Delta(void)
{
	uint8_t i;
    touch_ret_t touch_ret = TOUCH_SUCCESS;
    int16_t delta;

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        touch_ret = QDEBUG_GET_DELTA_FUNC( i, &delta );
        if( touch_ret != TOUCH_SUCCESS ){
            break;
        }
        QTouch_Delta[i] = delta;
        //if(delta < NOT_AVAIL_DELTA ){
        //    test_LED_Blinking = 1;
        //}		
    }
    if( touch_ret != TOUCH_SUCCESS ){
        uint8_t temp[] = "Error Get Delta, Button -  ";  // 27 + 1
		i++;
        if(i<10){
            temp[27] = 0x30+i;
        }
        else{
            temp[26] = 0x30 + (i/10);
            temp[27] = 0x30 + (i%10);            
        }
        Transmit_QTouch_string( temp, 27, 1 );
    }
}

void QTouch_get_measureement_data(void)
{
    //Get_QTouch_Signals();
    //Get_QTouch_Reference();
    //Get_QTouch_Delta();

	uint8_t i;

	uint16_t *p_signals = (uint16_t *)QDEBUG_SIGNALS_PTR;
	uint16_t *p_references = (uint16_t *)QDEBUG_REFERENCES_PTR;
    int16_t delta;
    touch_ret_t touch_ret = TOUCH_SUCCESS;

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        QTouch_Signals[i] = *p_signals;
        QTouch_References[i] = *p_references;
        p_signals++;
        p_references++;

        touch_ret = QDEBUG_GET_DELTA_FUNC( i, &delta );
        if( touch_ret != TOUCH_SUCCESS ){
            break;
        }
        QTouch_Delta[i] = delta;
        //if(delta < OPEN_DELTA_VALUE ){
        //    ready_to_start = 0;
        //    //test_LED_Blinking = 1;
        //}		
    }
    if( touch_ret != TOUCH_SUCCESS ){
        uint8_t temp[] = "Error Get Delta, Button -  ";  // 27 + 1
		i++;
        if(i<10){
            temp[27] = 0x30+i;
        }
        else{
            temp[26] = 0x30 + (i/10);
            temp[27] = 0x30 + (i%10);            
        }
        Transmit_QTouch_string( temp, 27, 1 );
    }
}

void Transmit_full_information(void)
{
	uint8_t i;
    uint8_t pos;

    pos = 0;
    TX_buffer[pos++] = OPERATION_START_CMD;
    TX_buffer[pos++] = 0; // data length
    TX_buffer[pos++] = TYPE_FULL_INFO; //0x24

    //for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
    //    TX_buffer[pos++] = (uint8_t)((QTouch_Signals[i] >> 8) & 0xFFu);
    //    TX_buffer[pos++] = (uint8_t)(QTouch_Signals[i] & 0xFFu);
    //}
    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        TX_buffer[pos++] = (uint8_t)((QTouch_References[i] >> 8) & 0xFFu);
        TX_buffer[pos++] = (uint8_t)(QTouch_References[i] & 0xFFu);
    }
    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        TX_buffer[pos++] = (uint8_t)((QTouch_Delta[i] >> 8) & 0xFFu);
        TX_buffer[pos++] = (uint8_t)(QTouch_Delta[i] & 0xFFu);
    }
    TX_buffer[1] = pos-2; // data length
    USART_data_send( TX_buffer, pos );
}

void Transmit_QTouch_Signals(void)
{
	uint8_t i;
    uint8_t pos;

    pos = 0;
    TX_buffer[pos++] = OPERATION_START_CMD;
    TX_buffer[pos++] = 0; // data length
    TX_buffer[pos++] = TYPE_SIGNAL; //0x24

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        TX_buffer[pos++] = (uint8_t)((QTouch_Signals[i] >> 8) & 0xFFu);
        TX_buffer[pos++] = (uint8_t)(QTouch_Signals[i] & 0xFFu);
    }
    TX_buffer[1] = pos-2; // data length
    USART_data_send( TX_buffer, pos );
}

void Transmit_QTouch_Reference(void)
{
	uint8_t i;
    uint8_t pos;

    pos = 0;
    TX_buffer[pos++] = OPERATION_START_CMD;
    TX_buffer[pos++] = 0; // data length
    TX_buffer[pos++] = TYPE_REFERENCE; //0x24

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        TX_buffer[pos++] = (uint8_t)((QTouch_References[i] >> 8) & 0xFFu);
        TX_buffer[pos++] = (uint8_t)(QTouch_References[i] & 0xFFu);
    }
    TX_buffer[1] = pos-2; // data length
    USART_data_send( TX_buffer, pos );
}
void Transmit_QTouch_Delta(void)
{
	uint8_t i;
    uint8_t pos;

    pos = 0;
    TX_buffer[pos++] = OPERATION_START_CMD;
    TX_buffer[pos++] = 0; // data length
    TX_buffer[pos++] = TYPE_DELTA; //0x24

    for( i=0 ; i<QDEBUG_NUM_CHANNELS ; i++ ){
        TX_buffer[pos++] = (uint8_t)((QTouch_Delta[i] >> 8) & 0xFFu);
        TX_buffer[pos++] = (uint8_t)(QTouch_Delta[i] & 0xFFu);
    }
    TX_buffer[1] = pos-2; // data length
    USART_data_send( TX_buffer, pos );
}

void Transmit_QTouch_string( uint8_t * buff, uint8_t length, uint8_t on_off )
{
    uint8_t i;
    uint8_t pos;
    // Operation Start + Total Length + Operation Type + Data
    
    if( on_off ){
        pos = 0;
        TX_buffer[pos++] = OPERATION_START_CMD;
        TX_buffer[pos++] = 0; // data length
        TX_buffer[pos++] = TYPE_STRING;

        for( i=0 ; i<length ; i++ ){
            TX_buffer[pos++] = buff[i];
	    }
        TX_buffer[1] = pos-2; // data length
        USART_data_send( TX_buffer, pos );
    }
}

void Transmit_QTouch_Infomation(void)
{
    if( USART_TX_READY ){
        //Transmit_QTouch_Signals();
        //Transmit_QTouch_Reference();
        //Transmit_QTouch_Delta();
        Transmit_full_information();
        USART_TX_READY = 0;
    }
}

void Transmit_QTouch_Setting(void)
{
    uint8_t i;
    uint8_t pos;

    pos = 0;
    TX_buffer[pos++] = OPERATION_START_CMD;
    TX_buffer[pos++] = 0;  //(length*2) + 1; // data length
    TX_buffer[pos++] = TYPE_SETTING;

    for( i=0 ; i<TEST_TOUCH_NUMBER ; i++ ){
        TX_buffer[pos++] = (uint8_t)(Delta_Average[i]>>8);
        TX_buffer[pos++] = (uint8_t)(Delta_Average[i]);		
    }
    for( i=0 ; i<TEST_TOUCH_NUMBER ; i++ ){
        TX_buffer[pos++] = (uint8_t)(Delta_Threshold[i]>>8);
        TX_buffer[pos++] = (uint8_t)(Delta_Threshold[i]);		
    }
    TX_buffer[1] = pos-2; // data length
    USART_data_send( TX_buffer, pos );
}

void usart_read_callback(struct usart_module *const usart_module)
{
    //usart_write_buffer_job( &usart_instance, (uint8_t *)rx_buffer, USART_RECV_LENGTH );
    USART_data_receive( rx_buffer[0] );
}

void usart_write_callback(struct usart_module *const usart_module)
{
    ;//port_pin_toggle_output_level(LED_0_PIN);
}

void configure_usart(void)
{
    struct usart_config config_usart;
    usart_get_config_defaults(&config_usart);

#if(SAMR30E)
    config_usart.baudrate    = 9600;
    config_usart.mux_setting = CDC_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = CDC_SERCOM_PINMUX_PAD0;
    config_usart.pinmux_pad1 = CDC_SERCOM_PINMUX_PAD1;
    config_usart.pinmux_pad2 = CDC_SERCOM_PINMUX_PAD2;
    config_usart.pinmux_pad3 = CDC_SERCOM_PINMUX_PAD3;

    while( usart_init(&usart_instance, CDC_MODULE, &config_usart) != STATUS_OK ){
        ;
    }
#else
    config_usart.baudrate    = 115200;
    config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
    config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
    config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
    config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
    config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
    while( usart_init( &usart_instance, EDBG_CDC_MODULE, &config_usart) != STATUS_OK ){
        ;
    }
#endif
	usart_enable( &usart_instance );
}

void configure_usart_callbacks(void)
{
	usart_register_callback( &usart_instance, usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED );
	usart_register_callback( &usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED );

    usart_enable_callback( &usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED );
    usart_enable_callback( &usart_instance, USART_CALLBACK_BUFFER_RECEIVED );
}

void USART_data_receive( uint8_t data )
{
	if( data == READY_RESPONSE ){
		USART_TX_READY = 1;
    }
    else if( data == APPLICATION_START ){
		NVIC_SystemReset();
		//QTouch_send_init_message();
		//QTouch_get_config();
    }	
	
#if 0
    switch( USART_status ){
        case USART_WAIT_TO_START:
            if( data == OPERATION_START_CMD ){
                data_receive_index = 0;
                USART_status = USART_RECV_START;
            }
            break;

        case USART_RECV_START:
            usart_receive_data_length = data;
            if( usart_receive_data_length == 0 ){
                // Ignore ...
                data_receive_index = 0;
                USART_status = USART_WAIT_TO_START;
            }
            else if( usart_receive_data_length > 64 ){
                // Error Case ...
                data_receive_index = 0;
                USART_status = USART_WAIT_TO_START;
			}
            else{
                USART_status = USART_DATA_RECEIVING;
                data_receive_index = 0;
            }
            break;

        case USART_DATA_RECEIVING:
            usart_recv_buffer[data_receive_index++] = data;
            if( data_receive_index >= usart_receive_data_length ){
                if( usart_recv_buffer[0] == TYPE_SETTING ){
                    QTouch_set_config( (uint8_t *)&usart_recv_buffer[1], (usart_receive_data_length-1) );
				}				
                USART_status = USART_WAIT_TO_START;
                data_receive_index = 0;
                usart_receive_data_length = 0;
                NVIC_SystemReset();
            }
            break;

        default:
            // garbage...
            break;
    }
#endif	
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void QTouch_init_buffer(void)
{
    uint8_t i;
    for( i=0 ; i<16 ; i++ ){
        QTouch_Signals[i] = 0;
        QTouch_References[i] = 0;
        QTouch_Delta[i] = 0;
    }

    for( i=0 ; i<MAX_TEST_PORTS ; i++ ){
        LED_result[i] = 0;
    }
    QTouch_nvm_buffer_clear();
	test_LED_Blinking = 0;
}

void QTouch_send_init_message(void)
{
    //uint8_t i;
	
    //for( i=0 ; i<128 ; i++ ){
    //    TX_buffer[i] = 0;		
    //}
	//usart_write_buffer_wait( &usart_instance, TX_buffer, 128 );
	
    Transmit_QTouch_string( (uint8_t *)"\r\n\r\n\r\n\r\n", 8, 1 );
    Transmit_QTouch_string( (uint8_t *)"*************** SelfCap Inspector init ... !!!!! ***************\r\n", 66, 1 );
}

void test_LED_toggle(void)
{
	if(test_LED_toggle_value){
		test_LED_toggle_value = 0;
		turn_on_LEDs( 0xFF );
	}
	else{
		test_LED_toggle_value = 1;
		turn_off_LEDs();		
	}
}

int main(void)
{
    system_init();
    configure_port_pins();

    delay_init();
    timer_init();

    button_init();
    configure_extint_channel();
    
    configure_usart();
    configure_usart_callbacks();
    system_interrupt_enable_global();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
	QTouch_init_buffer();
	//configure_nvm();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
	touch_sensors_init();
	port_pin_set_output_level(LED_0_PIN, 1);  // LED_0 Off
	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);

    QTouch_send_init_message();
    QTouch_get_config();
    Transmit_QTouch_Infomation();

	while (1) {
		/**
		 * Goto STANDBY sleep mode, unless woken by timer or PTC interrupt.
		 */
		//system_sleep();

        touch_sensors_measure(); // 20 ms duration ...
        if ((p_selfcap_measure_data->measurement_done_touch == 1u)) {
            p_selfcap_measure_data->measurement_done_touch = 0u;
            QTouch_get_measureement_data();
            if( board_start ){
                check_test_condition();
                if( ready_to_start ){
                    touch_check();
                }
            }
                
            RoofCounter++;
            if( RoofCounter >= 50/LOG_SPEED ){
                RoofCounter = 0;
                Transmit_QTouch_Infomation();					
            }
        }

        switch( process_sig ){
            case TIMER_SIG:
                process_sig = NO_SIG;
                RoofCounter2++;
                if( RoofCounter2>250 ){
                    RoofCounter2 = 0;
                    if( board_start && test_LED_Blinking ){
                        test_LED_toggle();
                    }
                }
                break;

            default:
                break;
        }
        usart_read_buffer_job( &usart_instance, (uint8_t *)rx_buffer, USART_RECV_LENGTH );
	}
}
