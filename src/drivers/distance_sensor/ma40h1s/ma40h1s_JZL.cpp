/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ma40h1s.cpp
 * @author KeChaofan
 *
 * Driver for the ma40h1s sonar range finders connected via GPIO.
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_defines.h>
#include <board_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <lib/mathlib/mathlib.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

//#include <systemlib/perf_counter.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/stm32/drv_io_timer.h>
#include <drivers/drv_pwm_output.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position_setpoint.h> 
#include <uORB/topics/adc_ultrasound.h>
// #include <uORB/topics/rtps_send_distance_sensor.h> //publish this topic in sensors.cpp

#include <modules/px4iofirmware/protocol.h>

#include <arch/board/board.h>

#include <stm32_adc.h>
#include <stm32_gpio.h>
#include <stm32_dma.h>
#include <float.h>
#include <getopt.h>


#include "sonar_decoder_c.c"
/*
 * Register accessors.
 */
#define REG(_reg)   (*(volatile uint32_t *)(STM32_ADC2_BASE + _reg))

#define rSR         REG(STM32_ADC_SR_OFFSET)
#define rCR1        REG(STM32_ADC_CR1_OFFSET)
#define rCR2        REG(STM32_ADC_CR2_OFFSET)
#define rCSR        REG(STM32_ADC_CSR_OFFSET)
#define rCCR        REG(STM32_ADC_CCR_OFFSET) 
#define rSMPR1      REG(STM32_ADC_SMPR1_OFFSET)
#define rSMPR2      REG(STM32_ADC_SMPR2_OFFSET)
#define rJOFR1      REG(STM32_ADC_JOFR1_OFFSET)
#define rJOFR2      REG(STM32_ADC_JOFR2_OFFSET)
#define rJOFR3      REG(STM32_ADC_JOFR3_OFFSET)
#define rJOFR4      REG(STM32_ADC_JOFR4_OFFSET)
#define rHTR        REG(STM32_ADC_HTR_OFFSET)
#define rLTR        REG(STM32_ADC_LTR_OFFSET)
#define rSQR1       REG(STM32_ADC_SQR1_OFFSET)
#define rSQR2       REG(STM32_ADC_SQR2_OFFSET)
#define rSQR3       REG(STM32_ADC_SQR3_OFFSET)
#define rJSQR       REG(STM32_ADC_JSQR_OFFSET)
#define rJDR1       REG(STM32_ADC_JDR1_OFFSET)
#define rJDR2       REG(STM32_ADC_JDR2_OFFSET)
#define rJDR3       REG(STM32_ADC_JDR3_OFFSET)
#define rJDR4       REG(STM32_ADC_JDR4_OFFSET)
#define rDR         REG(STM32_ADC_DR_OFFSET)


#define MA40H1S_CONVERSION_INTERVAL 40000 //us  43500 + 23000 + 23000= 89500   bymark  43500 -> 143500(+100000us)  25000

// #define MID_LENGTH  7

#define ADC_BUFFER_SIZE 10000  // 10000->20000 -> 30000 bymark

// #define ADC_LOG 1  //recording adc value log 
#if NUM_OF_ULTRASOUND > 2
#define MULTI_ADC 1 // using ADC2 and ADC3
#endif

#ifdef MULTI_ADC

#define REG_3(_reg)   (*(volatile uint32_t *)(STM32_ADC3_BASE + _reg))

#define r_SR         REG_3(STM32_ADC_SR_OFFSET)
#define r_CR1        REG_3(STM32_ADC_CR1_OFFSET)
#define r_CR2        REG_3(STM32_ADC_CR2_OFFSET)
#define r_CSR        REG_3(STM32_ADC_CSR_OFFSET)
#define r_CCR        REG_3(STM32_ADC_CCR_OFFSET) 
#define r_SMPR1      REG_3(STM32_ADC_SMPR1_OFFSET)
#define r_SMPR2      REG_3(STM32_ADC_SMPR2_OFFSET)
#define r_JOFR1      REG_3(STM32_ADC_JOFR1_OFFSET)
#define r_JOFR2      REG_3(STM32_ADC_JOFR2_OFFSET)
#define r_JOFR3      REG_3(STM32_ADC_JOFR3_OFFSET)
#define r_JOFR4      REG_3(STM32_ADC_JOFR4_OFFSET)
#define r_HTR        REG_3(STM32_ADC_HTR_OFFSET)
#define r_LTR        REG_3(STM32_ADC_LTR_OFFSET)
#define r_SQR1       REG_3(STM32_ADC_SQR1_OFFSET)
#define r_SQR2       REG_3(STM32_ADC_SQR2_OFFSET)
#define r_SQR3       REG_3(STM32_ADC_SQR3_OFFSET)
#define r_JSQR       REG_3(STM32_ADC_JSQR_OFFSET)
#define r_JDR1       REG_3(STM32_ADC_JDR1_OFFSET)
#define r_JDR2       REG_3(STM32_ADC_JDR2_OFFSET)
#define r_JDR3       REG_3(STM32_ADC_JDR3_OFFSET)
#define r_JDR4       REG_3(STM32_ADC_JDR4_OFFSET)
#define r_DR         REG_3(STM32_ADC_DR_OFFSET)

#endif


enum MA40H1S_ID
{
    MA40H1S_ID_DOWN     = 0,
    MA40H1S_ID_RIGHT    = 1,
    MA40H1S_ID_LEFT     = 2     
};

#ifdef ADC_LOG
struct interrupt_arg
{
    enum MA40H1S_ID *pdev_id;
    orb_advert_t *adc_pub;
};
#endif

#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MA40H1S : public device::CDev
{
public:
    MA40H1S();

    virtual ~MA40H1S();

    virtual int init();

    virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
    virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

    /**
    * Diagnostics - print some basic information about the driver.
    */
    void    print_info();
    void    trig();
    void    interrupt(hrt_abstime time);

    void    test_high();
    void    test_low();

protected:
    virtual int probe();

private:
    float _min_distance;
    float _max_distance;
    float distance_orginal;

    int _class_instance;
    int _orb_class_instance[NUM_OF_ULTRASOUND];

    int _measure_ticks;
    int _cycling_rate;

    static bool _echo_valid;
    static uint8_t trig_state;
	//static bool voltage_state;
    static uint8_t  _echo_count;

    bool new_value[NUM_OF_ULTRASOUND];
    uint8_t err_count[NUM_OF_ULTRASOUND];
    float pre_distance_m[NUM_OF_ULTRASOUND];
    uint64_t _data_invalid[NUM_OF_ULTRASOUND];
    float sonar_sample[NUM_OF_ULTRASOUND][10];
    
    orb_advert_t    _distance_sensor_pub[NUM_OF_ULTRASOUND];
    // orb_advert_t    _rtps_send_distance_pub;

#ifdef ADC_LOG
    int             _orb_adc_instance[NUM_OF_ULTRASOUND];
    orb_advert_t    _adc_value_pub[NUM_OF_ULTRASOUND];
#endif

    work_s          _work;
	
	struct actuator_armed_s	           _armed = {};	
	struct vehicle_land_detected_s	     _vehicle_land_detected = {};
	
    ringbuffer::RingBuffer  *_reports;
    static hrt_abstime _start_time; 
    hrt_abstime _end_time;
    uint16_t _end_index[NUM_OF_ULTRASOUND];

#ifdef MULTI_ADC
    // uint16_t    _end_index_1;
    static bool _adc3_end;    
#endif
    // hrt_abstime _last_inte;

    // struct hrt_call     _call;

    // DMA_HANDLE      _tx1_dma;
    DMA_HANDLE      _adc_dma;

    // static struct stm32_tim_dev_s * _tim8; 
    struct stm32_tim_dev_s * _tim13;    // timer5 --> timer13

    // static bool _time_up;

    // uint32_t     _dma_buffer[2];
    static uint16_t     adc_buffer[ADC_BUFFER_SIZE];
    static bool _adc2_end;

#ifdef MULTI_ADC
    static uint16_t     adc_buffer_1[ADC_BUFFER_SIZE];
    enum MA40H1S_ID     _ultrasound_id[2];
    DMA_HANDLE          _adc_dma_1;
#endif

#ifndef MULTI_ADC
    enum MA40H1S_ID     _ultrasound_id[1];
#endif

    uint16_t average_sample;
    // uint16_t thr_value;

    bool single_test_mode;

    // struct GPIOConfig {
    //     uint32_t        dr_a_port;
    //     uint32_t        dr_b_port;
    // };
    
    struct dev_config {
        enum MA40H1S_ID id;
        uint8_t pwm1_ch;
        uint8_t pwm2_ch;
        uint8_t timer_index;
        uint8_t adc_ch;
    };
    static const dev_config _ultrasound_config[NUM_OF_ULTRASOUND];
    static uint8_t _ultrasound_n;

    // static const GPIOConfig _gpio_tab;
    /**
    * Initialise the automatic measurement state machine and start it.
    */
    void start();
    
    /**
    * Stop the automatic measurement state machine.
    */
    void stop();

    /**
    * Perform a poll cycle; collect from the previous measurement
    * and start a new one.
    */
    void cycle();
    int  measure();
    int  collect();

    /**
    * Set the min and max distance thresholds if you want the end points of the sensors
    * range to be brought in at all, otherwise it will use the defaults MB12XX_MIN_DISTANCE
    * and MB12XX_MAX_DISTANCE
    */
    void  set_minimum_distance(float min);
    void  set_maximum_distance(float max);
    float get_minimum_distance();
    float get_maximum_distance();

    /**
    * Static trampoline from the workq context; because we don't have a
    * generic workq wrapper yet.
    *
    * @param arg        Instance pointer for the driver that is polling.
    */
    static void cycle_trampoline(void *arg);

    static void tick_trampoline(void *arg);

    static int timer13_interrupt(int irq, void *context, void *arg);

    static void  _dma_callback(DMA_HANDLE handle, uint8_t status, void *arg);

    void        _do_adc_dma_callback(unsigned status);

#ifdef MULTI_ADC
    static void  _dma_callback_1(DMA_HANDLE handle, uint8_t status, void *arg);

    void        _do_adc_dma_callback_1(unsigned status);
#endif    
};


// bymark 超声（两路互补PWM + 一路ADC采样）的配置说明：
// bymark 6,5表示PWM6和PWM5； 1表示timer_index为1，对应TIM4，用于控制PWM生成，在timer_config.c对应其配置; 14表示ADC通道
const MA40H1S::dev_config MA40H1S::_ultrasound_config[NUM_OF_ULTRASOUND] = {
    {MA40H1S_ID_DOWN,  6, 5, 1, 14}  // 6, 5, 1, 14->3(testing: good); ->2(testing: good)
#if NUM_OF_ULTRASOUND > 1
    ,{MA40H1S_ID_RIGHT, 8, 7, 2, 2} // 8, 7, 2, 4->2(ADC2CH4 is not good)
#endif
#if NUM_OF_ULTRASOUND > 2
    ,{MA40H1S_ID_LEFT, 9, 10, 3, 3} // 4, 3, 0, 3
#endif
};


hrt_abstime MA40H1S::_start_time = 0;
bool MA40H1S::_echo_valid = false;
uint8_t MA40H1S::trig_state = 5;
//bool MA40H1S::voltage_state = false;
uint8_t MA40H1S::_echo_count = 0;
uint8_t MA40H1S::_ultrasound_n = 0; 

 /*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ma40h1s_main(int argc, char *argv[]);
// static int sonar_isr(int irq, void *context);

uint16_t MA40H1S::adc_buffer[ADC_BUFFER_SIZE] = {};
bool MA40H1S::_adc2_end = false;
#ifdef MULTI_ADC
uint16_t MA40H1S::adc_buffer_1[ADC_BUFFER_SIZE] = {};
bool MA40H1S::_adc3_end = false;
#endif
// struct stm32_tim_dev_s * _tim8 = nullptr;

MA40H1S::MA40H1S():
    CDev("MA40H1S", MA40H1S_DEVICE_PATH),
    _min_distance(0.28f),
    _max_distance(3.0f),
    _class_instance(-1),
    // _orb_class_instance(-1),
    _measure_ticks(0),
    _cycling_rate(0),
    // _rtps_send_distance_pub(nullptr),
    // new_value(false),
    // _distance_sensor_pub(nullptr),
    // _adc_value_pub(nullptr),
    _reports(nullptr),
    _end_time(0),
    // _end_index(0),
    // _last_inte(0),
    // _tx1_dma(nullptr),
    _adc_dma(nullptr)
{
    _armed.armed = false;
    _vehicle_land_detected.landed = true;
    single_test_mode = false;
    memset(&_work, 0, sizeof(_work));
    // memset(&_call, 0, sizeof(_call));
    for (uint8_t i=0; i<NUM_OF_ULTRASOUND; i++) {
    	_orb_class_instance[i] = -1;
    	_distance_sensor_pub[i] = nullptr;
        new_value[i] = false;
        err_count[i] = 0;
        pre_distance_m[i] = 0.283f;
        _data_invalid[i] = 0;
        _end_index[i] = 0;
#ifdef ADC_LOG
        _orb_adc_instance[i] = -1;
        _adc_value_pub[i] = nullptr;
#endif
    }

#ifdef MULTI_ADC
    _adc_dma_1 = nullptr;
#endif
}

MA40H1S::~MA40H1S()
{
    /* close timer13*/
    if (_tim13 != NULL || _tim13 != nullptr) {
        if (OK != stm32_tim_deinit(_tim13)) {
            printf("[@ma40h1s_JZL.cpp][~MA40H1S()] Fail to close tim13\n");
        }
    }
    /* unadvertise ''distance_sensor'' topic */
    for (uint8_t i=0; i<NUM_OF_ULTRASOUND; i++) {
        if (_distance_sensor_pub[i] != nullptr) {
            if (0 != orb_unadvertise(_distance_sensor_pub[i])) {
                printf("[@ma40h1s_JZL.cpp][~MA40H1S()] Fail to orb_unadvertise distance_sensor_instance %d\n", i);
            }
        }
    }
    /* stop and free DMA for ADC */
    if (_adc_dma != NULL) {
        stm32_dmastop(_adc_dma);
        stm32_dmafree(_adc_dma);
    }

#ifdef MULTI_ADC
    if (_adc_dma_1 != NULL) {
        stm32_dmastop(_adc_dma_1);
        stm32_dmafree(_adc_dma_1);
    }
#endif

    stop();

    /* free any existing reports */
    if (_reports != nullptr) {
        delete _reports;
    }

    // if (_tim5 != nullptr) {
    // 	stm32_tim_deinit(_tim5);
    // 	_tim5 = nullptr;
    // }

    if (_class_instance != -1) {
        unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
    }

    printf("[@ma40h1s_JZL.cpp][~MA40H1S()] Now, stop MA40H1S\n");   
}

int MA40H1S::init()
{
    int ret = ERROR;

    /* do super class device init first*/
    if(CDev::init() != OK){
        return ret;
    }

    /* allocate basic report buffers */
    _reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

    if (_reports == nullptr) {
        return ret;
    }

    _class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

    if (_class_instance == CLASS_DEVICE_PRIMARY) {
        /* get a publish handle on the range finder topic */
        struct distance_sensor_s ds_report = {};
        // struct rtps_send_distance_sensor_s rtps_report = {};
#ifdef ADC_LOG
        struct adc_ultrasound_s ar_report = {}; //bymark recording ADC log
#endif

        for (uint8_t i=0; i<NUM_OF_ULTRASOUND; i++) {
            for (uint j=0; j<10; j++) {
                sonar_sample[i][j] = 0.283f;
            }
	        _distance_sensor_pub[i] = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
	                                 &_orb_class_instance[i], ORB_PRIO_LOW);

            if (_distance_sensor_pub[i] == nullptr) {
            	DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
            	break;
        	}
        	printf("[@ma40h1s.cpp][init()]_orb_class_instance = %d \n", _orb_class_instance[i]);

#ifdef ADC_LOG
            _adc_value_pub[i] = orb_advertise_multi(ORB_ID(adc_ultrasound), &ar_report,
                                     &_orb_adc_instance[i], ORB_PRIO_LOW);

            if (_adc_value_pub[i] == nullptr) {
                DEVICE_LOG("failed to create adc_ultrasound object. Did you start uOrb?");
                break;
            }
            printf("[@ma40h1s.cpp][init()]_orb_adc_instance = %d \n", _orb_adc_instance[i]);  //bymark
#endif
        }

        // _rtps_send_distance_pub = orb_advertise(ORB_ID(rtps_send_distance_sensor), &rtps_report);
        // if (_rtps_send_distance_pub == nullptr) {
        //     DEVICE_LOG("failed to create rtps_send_distance_sensor object. Did you start uOrb?");
        // }

        // _distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
        //                          &_orb_class_instance, ORB_PRIO_LOW);

        // if (_distance_sensor_pub == nullptr) {
        //     DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
        // }
    }
    // stm32_configgpio(_dr_a_port);
    // stm32_configgpio(_dr_b_port);
    //stm32_configgpio(_gpio_tab.sw_a_port); 
    // stm32_configgpio(_gpio_tab.sw_b_port);
    //stm32_configgpio(GPIO_BAK_DR_B);
    // printf("dma init start\n");
    // _tx1_dma = stm32_dmachannel(PX4FMU_SONAR_TX4_DMAMAP); 

    _ultrasound_id[0] = _ultrasound_config[0].id;

#ifndef MULTI_ADC
    uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1)); //bymark 0b00110000 <==> pwm6 and pwm5  
#endif 
#ifdef MULTI_ADC
    _ultrasound_id[1] = _ultrasound_config[1].id;  
    uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                             1<<(_ultrasound_config[1].pwm1_ch-1) | 1<<(_ultrasound_config[1].pwm2_ch-1));
#endif

	for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {
			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);
			channel_mask &= ~(1 << channel);
		}
	}
   
	io_timer_set_ccr(_ultrasound_config[0].pwm1_ch-1, 12); // bymark 需要PWM频率是40KHz,驱动超声，timer的时钟是1MHz，所以ARR为24，取CCR为12，这样PWM的占空比为50%
	io_timer_set_ccr(_ultrasound_config[0].pwm2_ch-1, 12);
    io_timer_set_rate(_ultrasound_config[0].timer_index, 40000); // bymark 设置PWM频率为40KHz;timer_index 1: TIM4   timer_index 2: TIM12
    // io_timer_set_ccr(_ultrasound_config[0].pwm2_ch, 12);
    // io_timer_trigger();

#ifdef MULTI_ADC
    io_timer_set_ccr(_ultrasound_config[1].pwm1_ch-1, 12);
    io_timer_set_ccr(_ultrasound_config[1].pwm2_ch-1, 12);
    io_timer_set_rate(_ultrasound_config[1].timer_index, 40000);
#endif    

    /*
     stm32_dmasetup(
        _tx1_dma, 
        _GPIOx_BSRR_addr, // Pb 0x40020818 0x40021000
        reinterpret_cast<uint32_t>(&_dma_buffer),
        2,
        DMA_SCR_DIR_M2P |\
        DMA_SCR_MINC |\
        DMA_SCR_PSIZE_32BITS |\
        DMA_SCR_MSIZE_32BITS |\
        DMA_SCR_PBURST_SINGLE |\
        DMA_SCR_MBURST_SINGLE |\
	    DMA_SCR_CIRC);*/
    // stm32_dmastart(_tx1_dma, nullptr, nullptr, false);
    // printf("tx1 dma\n");
	
 
    // _tim8 = stm32_tim_init(8);
    // if(_tim8 == NULL){
    //     // printf("timer8 init failed\n");
    //     return ret;
    // }
    // // printf("timer8 init success\n");
    // STM32_TIM_SETPERIOD(_tim8, 24);
    // STM32_TIM_SETCLOCK(_tim8,2000000);
    // STM32_TIM_SETMODE(_tim8,STM32_TIM_MODE_UP);
    // //STM32_TIM_SETCOMPARE(_tim8,3,11);
    // //STM32_TIM_SETCOMPARE(_tim8,2,5);
    // usleep(200000);

    _tim13 = stm32_tim_init(13); // bymark 初始化TIM13，该定时器用于控制超声发波，接收等时序
    if(_tim13 == NULL){
        PX4_WARN("Timer13 init failed");
        return ret;
    }
    printf("[@ma40h1s.cpp][init()] timere13 init successfully \n"); //bymark

#ifdef ADC_LOG
    struct interrupt_arg tim13_arg = {_ultrasound_id, _adc_value_pub};
    STM32_TIM_SETISR(_tim13, MA40H1S::timer13_interrupt, &tim13_arg, 0);
#endif

#ifndef ADC_LOG
    // enum MA40H1S_ID * pdev_id = _ultrasound_id;
    // STM32_TIM_SETISR(_tim13, MA40H1S::timer13_interrupt, _ultrasound_id, 0);
    STM32_TIM_SETISR(_tim13, MA40H1S::timer13_interrupt, NULL, 0); // bymark 设置TIM13的中断服务函数
#endif
    putreg16(0x0001,STM32_TIM13_DIER);// bymark 设置TIM13中断使能寄存器 putreg16(0x0101,0x40000c0c);  //STM32_TIM5_BASE:0x40000c00  STM32_GTIM_DIER_OFFSET:0x000c
    STM32_TIM_SETPERIOD(_tim13, 4);     // bymark 设置TIM13的ARR
    STM32_TIM_SETCLOCK(_tim13,1000000); // bymark 设置TIM13的时钟为1MHz
    STM32_TIM_SETMODE(_tim13,STM32_TIM_MODE_UP); // bymark 设置TIM13的计数模式
    // STM32_TIM_SETCOMPARE();
    printf("[@ma40h1s.cpp][init()] timere13 config successfully \n");   //bymark
    _cycling_rate = MA40H1S_CONVERSION_INTERVAL;


    _adc_dma = stm32_dmachannel(DMAMAP_ADC2_1); // bymark 设置ADC2的DMA流和通道
    if(_adc_dma == nullptr || _adc_dma == NULL){
        printf("[@ma40h1s.cpp][init()] adc dma init failed\n");
        return ret;
    }
    printf("[@ma40h1s.cpp][init()] adc dma init successfully\n");   //bymark
    // PX4_INFO("ADC_DMA channel");
    stm32_dmasetup(
        _adc_dma, 
        STM32_ADC2_DR, // adc2 DR  
        reinterpret_cast<uint32_t>(&adc_buffer),
        ADC_BUFFER_SIZE,
        DMA_SCR_DIR_P2M |\
        DMA_SCR_MINC |\
        DMA_SCR_PSIZE_16BITS |\
        DMA_SCR_MSIZE_16BITS |\
        DMA_SCR_PBURST_SINGLE |\
        DMA_SCR_MBURST_SINGLE);//DMA_SCR_CIRC

    printf("[@ma40h1s.cpp][init()] adc dma setup\n");   //bymark
    stm32_dmastart(_adc_dma, _dma_callback, this, false);
    printf("[@ma40h1s.cpp][init()] adc dma start\n");   //bymark
#ifdef MULTI_ADC
    _adc_dma_1 = stm32_dmachannel(DMAMAP_ADC3_1);
    if(_adc_dma_1 == nullptr || _adc_dma_1 == NULL){
        printf("adc_dma_1 init failed\n");
        return ret;
    }
    stm32_dmasetup(
        _adc_dma_1, 
        STM32_ADC3_DR, // adc3 DR  
        reinterpret_cast<uint32_t>(&adc_buffer_1),
        ADC_BUFFER_SIZE,
        DMA_SCR_DIR_P2M |\
        DMA_SCR_MINC |\
        DMA_SCR_PSIZE_16BITS |\
        DMA_SCR_MSIZE_16BITS |\
        DMA_SCR_PBURST_SINGLE |\
        DMA_SCR_MBURST_SINGLE);//DMA_SCR_CIRC
    stm32_dmastart(_adc_dma_1, _dma_callback_1, this, false);

    // r_SMPR1 = 0b00000000000000000101000000000000; //  set sample time of adc3_ch14 to 010 (28T) 100(84T) 101(112T)
    r_SMPR1 = 0; // bymark ADC的默认时钟是54MHz
    r_SMPR2 = 0b00000000000000000000101101000000; // set sample time of adc3_ch2 to 101(112T)   adc3_ch3(testing)

    r_CR1 = ADC_CR1_RES_12BIT; //Resolution
    r_CR2 = 0;

    r_SQR1 = 0;
    r_SQR2 = 0;
    r_SQR3 = _ultrasound_config[1].adc_ch;

    if(r_SR & ADC_SR_EOC) {
       r_SR &= ~ADC_SR_EOC;
    }

    r_CR2 |= ADC_CR2_DDS;
    r_CR2 |= ADC_CR2_DMA;
    r_CR2 |= ADC_CR2_CONT;

    /* power-cycle the ADC and turn it on */
    r_CR2 &= ~ADC_CR2_ADON;
    usleep(10);
    r_CR2 |= ADC_CR2_ADON;
    usleep(10);
    r_CR2 |= ADC_CR2_ADON;
    usleep(10);

    if(r_SR & ADC_SR_EOC) {
       r_SR &= ~ADC_SR_EOC;
    }
    r_CR2 |= ADC_CR2_DDS;
    r_CR2 |= ADC_CR2_DMA;

#endif     
    //rSMPR1 = 0b00 000 000 000 000 010 000 000 000 000 000; //  10--18  Channel 15
    //rSMPR2 = 0b00 000 000 000 000 010 000 000 000 000 000; //  0--9  Channel 5
    rSMPR1 = 0b00000000000000000101000000000000; //  set sample time of adc2_ch14 to 010 (28T) 100(84T) 101(112T)
    // rSMPR2 = 0b00000000000000000101000000000000; // set sample time of adc2_ch4 to 010 (28T)
#ifndef MULTI_ADC
    rSMPR2 = 0b00000000000000000000101101000000; // set sample time of adc2_ch2 to 101(112T)   adc2_ch3(testing)
#endif
#ifdef MULTI_ADC
    rSMPR2 = 0;
#endif

    rCR1 = ADC_CR1_RES_12BIT; //Resolution
    rCR2 = 0;

    rSQR1 = 0;
    rSQR2 = 0;
    rSQR3 = _ultrasound_config[0].adc_ch;

    if(rSR & ADC_SR_EOC) {
       rSR &= ~ADC_SR_EOC;
    }

    rCR2 |= ADC_CR2_DDS;
    rCR2 |= ADC_CR2_DMA;
    rCR2 |= ADC_CR2_CONT;

    /* power-cycle the ADC and turn it on */
    rCR2 &= ~ADC_CR2_ADON;
    usleep(10);
    rCR2 |= ADC_CR2_ADON;
    usleep(10);
    rCR2 |= ADC_CR2_ADON;
    usleep(10);

    if(rSR & ADC_SR_EOC) {
       rSR &= ~ADC_SR_EOC;
    }
    rCR2 |= ADC_CR2_DDS;
    rCR2 |= ADC_CR2_DMA;
    // rCR2 |= ADC_CR2_SWSTART;
   	PX4_INFO("ma40h1s start!");
    ret = OK;

    return ret;
}

int MA40H1S::probe()
{
    return OK;
}

void MA40H1S::set_minimum_distance(float min)
{
    _min_distance = min;
}

void MA40H1S::set_maximum_distance(float max)
{
    _max_distance = max;
}

float MA40H1S::get_minimum_distance()
{
    return _min_distance;
}

float MA40H1S::get_maximum_distance()
{
    return _max_distance;
}

int MA40H1S::ioctl(struct file *filp, int cmd, unsigned long arg)
{
    switch (cmd) {
        case SENSORIOCSPOLLRATE: {
                switch (arg) {

                /* switching to manual polling */
                case SENSOR_POLLRATE_MANUAL:
                    stop();
                    _measure_ticks = 0;
                    return OK;

                /* external signalling (DRDY) not supported */
                case SENSOR_POLLRATE_EXTERNAL:

                /* zero would be bad */
                case 0:
                    return -EINVAL;

                /* set default/max polling rate */
                case SENSOR_POLLRATE_MAX:
                case SENSOR_POLLRATE_DEFAULT: {
                        /* do we need to start internal polling? */
                        bool want_start = (_measure_ticks == 0);

                        /* set interval for next measurement to minimum legal value */
                        _measure_ticks = USEC2TICK(_cycling_rate);

                        /* if we need to start the poll state machine, do it */
                        if (want_start) {
                            start();

                        }

                        return OK;
                    }

                /* adjust to a legal polling interval in Hz */
                default: {
                        /* do we need to start internal polling? */
                        bool want_start = (_measure_ticks == 0);

                        /* convert hz to tick interval via microseconds */
                        int ticks = USEC2TICK(1000000 / arg);

                        /* check against maximum rate */
                        if (ticks < USEC2TICK(_cycling_rate)) {
                            return -EINVAL;
                        }

                        /* update interval for next measurement */
                        _measure_ticks = ticks;

                        /* if we need to start the poll state machine, do it */
                        if (want_start) {
                            start();
                        }

                        return OK;
                    }
                }
            }

        case SENSORIOCGPOLLRATE:
            if (_measure_ticks == 0) {
                return SENSOR_POLLRATE_MANUAL;
            }

            return (1000 / _measure_ticks);

        case SENSORIOCSQUEUEDEPTH: {
                /* lower bound is mandatory, upper bound is a sanity check */
                if ((arg < 1) || (arg > 100)) {
                    return -EINVAL;
                }

                irqstate_t flags = px4_enter_critical_section();

                if (!_reports->resize(arg)) {
                    px4_leave_critical_section(flags);
                    return -ENOMEM;
                }

                px4_leave_critical_section(flags);

                return OK;
            }

        case SENSORIOCGQUEUEDEPTH:
            return _reports->size();

        case SENSORIOCRESET:
            /* XXX implement this */
            return -EINVAL;

        case RANGEFINDERIOCSETMINIUMDISTANCE: {
                set_minimum_distance(*(float *)arg);
                return 0;
            }
            break;

        case RANGEFINDERIOCSETMAXIUMDISTANCE: {
                set_maximum_distance(*(float *)arg);
                return 0;
            }
            break;
        case RANGEFINDERSINGLEMEASURE: {
                single_test_mode = true;
                measure();
                return 0;
            }
            break;
        case RANGEFINDERSINGLECOLLECT: {
                collect();
                return (int)(distance_orginal*1000.0f);
            }
        case RANGEFINDERGETAVR:
            return average_sample;
        case RANGEFINDERGETTHR:
            return adc_buffer[_end_index[0]];
        default:
            /* give it to the superclass */
            return CDev::ioctl(filp, cmd, arg);
    }
}

ssize_t MA40H1S::read(struct file *filp, char *buffer, size_t buflen)
{
    // const size_t maxsize = sizeof(uint16_t) * ADC_BUFFER_SIZE;
    ssize_t ret = 0;
    if(buflen > ADC_BUFFER_SIZE-128){
        // buflen = ADC_BUFFER_SIZE-64;
        ret = 1;
    }

    memcpy(buffer, adc_buffer + buflen, 128);
    // for (int i = 0; i < 64; ++i)
    // {
    //     /* code */
    //     buffer[i] = adc_buffer[buflen+i];
    // }

    return ret;
}

int
MA40H1S::measure()
{
    int ret;
    /**
    * Send pulse sequence to begin a measurement
    */
    // trig();
    // stm32_gpiowrite(_gpio_tab.dr_a_port,true);
    // usleep(10);
    // stm32_gpiowrite(_gpio_tab.dr_a_port,false);
    // stm32_configgpio(_gpio_tab.adc_port);
    // stm32_configgpio(_gpio_tab.dr_a_port);
    // stm32_configgpio(_gpio_tab.dr_b_port);
    //stm32_configgpio(_gpio_tab.sw_a_port);  
    // stm32_configgpio(_gpio_tab.sw_b_port);
    // stm32_dmastart(_tx1_dma, nullptr, this, false);
    // printf("L%u\n", getreg32(0x40026000));
    // printf("H%u\n", getreg32(0x40026004));
   // printf("aa%u\n", getreg32(0x40026428));
	//printf("lt%u\n", getreg32(0x40026400));
	//printf("fifo%u\n", getreg32(0x40026442));
   // printf("cnt%u\n", getreg32(0x4002642c));
    //printf("a%u\n",  getreg32(0x40026410));
    // printf("pa%u\n", getreg32(0x40026018));
    // printf("ma%u\n", getreg32(0x4002601c));
    // printf("ff%u\n", getreg32(0x40026024));
    // stm32_configgpio(_gpio_tab.adc_port);

    uint16_t val = getreg16(STM32_TIM13_CR1);
    val |= ATIM_CR1_CEN;  // counter enable
    putreg16(val, STM32_TIM13_CR1);

    trig_state = 0;
    ret = OK;
    return ret;
}

int MA40H1S::collect()
{
    int ret = -EIO;

	// static uint8_t err_count = 0;
 //    static float pre_distance_m = 0.283f;
 //    static uint64_t _data_invalid = 0;
	// static float sonar_sample[10] = {0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f,0.283f};

	//static float sonar_deta[20] = {1.2f,1.2f,1.2f,1.0f,1.0f,1.0f,0.8f,0.8f,0.8f,0.6f,0.6f,0.6f,0.4f,0.4f,0.4f,0.3f,0.3f,0.3f,0.2f,0.2f};

    static uint64_t landed_time = 0;
	
    if (trig_state == 5) {
        if (_adc2_end) {
            _end_index[(uint8_t)_ultrasound_id[0]] = sonar_decoder_c((int16_t *)adc_buffer,(uint16_t)ADC_BUFFER_SIZE);
            memset(adc_buffer, 0, sizeof(adc_buffer));
            _adc2_end = false;
            if(_end_index[(uint8_t)_ultrasound_id[0]] != 1){
                new_value[(uint8_t)_ultrasound_id[0]] = true;
            } else {
                new_value[(uint8_t)_ultrasound_id[0]] = false;
            }
        } 
    #ifdef MULTI_ADC
        if (_adc3_end) {
            _end_index[(uint8_t)_ultrasound_id[1]] = sonar_decoder_c((int16_t *)adc_buffer_1,(uint16_t)ADC_BUFFER_SIZE);
            memset(adc_buffer_1, 0, sizeof(adc_buffer_1));
            _adc3_end = false;
            // printf("[@ma40h1s.cpp][collect()]_end_index_1 = %u\n", _end_index_1);
            if(_end_index[(uint8_t)_ultrasound_id[1]] != 1){
                new_value[(uint8_t)_ultrasound_id[1]] = true;
            } else {
                new_value[(uint8_t)_ultrasound_id[1]] = false;
            }
        }
    #endif     
    }
 
	int  _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	orb_unsubscribe(_vehicle_land_detected_sub);
	
	int  _armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	orb_unsubscribe(_armed_sub);
	
    struct vehicle_local_position_setpoint_s lpsp = {};
    int vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
    orb_copy(ORB_ID(vehicle_local_position_setpoint), vehicle_local_position_setpoint_sub, &lpsp);
    orb_unsubscribe(vehicle_local_position_setpoint_sub);

    // struct rtps_send_distance_sensor_s rtps_send_report = {} ; //bymark send this data to TX1 via RTPS

    uint8_t j = 0;
#ifdef MULTI_ADC
    for (j=0; j<2; j++)
#endif 
    // for (uint8_t k=0; k<NUM_OF_ULTRASOUND; k++)    
    {
        uint8_t k = (uint8_t)_ultrasound_id[j];
        float distance_m  = 0.283f;
        float average = 0.0f;
        float sum = 0.0f;
        float sum_2 = 0.0f;
        float variance = 0.0f;

        distance_orginal = 0.0f;

        if(_vehicle_land_detected.landed){
            err_count[k] = 0;
        }
        
        if(_armed.armed && _vehicle_land_detected.landed && lpsp.throw_state>50){
            distance_m=0.283f;
            pre_distance_m[k]=0.283f;
            landed_time = hrt_absolute_time();
            goto out;
        }

        if(!_vehicle_land_detected.landed && (hrt_absolute_time() - landed_time <1e6)){
            distance_m=0.283f;
            pre_distance_m[k]=0.283f;
            goto out;
        }

        // if(lpsp.throw_state==2 || lpsp.throw_state==1){
        //     distance_m=1.2f;
        //     goto out;
        // }  

        if(new_value[k] == true){
            distance_m = ((float)_end_index[k]*0.00229f)*0.17f;   // 0.00042823 340 m/s  --> 0.17 m/ms ; 2.29us
            // printf("[@ma40h1s.cpp][init()]distance_m = %.2f\n", distance_m);
            new_value[k] = false;
            //err_count = 0;
            _data_invalid[k] = 0;
        } else {
            _echo_valid = false;
            _echo_count = 0; 
            _data_invalid[k] ++;
            //err_count ++;
            if(!_armed.armed && _data_invalid[k] > 4){
                distance_m=0.283f;
                pre_distance_m[k]=0.283f;
                goto out;
            }
            return ret;
        }

    out:
        distance_orginal = distance_m;
        
        if(!_armed.armed){
            pre_distance_m[k] = distance_m;
        }

        if(err_count[k] > 1 && !_vehicle_land_detected.landed){
            pre_distance_m[k] = distance_m;
        }
        //PX4_WARN("land %.2f",(double)_vehicle_land_detected.landed);  
        if(fabsf(distance_m-pre_distance_m[k])>0.25f){
            distance_m = pre_distance_m[k];
            err_count[k] ++;
        }else{
            err_count[k] = 0;
            pre_distance_m[k] = distance_m;    
        }

        if(distance_m < 0.28f){
           distance_m = 0.28f;
        }
        if(pre_distance_m[k] < 0.28f){
            pre_distance_m[k] = 0.28f;
        }

        for(int i=8;i>=0;i--){
           sonar_sample[k][i+1] = sonar_sample[k][i];
        }
        sonar_sample[k][0] = distance_m;
        for(int i=0;i<10;i++){
           sum += sonar_sample[k][i];
        }
        average = sum/10;

        for(int i=0;i<10;i++){
           sum_2 += (sonar_sample[k][i] - average)*(sonar_sample[k][i] - average);
        }
        if((!_vehicle_land_detected.landed && (hrt_absolute_time() - landed_time <1200000))){
           variance = 0.0f;
        }else{
           variance = sum_2/10;
        }

        struct distance_sensor_s report = {};  
        report.timestamp = hrt_absolute_time();
        report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
        report.orientation = err_count[k];
        report.current_distance = distance_m; //distance_mid;
        report.min_distance = get_minimum_distance();
        report.max_distance = get_maximum_distance();
        report.sonar_test = distance_orginal;
        report.covariance = variance;
        /* TODO: set proper ID */
        report.id = (uint8_t)_ultrasound_id[0];//uint32_t(distance_mid*100.0f);
#ifdef MULTI_ADC 
        if (k != 0) {
            report.id = (uint8_t)_ultrasound_id[1]; 
        }     
#endif


#if NUM_OF_ULTRASOUND >1
        static float distance_1 = 0; // To show the distance of ultrasound_1 via QGC
#endif

#if NUM_OF_ULTRASOUND >2
        static float distance_2 = 0;  // To show the distance of ultrasound_2 via QGC
#endif

#ifndef MULTI_ADC
        if (_ultrasound_id[0] == MA40H1S_ID_DOWN && (uint8_t)_ultrasound_id[0] == k && _distance_sensor_pub[0] != nullptr) {   
        // ultrasound_0: MA40H1S_ID_DOWN : down
    #if NUM_OF_ULTRASOUND >1
            report.max_distance = distance_1; // bymark  to show ultrasound_1: right on QGC 
    #endif

    #if NUM_OF_ULTRASOUND >2
            report.min_distance = distance_2; // bymark  to show ultrasound_2: left  on QGC
    #endif
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[0], &report);
        }
    #if NUM_OF_ULTRASOUND > 1           
        else if (_ultrasound_id[0] == MA40H1S_ID_RIGHT && (uint8_t)_ultrasound_id[0] == k && _distance_sensor_pub[1] != nullptr) { 
        // ultrasound_1: MA40H1S_ID_RIGHT : right
            distance_1 = distance_m; //bymark
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[1], &report);
        }
        // printf("[@ma40h1s.cpp][collect()]k is %d, ultasound_id[0] is %d\n", k, (uint8_t)_ultrasound_id[0]);
    #endif

    #if NUM_OF_ULTRASOUND > 2
        else if (_ultrasound_id[0] == MA40H1S_ID_LEFT && (uint8_t)_ultrasound_id[0] == k && _distance_sensor_pub[2] != nullptr) {  
        // ultrasound_2: MA40H1S_ID_LEFT : left
            distance_2 = distance_m; // bymark
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[2], &report);
        }
    #endif
#endif

#ifdef MULTI_ADC
        if (_ultrasound_id[0] == MA40H1S_ID_DOWN && (uint8_t)_ultrasound_id[0] == k && _distance_sensor_pub[0] != nullptr) { 
        // if (k == (uint8_t)MA40H1S_ID_DOWN && _distance_sensor_pub[0] != nullptr) {   
        // ultrasound_0: MA40H1S_ID_DOWN : down
            report.max_distance = distance_1; // bymark  to show ultrasound_1: right on QGC
    #if NUM_OF_ULTRASOUND >2
            report.min_distance = distance_2; // bymark  to show ultrasound_2: left  on QGC
    #endif
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[0], &report);
        }

    #if NUM_OF_ULTRASOUND > 1
        if (_ultrasound_id[1] == MA40H1S_ID_RIGHT && (uint8_t)_ultrasound_id[1] == k && _distance_sensor_pub[1] != nullptr) {
        // if (k == (uint8_t)MA40H1S_ID_RIGHT && _distance_sensor_pub[1] != nullptr) {  
        // ultrasound_1: MA40H1S_ID_RIGHT : right
            distance_1 = distance_m; // bymark
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[1], &report);
        }
    #endif

    #if NUM_OF_ULTRASOUND > 2
        if (_ultrasound_id[1] == MA40H1S_ID_LEFT && (uint8_t)_ultrasound_id[1] == k && _distance_sensor_pub[2] != nullptr) {
        // if (k == (uint8_t)MA40H1S_ID_LEFT && _distance_sensor_pub[2] != nullptr) {   
        // ultrasound_2: MA40H1S_ID_LEFT : left
            distance_2 = distance_m; // bymark
            // rtps_send_report.current_distance[k] = distance_m;
            orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub[2], &report);
        }
    #endif     
#endif    
        _reports->force(&report);       
    }  // end of for(){}


    // rtps_send_report.timestamp = hrt_absolute_time();
    // rtps_send_report.n_ultrasound = (uint8_t)NUM_OF_ULTRASOUND;
    // if (_rtps_send_distance_pub != nullptr) { 
    //     orb_publish(ORB_ID(rtps_send_distance_sensor), _rtps_send_distance_pub, &rtps_send_report);
    // }    

#if NUM_OF_ULTRASOUND > 1   // when NUM_OF_ULTRASOUND is more than 2, using ADC2 and ADC3,or using only ADC2   Async. > 1 ;  Sync. : > 2 
    if (trig_state == 5) {
        if ((++_ultrasound_n) >= NUM_OF_ULTRASOUND) _ultrasound_n = (NUM_OF_ULTRASOUND>2 ? 1 : 0);  // Async. = 0 ;  Sync. : = 1
        _ultrasound_id[NUM_OF_ULTRASOUND>2 ? 1 : 0] = _ultrasound_config[_ultrasound_n].id;  // Async. [0] ;  Sync. : [1]

      	uint16_t channel_mask = (1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));//  0b00110000 <==> pwm6 and pwm5
		for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
			if (channel_mask & (1 << channel)) {
				if (-EBUSY == io_timer_is_channel_free(channel)) {
					io_timer_free_channel(channel);
				}
				io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);
				channel_mask &= ~(1 << channel);
			}
		}

		io_timer_set_ccr(_ultrasound_config[_ultrasound_n].pwm1_ch-1, 12);
		io_timer_set_ccr(_ultrasound_config[_ultrasound_n].pwm2_ch-1, 12);
		io_timer_set_rate(_ultrasound_config[_ultrasound_n].timer_index, 40000); //timer_index 1: TIM4   timer_index 2: TIM12 
		
    #ifndef MULTI_ADC   
        rSQR3 = _ultrasound_config[_ultrasound_n].adc_ch;  
    #endif
    #ifdef MULTI_ADC
        r_SQR3 = _ultrasound_config[_ultrasound_n].adc_ch;
    #endif
    }
#endif

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

    ret = OK;

    return ret;
}

void MA40H1S::start()
{

    /* reset the report ring and state machine */
    //_collect_phase = false;
    _reports->flush();

    measure();  /* begin measure */

    /* schedule a cycle to start things */
    work_queue(HPWORK,
           &_work,
           (worker_t)&MA40H1S::cycle_trampoline,
           this,
           USEC2TICK(_cycling_rate));
}

void MA40H1S::stop()
{
    work_cancel(HPWORK, &_work);
}

void MA40H1S::cycle_trampoline(void *arg)
{

    MA40H1S *dev = (MA40H1S *)arg;

    dev->cycle();

}

void MA40H1S::cycle()
{
    /* perform collection */
    if(!single_test_mode){
        if (OK != collect()) {
            DEVICE_DEBUG("collect error");
        }   // kecf

        /* next sonar */
        if (OK != measure()) {
            DEVICE_DEBUG("measure error");
        }
    }

    work_queue(HPWORK,
           &_work,
           (worker_t)&MA40H1S::cycle_trampoline,
           this,
           USEC2TICK(_cycling_rate));
}

void MA40H1S::print_info()
{
    PX4_WARN("absolute time:%lld",_end_time - _start_time);
    // PX4_WARN("absolute time:%lld", _end_time);

    PX4_WARN("poll interval:  %u ticks\n", _measure_ticks);
    _reports->print_info("report queue");
}

void MA40H1S::trig()
{
    trig_state = 0;
}

void MA40H1S::interrupt(hrt_abstime time)
{
	if(_echo_valid){
		// printf("v\n");
		_echo_count++;
		if(_echo_count >= 2){
			// we have waited for too long, so clear the state
			if(hrt_absolute_time() - _start_time > 30000){
				_echo_count = 0;
				_echo_valid = false;
				return;
			}
			_echo_valid = false;
			_echo_count = 0;
			new_value[0] = true;
		}
	}
}

void MA40H1S::test_high()
{
    static uint16_t tick=0;
    printf("%d\n",adc_buffer[tick]);
    tick++;
    if (tick==ADC_BUFFER_SIZE)
    {
        /* code */
        tick = 0;
    }
    // hrt_abstime now = hrt_absolute_time();
    // while (!(rSR & ADC_SR_EOC)) {
    //     // don't wait for more than 50us, since that means something broke - should reset here if we see this 
    //     if ((hrt_absolute_time() - now) > 50) {
    //         printf("sample timeout\n");
    //         return;
    //     }
    // }
    printf("%d\n",rDR);
}

void MA40H1S::test_low()
{
    rCR2 |= ADC_CR2_SWSTART;
    printf("tr:%d\n", trig_state);
}

void MA40H1S::tick_trampoline(void *arg)
{
}

int MA40H1S::timer13_interrupt(int irq, void *context, void *arg)
{
#ifdef ADC_LOG
    enum MA40H1S_ID *pdev_id = ((interrupt_arg *)arg)->pdev_id;
    orb_advert_t *adc_pub = ((interrupt_arg *)arg)->adc_pub;
#endif

// #ifndef ADC_LOG    
//     enum MA40H1S_ID *pdev_id = (enum MA40H1S_ID *)arg;
// #endif

    static uint16_t ticks = 0;
    putreg16(0xFFFE,STM32_TIM13_SR); // putreg16(0xFFFE,0x40000c10); //STM32_TIM5_BASE:0x40000c00    STM32_GTIM_SR_OFFSET:0x0010
    
	switch(trig_state){
		case 0:
			// stm32_gpiosetevent(_gpio_tab.adc_port, true, false, false, nullptr);
			//stm32_gpiowrite(_gpio_tab.sw_a_port,false);
			trig_state = 1;
			ticks = 0;
			break;
		case 1:
			ticks++;
			if(ticks >= 20){     //bymark 20 -> 20*20
				trig_state = 2;
				ticks = 0;
        #ifdef MULTI_ADC
            #if NUM_OF_ULTRASOUND > 2
                // if (*(pdev_id+1) == MA40H1S_ID_RIGHT) {
                //     uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                //          1<<(_ultrasound_config[1].pwm1_ch-1) | 1<<(_ultrasound_config[1].pwm2_ch-1));
                //     io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask);
                // }
                // else if (*(pdev_id+1) == MA40H1S_ID_LEFT) {
                //     uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                //          1<<(_ultrasound_config[2].pwm1_ch-1) | 1<<(_ultrasound_config[2].pwm2_ch-1));
                //     io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask);
                // }
                uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                     1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));
                io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask);
            #endif 
        #endif

        #ifndef MULTI_ADC
                // putreg16(0x0145,STM32_TIM8_DIER);// putreg16(0x0145,0x4001040c);//TIM8 STM32_TIM8_BASE:0x40010400 STM32_GTIM_DIER_OFFSET:0x000c  1 0100 0101
     //           if ((*pdev_id) == MA40H1S_ID_DOWN) {
     //                uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1));
					// io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask);  // 0b00110000  PWM-ch5 PWM-ch6
     //                // printf("enable timer, device id = %d\n", (int)(*pdev_id));
     //           }

     //       #if NUM_OF_ULTRASOUND > 1
     //           else if ((*pdev_id) == MA40H1S_ID_RIGHT) {
     //                uint16_t channel_mask = (1<<(_ultrasound_config[1].pwm1_ch-1) | 1<<(_ultrasound_config[1].pwm2_ch-1));
					// io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask); // 0b11000000 PWM-ch7 PWM-ch8  --> 0b00001100
     //           }
     //       #endif
                uint16_t channel_mask = (1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));
                io_timer_set_enable(true, IOTimerChanMode_PWMOut, channel_mask);  
       #endif    

			}
			break;
		case 2:
			ticks++;
			if(ticks >= 40){
				// putreg16(0x0000,0x4001040c);  STM32_GPIOD_BSRR
                // stm32_gpiowrite(_dr_b_port,false);
        #ifdef MULTI_ADC
            #if NUM_OF_ULTRASOUND > 2
                // if (*(pdev_id+1) == MA40H1S_ID_RIGHT) {
                //     uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                //          1<<(_ultrasound_config[1].pwm1_ch-1) | 1<<(_ultrasound_config[1].pwm2_ch-1));
                //     io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask);
                // }
                // else if (*(pdev_id+1) == MA40H1S_ID_LEFT) {
                //     uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                //          1<<(_ultrasound_config[2].pwm1_ch-1) | 1<<(_ultrasound_config[2].pwm2_ch-1));
                //     io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask);
                // }
                uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1) |
                     1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));
                io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask);
            #endif  
        #endif

        #ifndef MULTI_ADC
     //            if ((*pdev_id) == MA40H1S_ID_DOWN) {
    	// 			// io_timer_set_ccr(_ultrasound_config[0].pwm1_ch-1, 25);
					// // io_timer_set_ccr(_ultrasound_config[0].pwm2_ch-1, 25);
					// // putreg32(GPIO_BSRR_RESET(13) | GPIO_BSRR_RESET(14), STM32_GPIOD_BSRR); // PD13 and PD14
     //                uint16_t channel_mask = (1<<(_ultrasound_config[0].pwm1_ch-1) | 1<<(_ultrasound_config[0].pwm2_ch-1));
     //                io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask);  // 0b00110000  PWM-ch5 PWM-ch6
     //            }

     //        #if NUM_OF_ULTRASOUND > 1
     //            else if ((*pdev_id) == MA40H1S_ID_RIGHT) {
     //                // printf("disable timer, device id = %d\n", (int)(*pdev_id));
     //                uint16_t channel_mask = (1<<(_ultrasound_config[1].pwm1_ch-1) | 1<<(_ultrasound_config[1].pwm2_ch-1));
     //                io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask); // 0b11000000 PWM-ch7 PWM-ch8  --> 0b00001100
     //            }
     //        #endif
            uint16_t channel_mask = (1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));
            io_timer_set_enable(false, IOTimerChanMode_PWMOut, channel_mask);    
        #endif    
				trig_state = 3;
				ticks = 0;
			}
			break;
		case 3:
			ticks++;
			if(ticks>=40){
				ticks=0;
				_echo_valid = true;
				_echo_count = 0;
				// stm32_gpiosetevent(_gpio_tab.adc_port, true, false, false, sonar_isr);
				trig_state = 4;
                // printf("st:%llu\n", hrt_absolute_time());                
			}
			break;
        case 4:
            // ticks++;
            // if(ticks>=10){
                // ticks = 0;
                //rSR &= ~ADC_SR_EOC;
                //rCR2 |= ADC_CR2_SWSTART;
                // printf("t\n");
            // }
				// // start adc
            #ifdef MULTI_ADC
                r_SR &= ~ADC_SR_OVR;
                r_SR &= ~ADC_SR_EOC;
                r_CR2 |= ADC_CR2_CONT;
                r_CR2 |= ADC_CR2_ADON;
                r_CR2 |= ADC_CR2_SWSTART;
            #endif
                rSR &= ~ADC_SR_OVR;
                rSR &= ~ADC_SR_EOC;
                rCR2 |= ADC_CR2_CONT;
                rCR2 |= ADC_CR2_ADON;
                rCR2 |= ADC_CR2_SWSTART;
                _start_time = hrt_absolute_time();
                trig_state = 5;
                ticks = 0;
                break;
#ifdef ADC_LOG
        case 6:
                ticks++;
                static uint8_t n = 0;
                if (ticks>=80*10) { // bymark 80*7 -> 80*10 : 10000 or 20000 -> 30000
                    ticks = 0;
                #ifndef MULTI_ADC
                    if (n < 50) { //bymark 50->100->150 : 10000 -> 20000 -> 30000
                        struct adc_ultrasound_s adc={}; // bymark
                        for (int i = n*200, j = 0; j < 200; i++, j++){
                            if (n == 49) {  //bymark 49->99->149 : 10000 -> 20000 -> 30000
                                adc.channel_value[j] = 0;
                            }
                            else {
                                adc.channel_value[j] = adc_buffer[i];
                            }
                        }
                        adc.timestamp = n;
                        n++;
                        if ((*adc_pub) != nullptr && (*pdev_id) == MA40H1S_ID_DOWN) {
                            orb_publish(ORB_ID(adc_ultrasound), *adc_pub, &adc);
                        }

                        #if NUM_OF_ULTRASOUND > 1
                        else if (*(adc_pub+1) != nullptr && (*pdev_id) == MA40H1S_ID_RIGHT) {
                            orb_publish(ORB_ID(adc_ultrasound), *(adc_pub+1), &adc);
                        }
                        #endif
                        #if NUM_OF_ULTRASOUND > 2
                        else if (*(adc_pub+2) != nullptr && (*pdev_id) == MA40H1S_ID_LEFT) {
                            orb_publish(ORB_ID(adc_ultrasound), *(adc_pub+2), &adc);
                        }
                        #endif
                    } 
                    else { 
                        // uint16_t val = getreg16(STM32_TIM13_CR1);
                        // val &= ~ATIM_CR1_CEN;
                        // putreg16(val, STM32_TIM13_CR1);
                        // // uint16_t end_index = sonar_decoder_c((int16_t *)adc_buffer,(uint16_t)ADC_BUFFER_SIZE);  //bymark measure
                        // // float distance_m = ((float)_end_index*0.00229f)*0.17f;   // 0.00042823 340 m/s  --> 0.17 m/ms ; 2.29us
                        // // printf("[@ma40h1s.cpp][timer13_interrupt]end_index = %d\n", end_index);

                        // val = getreg16(STM32_TIM13_CR1);
                        // val |= ATIM_CR1_CEN;
                        // putreg16(val, STM32_TIM13_CR1);

                        (*pdev_id) = _ultrasound_config[_ultrasound_n].id;
                        n = 0;
                        trig_state = 0;
                    }
                #endif

                #ifdef MULTI_ADC
                    if (_adc2_end && _adc3_end) {
                        if (n < 50) { //bymark 50->100->150 : 10000 -> 20000 -> 30000
                            struct adc_ultrasound_s adc={}; // bymark
                            for (int i = n*200, j = 0; j < 200; i++, j++){
                                if (n == 49) {  //bymark 49->99->149 : 10000 -> 20000 -> 30000
                                    adc.channel_value[j] = 0;
                                }
                                else {
                                    adc.channel_value[j] = adc_buffer[i];
                                }
                            }
                            adc.timestamp = n;
                            if ((*adc_pub) != nullptr && (*pdev_id) == MA40H1S_ID_DOWN) {
                                orb_publish(ORB_ID(adc_ultrasound), *adc_pub, &adc);
                            }

                        #if NUM_OF_ULTRASOUND > 1
                            for (int i = n*200, j = 0; j < 200; i++, j++){
                                if (n == 49) {  //bymark 49->99->149 : 10000 -> 20000 -> 30000
                                    adc.channel_value[j] = 0;
                                }
                                else {
                                    adc.channel_value[j] = adc_buffer_1[i];
                                }
                            }
                            adc.timestamp = n;
                            if (*(adc_pub+1) != nullptr && *(pdev_id+1) == MA40H1S_ID_RIGHT) {
                                orb_publish(ORB_ID(adc_ultrasound), *(adc_pub+1), &adc);
                            }
                        #endif

                        #if NUM_OF_ULTRASOUND > 2
                            else if (*(adc_pub+2) != nullptr && *(pdev_id+1) == MA40H1S_ID_LEFT) {
                                orb_publish(ORB_ID(adc_ultrasound), *(adc_pub+2), &adc);
                            }
                        #endif
                            n++;
                        } 
                        else { 
                            *(pdev_id+1) = _ultrasound_config[_ultrasound_n].id;
                            _adc2_end = false;
                            _adc3_end = false;
                            n = 0;
                            trig_state = 0;
                        }
                    }
                #endif
                }
                break;
#endif
		default:
		 //    if(rSR & ADC_SR_EOC) {
		 //       uint32_t adc_value;
		 //       adc_value = rDR;
		 //       PX4_INFO("adc2_value = %d\n", adc_value);
		 //       rSR &= ~ADC_SR_EOC;
		 //    }
			// //PX4_INFO("timer5_interrupt running");
			break;
	}

	return OK;
}

#ifdef MULTI_ADC
void MA40H1S::_dma_callback_1(DMA_HANDLE handle, uint8_t status, void *arg)
{
    if (arg != nullptr) {
        MA40H1S *ps = reinterpret_cast<MA40H1S *>(arg);
        ps->_do_adc_dma_callback_1(status);
    }
}

void MA40H1S::_do_adc_dma_callback_1(unsigned status)
{
    trig_state = 5;
    _end_time = hrt_absolute_time();

    r_CR2 &= ~ADC_CR2_ADON;
    // printf("_ultrasound_id = %d\n", _ultrasound_id);
    // printf("_deltaT = %llu\n", _end_time - _start_time);
    stm32_dmastart(_adc_dma_1, _dma_callback_1, this, false);

    _adc3_end = true;

#ifdef ADC_LOG
    trig_state = 6;  // bymark measure
#endif
 
#ifndef ADC_LOG
    uint16_t val = getreg16(STM32_TIM13_CR1);
    val &= ~ATIM_CR1_CEN;
    putreg16(val, STM32_TIM13_CR1);
    // _end_index = sonar_decoder_c((int16_t *)adc_buffer,(uint16_t)ADC_BUFFER_SIZE);  //bymark measure
    // printf("[@ma40h1s_JZL.cpp][_do_adc_dma_callback_1()]\n");
#endif

}
#endif

void
MA40H1S::_dma_callback(DMA_HANDLE handle, uint8_t status, void *arg)
{
    if (arg != nullptr) {
        MA40H1S *ps = reinterpret_cast<MA40H1S *>(arg);
        ps->_do_adc_dma_callback(status);
       	// printf("et:%llu\n",hrt_absolute_time());
    }
}

void MA40H1S::_do_adc_dma_callback(unsigned status)
{
    trig_state = 5;
    _end_time = hrt_absolute_time();

	rCR2 &= ~ADC_CR2_ADON;
    // printf("_deltaT = %llu\n", _end_time - _start_time);
    stm32_dmastart(_adc_dma, _dma_callback, this, false);
    _adc2_end = true;

    // uint16_t val = getreg16(STM32_TIM13_CR1);
    // val &= ~ATIM_CR1_CEN;
    // putreg16(val, STM32_TIM13_CR1);

    // printf("[@ma40h1s_JZL.cpp][timer13_interrupt()]single_test_mode = %d\n", single_test_mode);
    // hrt_abstime temptime = hrt_absolute_time();
    // static int num=0;
    // num++;
    // printf("[@ma40h1s_JZL.cpp][_do_adc_dma_callback()]\n");

#ifdef ADC_LOG
    #if NUM_OF_ULTRASOUND > 1 //bymark   Async: >1 ; Sync: >2 when recording ADC log
    if (trig_state == 5) {
        if ((++_ultrasound_n) >= NUM_OF_ULTRASOUND) _ultrasound_n = (NUM_OF_ULTRASOUND>2 ? 1 : 0);  //  Async: =0 ; Sync: =1 when recording ADC log
        // _ultrasound_id = _ultrasound_config[_ultrasound_n].id;

        uint8_t channel_mask = (1<<(_ultrasound_config[_ultrasound_n].pwm1_ch-1) | 1<<(_ultrasound_config[_ultrasound_n].pwm2_ch-1));//  0b00110000 <==> pwm6 and pwm5
        for (unsigned channel = 0; channel_mask != 0 &&  channel < MAX_TIMER_IO_CHANNELS; channel++) {
            if (channel_mask & (1 << channel)) {
                if (-EBUSY == io_timer_is_channel_free(channel)) {
                    io_timer_free_channel(channel);
                }

                io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);
                channel_mask &= ~(1 << channel);
            }
        }

        io_timer_set_ccr(_ultrasound_config[_ultrasound_n].pwm1_ch-1, 12);
        io_timer_set_ccr(_ultrasound_config[_ultrasound_n].pwm2_ch-1, 12);
        io_timer_set_rate(_ultrasound_config[_ultrasound_n].timer_index, 40000); //timer_index 1: TIM4   timer_index 2: TIM12 
        
        rSQR3 = _ultrasound_config[_ultrasound_n].adc_ch;
    }
    #endif
    trig_state = 6;  // bymark measure
#endif
 
#ifndef ADC_LOG
    uint16_t val = getreg16(STM32_TIM13_CR1);
    val &= ~ATIM_CR1_CEN;
    putreg16(val, STM32_TIM13_CR1);
    // _end_index = sonar_decoder_c((int16_t *)adc_buffer,(uint16_t)ADC_BUFFER_SIZE);  //bymark measure
#endif
    // printf("[@ma40h1s_JZL.cpp][_do_adc_dma_callback()]orb_deltaT = %llu\n", hrt_absolute_time() - temptime);
    // float distance_m = ((float)_end_index*0.00229f)*0.17f;   // 0.00042823 340 m/s  --> 0.17 m/ms ; 2.29us
    // printf("[@ma40h1s.cpp][init()]distance_m = %.2f\n", distance_m);
    // printf("[@ma40h1s.cpp][init()]_end_index = %d\n", _end_index);

    // if(_end_index != 1){
    //     new_value = true;
    // } else {
    //     new_value = false;
    // }

    // val = getreg16(STM32_TIM13_CR1);
    // val |= ATIM_CR1_CEN;
    // putreg16(val, STM32_TIM13_CR1);
}


/**
 * Local functions in support of the shell command.
 */
namespace ma40h1s
{
#ifdef ERROR
    #undef ERROR
#endif
const int ERROR = -1;

MA40H1S *g_dev;

void start();
void stop();
void test();
void reset();
void trig();
void info();
void test_high();
void test_low();

/**
* Start the driver
*/
void start()
{
    int fd;

    if(g_dev != nullptr){
        errx(1,"already started");
    }

    /* creat the driver */
    g_dev = new MA40H1S();

    if(g_dev == nullptr){
        goto fail;
    }

    if(OK != g_dev->init()){
        goto fail;
    }

    fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0){
        goto fail;
    }

#ifndef ADC_LOG
    if(ioctl(fd,SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
#endif
#ifdef ADC_LOG    
   	if(ioctl(fd,RANGEFINDERSINGLEMEASURE, SENSOR_POLLRATE_DEFAULT) < 0)    	// bymark single measure
#endif 
    {   
        goto fail;
    }

    exit(0);

fail:

    if(g_dev != nullptr){
        delete g_dev;
        g_dev = nullptr;
    }

    errx(1,"driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
    if (g_dev != nullptr) {
        delete g_dev;
        g_dev = nullptr;

    } else {
        errx(1, "driver not running");
    }

    exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void test()
{
    struct distance_sensor_s report;
    ssize_t sz;
    int ret;

    int fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0 ){
        err(1,"open failed");
    }

    sz = read(fd, &report, sizeof(report));

    if(sz != sizeof(report)){
        err(1,"immediate read failed");
    }

    warnx("single read");
    warnx("time:        %llu",report.timestamp);

    /* start the sensor polling at 2Hz */
    if(OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)){
        errx(1, "failed to set 2Hz poll rate");
    }

    for (unsigned i = 0; i < 5; i++)
    {
        struct pollfd fds;

        fds.fd = fd;
        fds.events = POLLIN;
        ret = poll(&fds,1,2000);

        if(ret != 1){
            errx(1,"timed out waiting for sensor data");
        }

        /* now go get it */
        sz = read(fd, & report, sizeof(report));

        if(sz != sizeof(report)){
            err(1,"periodic read failed");
        }

        warnx("periodic read %u", i);
        warnx("time:        %llu", report.timestamp);
    }

    /* reset the sensor polling to default rate */
    if(OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)){
        errx(1,"failed to set default poll rate");
    }

    errx(0,"PASS");
}

/**
 * Reset the driver.
 */
 void reset()
 {
    int fd = open(MA40H1S_DEVICE_PATH,O_RDONLY);

    if(fd < 0 ){
        err(1,"failed ");
    }

    if(ioctl(fd, SENSORIOCRESET, 0) < 0){
        err(1,"driver reset failed");
    }

    if(ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0){
        err(1, "driver poll restart failed");
    }

    exit(0);
 }

 void trig()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->trig();
    exit(0);
 }

 void test_high()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->test_high();
    exit(0);    
 }

 void test_low()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    g_dev->test_low();
    exit(0);    
 }


/**
 * Print a little info about the driver.
 */
 void info()
 {
    if(g_dev == nullptr){
        errx(1,"driver not running");
    }

    printf("stage @ %p\n",g_dev);
    g_dev->print_info();

    exit(0);
 }

}/* namespace */

// Handle the echo GPIO interrupt
// static int sonar_isr(int irq, void *context)
// {
//     // static int i = 0;
//     hrt_fabsftime time = hrt_fabsfolute_time();
//     // if(i++ > 80){
//     //  i = 0;
//     //  printf("i%llu\n", time);
//     // }
//     // printf("o\n");
//     if(ma40h1s::g_dev != nullptr){
//         ma40h1s::g_dev->interrupt(time);
//     }

//     return OK;
// }


int ma40h1s_main(int argc, char *argv[])
{
    /*
     * Start/load the driver.
     */
    if (!strcmp(argv[1], "start")) {
        ma40h1s::start();
    }

    /*
     * Stop the driver
     */
    if (!strcmp(argv[1], "stop")) {
        ma40h1s::stop();
    }

    /*
     * Test the driver/device.
     */
    if (!strcmp(argv[1], "test")) {
        ma40h1s::test();
    }

    /*
     * Reset the driver.
     */
    if (!strcmp(argv[1], "reset")) {
        ma40h1s::reset();
    }

    // send a trig signal
    if (!strcmp(argv[1], "trig")) {
        ma40h1s::trig();
    }

    if (!strcmp(argv[1], "high")) {
        ma40h1s::test_high();
    }

    if (!strcmp(argv[1], "low")) {
        ma40h1s::test_low();
    }

    /*
     * Print driver information.
     */
    if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
        ma40h1s::info();
    }

    errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");  
}
