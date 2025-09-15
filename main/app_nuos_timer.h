#ifndef _APP_NUOS_TIMER_H_
    #define _APP_NUOS_TIMER_H_


    void nuos_start_commissioning();
    void nuos_stop_commissioning(uint8_t flag);
    #ifdef __cplusplus
    extern "C" {
    #endif
    void esp_start_timer();
    void esp_stop_timer();
    void init_timer();
    void start_color_temp_timer();
    void stop_color_temp_timer();    
    #ifdef __cplusplus
    }
    #endif

    extern uint8_t get_button_pressed_mode();
    extern void actionOnTwoSwitchPressed(int64_t timeout);
    extern uint32_t IdentifyTwoSwitchPressed();
    extern void initTwoSwitchPressedPins();
    void init_timer_2();
    void esp_start_timer_2();
    void esp_stop_timer_2();
    void recheckTimer();

    void init_timer_3();
    void esp_start_timer_3();
    void esp_stop_timer_3();  
    void init_color_temp_timer(); 

    
#endif
    