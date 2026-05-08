#include "DaliCommands.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dali_receiver.h"

#define DELAY_COMMAND_SEND 50
#define TAG "DaliCommands"

bool group_state = false;
struct DaliMessage {
    uint8_t data[3];
    size_t len;
};
static dali_rx::Receiver            receiver;
// Queue handle for passing messages from callback to task
static QueueHandle_t dali_msg_queue = nullptr;
// static QueueHandle_t rxFrameQueue = nullptr;           // Queue for received frames (each is uint32_t)
extern bool isr_service_installed;
////////////////////////////////////////////////////////////////////////////////////////////////////
DaliCommands::DaliCommands(gpio_num_t txPin, gpio_num_t rxPin)
    : txPin(txPin), rxPin(rxPin), daliCore(txPin, rxPin) {
   
    // Configure GPIO pins
    gpio_config_t io_conf_tx = {};
    io_conf_tx.intr_type = GPIO_INTR_DISABLE;
    io_conf_tx.mode = GPIO_MODE_OUTPUT;
    io_conf_tx.pin_bit_mask = (1ULL << txPin);
    io_conf_tx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_tx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_tx);

    gpio_config_t io_conf_rx = {};
    io_conf_rx.intr_type = GPIO_INTR_DISABLE;
    io_conf_rx.mode = GPIO_MODE_INPUT;
    io_conf_rx.pin_bit_mask = (1ULL << rxPin);
    io_conf_rx.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf_rx.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf_rx);
   
    ESP_LOGI(TAG, "DaliCommands initialized");
}


// DALI receiver callback (runs in timer task context)
static void on_dali_message(const uint8_t* data, size_t len) {
    // Copy message and send to queue
    DaliMessage msg;
    msg.len = len;
    memcpy(msg.data, data, len);

    // xQueueSend is safe because we are in task context (ESP_TIMER_TASK)
    if (xQueueSend(dali_msg_queue, &msg, 0) != pdTRUE) {
        ESP_EARLY_LOGE("DALI", "Queue full, message dropped");
    }
}


// DALI processing task
static void dali_task(void* arg) {

    QueueHandle_t pRxQueue = (QueueHandle_t)arg;
    printf("pRxQueue = %p\n", pRxQueue);
    DaliMessage msg;

    while (1) {
        // Wait for a message from the queue
        if (xQueueReceive(dali_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {
            // DaliCommands* driver = (DaliCommands*)context;
            // Process the DALI command
            printf("msg[0]: 0x%X msg[1]:0x%X\n", msg.data[0], msg.data[1]);
            
            if(pRxQueue != nullptr){
                xQueueSend(pRxQueue, &msg, 0);
            }
            
           
        }
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void DaliCommands::begin(bool* is_isr) {
    daliCore.begin(is_isr); 
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void DaliCommands::begin_rx(bool* is_isr, QueueHandle_t rxFrameQueue) {
     // Create a queue capable of holding up to 10 DALI messages
    dali_msg_queue = xQueueCreate(10, sizeof(DaliMessage));
    if (dali_msg_queue == nullptr) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }

 // Start the DALI receiver on GPIO 19 in C6 (active low, standard DALI)
    #ifdef IS_INVERTED
    esp_err_t err = receiver.begin(rxPin, on_dali_message, true, is_isr);
    #else
    esp_err_t err = receiver.begin(rxPin, on_dali_message, false, is_isr);
    #endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start DALI receiver");
        return;
    }
    // Create the DALI processing task (priority 14, stack 4096)
    xTaskCreate(dali_task, "dali_task", 4096, (void*)rxFrameQueue, 24, nullptr); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Basic Control Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void DaliCommands::turn_off(uint8_t nodeNumber) {
    
    daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x00);
    
}

void DaliCommands::turn_on_to_max(uint8_t nodeNumber) {
    
    daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x05);
    
}

void DaliCommands::turn_on_to_last_level(uint8_t nodeNumber) {
    
    daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01,
                               GO_TO_LAST_ACTIVE_LEVEL);
                               
}

void DaliCommands::factory_reset(uint8_t nodeNumber) {
    
    daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x20);
    daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e) | 0x01, 0x20);
    
}

void DaliCommands::set_dim_value(uint8_t nodeNumber, uint8_t value) {
    
    if (value == 0) {
        turn_off(nodeNumber);
    } else if (value == 255) {
        turn_on_to_max(nodeNumber);
    } else {
        daliCore.sendCommandPublic(SHORT_POWER | ((nodeNumber << 1) & 0x7e), value);
    }
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Command Sending Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void DaliCommands::send_command_special(uint8_t opcode, uint8_t address) {
    
    daliCore.sendCommandPublic(opcode, address);
    
}

void DaliCommands::send_command_standard(uint8_t opcode, uint8_t address) {
    // Get the upper bit
    uint8_t mask = address & 0x80;
    // Change address to have 1 in LSb to signify 'standard command'
    uint8_t new_address = mask | ((address << 1) + 1);
    
    daliCore.sendCommandPublic(new_address, opcode);
}

void DaliCommands::send_command_special32(uint8_t opcode1, uint8_t address1,
                                          uint8_t opcode2, uint8_t address2) {
    
    daliCore.sendCommandPublic32(opcode1, address1, opcode2, address2);
    
}

void DaliCommands::send_command_standard32(uint8_t opcode1, uint8_t address1,
                                           uint8_t opcode2, uint8_t address2) {
    
    // Get the upper bit for address 1
    uint8_t mask1 = address1 & 0x80;
    uint8_t new_address1 = mask1 | ((address1 << 1) + 1);
   
    // Get the upper bit for address 2
    uint8_t mask2 = address2 & 0x80;
    uint8_t new_address2 = mask2 | ((address2 << 1) + 1);
   
    daliCore.sendCommandPublic32(new_address1, opcode1, new_address2, opcode2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Color Temperature Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

void DaliCommands::set_color_temp(uint8_t addr, uint16_t kelvin) {
    // Calculate Mirek from Kelvin
    if(kelvin == 0) return;
    
    uint16_t mirek = 1000000 / kelvin;
    uint8_t dtr0 = (uint8_t)(mirek & 0x00FF);
    uint8_t dtr1 = (uint8_t)((mirek >> 8) & 0x00FF);
    // Set temperature
    send_command_special(SET_DTR0, dtr0);
    send_command_special(SET_DTR1, dtr1);
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    // Set the temporary color to the temperature
    send_command_standard(SET_COLOR_TEMP, addr);
    
}

void DaliCommands::set_color_temperature(uint8_t addr, uint16_t temp) {
    
    set_color_temp(addr, temp);
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// RGB Color Functions
////////////////////////////////////////////////////////////////////////////////////////////////////
void DaliCommands::set_rgb_2(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim) {
    
    // Set RGB
    //printf("set_rgb_2   R:%d G:%d B:%d\n", r, g, b);
    send_command_special(SET_DTR0, r);
    send_command_special(SET_DTR1, g);
    send_command_special(SET_DTR2, b);
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_RGB_DIM, addr);
    
    
}

void DaliCommands::set_rgb_3(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim) {
    
    // Set RGB
    send_command_special(SET_DTR0, r);
    send_command_special(SET_DTR1, g);
    send_command_special(SET_DTR2, b);
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_RGB_DIM, addr);
   
    send_command_special(SET_DTR0, 0);
    send_command_special(SET_DTR1, 0);
    send_command_special(SET_DTR2, 0);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_WAF_DIM, addr);
    
}

void DaliCommands::set_rgb_32(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim) {
    
    // Set RGB using 32-bit command
    send_command_special32(SET_DTR0, r, SET_DTR1, g);
    send_command_special32(SET_DTR2, b, ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_RGB_DIM, addr);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_off_waf_channels(uint8_t addr) {
    
    // Set dim to 0 for WAF channels
    send_command_special(SET_DTR0, 0);  // w
    send_command_special(SET_DTR1, 0);  // a
    send_command_special(SET_DTR2, 0);  // f
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_WAF_DIM, addr);

    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_rgb_WAF(uint8_t addr, uint8_t white_dim) {
    
    // Set RGB to 0
    send_command_special(SET_DTR0, 0);
    send_command_special(SET_DTR1, 0);
    send_command_special(SET_DTR2, 0);
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_RGB_DIM, addr);
   
    send_command_special(SET_DTR0, white_dim);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_WAF_DIM, addr);
    
}

void DaliCommands::set_primary_dim_level(uint8_t addr, uint16_t white_dim) {
    
    uint8_t dtr0 = (uint8_t)(white_dim & 0x00FF);
    uint8_t dtr1 = (uint8_t)((white_dim >> 8) & 0x00FF);
   
    send_command_special(SET_DTR0, dtr0);
    send_command_special(SET_DTR1, dtr1);
    send_command_special(SET_DTR2, 0);
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_PRIMARY_DIM, addr);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_cct_channel_4_5_dim(uint8_t addr, uint8_t cool_dim, uint8_t warm_dim) {
    
    send_command_special(SET_DTR0, cool_dim);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_TEMPC_COOLER, addr);
   
    send_command_special(SET_DTR0, warm_dim);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_TEMPC_WARMER, addr);
   
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_rgbwaf_ctrl() {
    static uint8_t _data = 0b00111111;  // Setting channels
    
    send_command_special(SET_DTR0, _data);
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(SET_TEMP_RGB_WAF_DIM, 0);
    
}

void DaliCommands::set_cct_dimming(uint8_t dim) {
    
    send_command_special(SET_DTR0, dim);
    send_command_standard(SET_TEMP_PRIMARY_DIM, 2);
    send_command_standard(SET_TEMP_PRIMARY_DIM, 2);
    
}

void DaliCommands::set_color_rgb(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim) {
    
    set_rgb_3(addr, r, g, b, dim);
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
}

void DaliCommands::set_color_rgb_2(uint8_t addr, uint8_t r, uint8_t g, uint8_t b, uint8_t dim) {
    
    set_rgb_3(addr, r, g, b, dim);
   
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_color_rgb_WAF(uint8_t addr, uint8_t dim) {
    
    set_rgb_WAF(addr, dim);
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_color_cct_waf_dim(uint8_t addr, uint8_t dim) {
    
    set_cct_channel_4_5_dim(addr, dim, dim);
    // Enable device type 8
    send_command_special(ENABLE_DEVICE_TYPE, 0x08);
    send_command_standard(COLOR_ACTIVATE, addr);
    
}

void DaliCommands::set_mode(uint8_t addr, uint8_t mode){
    
    daliCore.sendCommandPublic(SET_DTR0, mode);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, 0x23);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, 0x23);
    
}

/**********Set Dali Device Power On Level value***********/
void DaliCommands::set_power_on_level(uint8_t addr, uint8_t power_on_level){
    
    daliCore.sendCommandPublic(SET_DTR0, power_on_level);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_POWER_ON_LEVEL);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_POWER_ON_LEVEL);
    
}

/**********Set Dali Device Min Level value***********/
void DaliCommands::set_min_level(uint8_t addr, uint8_t value){
    
    daliCore.sendCommandPublic(DALI_CMD_STORE_DTR, value);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, DALI_CMD_STORE_DTR_MIN_LEVEL);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, DALI_CMD_STORE_DTR_MIN_LEVEL);
    
}

/**********Set DALI Device Max Level value***********/
void DaliCommands::set_max_level(uint8_t addr, uint8_t value){
    
    daliCore.sendCommandPublic(DALI_CMD_STORE_DTR, value);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, DALI_CMD_STORE_DTR_MAX_LEVEL);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, DALI_CMD_STORE_DTR_MAX_LEVEL);
    
}

/**********Set DALI Device fade time value***********/
void DaliCommands::set_ext_fade_time(uint8_t addr, uint8_t time)
{
    
    // Send twice command
    daliCore.sendCommandPublic(SET_DTR0, time);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_EXT_FADE_TIME);
    //delay(DELAY_COMMAND_SEND);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_EXT_FADE_TIME);
      
}

void DaliCommands::set_fade_time(uint8_t addr, uint8_t time)
{
    
    daliCore.sendCommandPublic(SET_DTR0, time);
    vTaskDelay(DELAY_COMMAND_SEND / portTICK_PERIOD_MS);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_FADE_TIME);
    send_command_standard(SET_FADE_TIME, addr); 
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_FADE_TIME);
    send_command_standard(SET_FADE_TIME, addr);
    
}

/**********Set DALI Device fade rate value***********/
void DaliCommands::set_fade_rate(uint8_t addr, uint8_t rate)
{
    
    // Send twice command
    daliCore.sendCommandPublic(SET_DTR0, rate);
    vTaskDelay(DELAY_COMMAND_SEND / portTICK_PERIOD_MS);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_FADE_RATE);
    vTaskDelay(DELAY_COMMAND_SEND / portTICK_PERIOD_MS);
    daliCore.sendCommandPublic(SHORT_POWER | ((addr << 1) & 0x7e) | 0x01, SET_FADE_RATE);
    send_command_standard(SET_FADE_RATE, addr);
    
}

#define BROADCAST_DP 0b11111110
void DaliCommands::set_broadcast_fade_time(uint8_t time)
{
    
    // Send twice command
    daliCore.sendCommandPublic(SET_DTR0, time);
    send_command_standard(SET_FADE_TIME, BROADCAST_DP);
    send_command_standard(SET_FADE_TIME, BROADCAST_DP);     
         
}

/**********Set DALI Device fade rate value***********/
void DaliCommands::set_broadcast_fade_rate( uint8_t rate)
{
    
    // Send twice command
    daliCore.sendCommandPublic(SET_DTR0, rate);
    send_command_standard(SET_FADE_RATE, BROADCAST_DP);
    send_command_standard(SET_FADE_RATE, BROADCAST_DP);
    
}

 

///////////////////////////Dali group Functions////////////////////////
bool DaliCommands::add_to_group(uint8_t addr, uint8_t group)
{
    
    // Send the command to add to group
    send_command_standard(ADD_TO_GROUP + group, addr);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    send_command_standard(ADD_TO_GROUP + group, addr);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    return true;
}

void DaliCommands::remove_from_group(uint8_t addr, uint8_t group)
{
    
    // Send the command to remove from group
    send_command_standard(REMOVE_FROM_GROUP + group, addr);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    send_command_standard(REMOVE_FROM_GROUP + group, addr);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
}

void DaliCommands::set_group_fade_time(uint8_t addr, uint8_t time)
{
    
    daliCore.sendCommandPublic(SET_DTR0, time);
    send_command_standard(SET_FADE_TIME, addr);
    send_command_standard(SET_FADE_TIME, addr);
    
}

void DaliCommands::set_group_fade_rate(uint8_t addr, uint8_t rate)
{
    
    daliCore.sendCommandPublic(SET_DTR0, rate);
    send_command_standard(SET_FADE_RATE, addr);
    send_command_standard(SET_FADE_RATE, addr);
    
}



void DaliCommands::set_group_power_on_level(uint8_t addr, uint8_t power_on_level){
    
    daliCore.sendCommandPublic(SET_DTR0, power_on_level);
    send_command_standard(SET_POWER_ON_LEVEL, addr);
    send_command_standard(SET_POWER_ON_LEVEL, addr);
    
}

void DaliCommands::set_group_off(uint8_t group_id)
{
    //daliCore.sendCommandPublic(0x80 | (group_id << 1), 0x00);
    send_command_standard(OFF, group_id | (1<<7));
    group_state = 0;
}

void DaliCommands::set_group_on(uint8_t group_id)
{
    send_command_standard(GO_TO_LAST_ACTIVE_LEVEL, group_id | (1<<7));
    group_state = 1;
}


uint8_t DaliCommands::get_group_addr(uint8_t group_id)
{
    uint8_t mask = 1 << 7;
    // Make MSb a 1 to signify >1 device being addressed
    return mask | (group_id<<1);
}

void DaliCommands::set_group_level(uint8_t group_id, uint8_t value){
    daliCore.sendCommandPublic(0x80 | (group_id << 1), value);
}

void DaliCommands::set_group_color_cct(uint8_t group_id, uint16_t color_temp_kelvin){
    uint8_t group_addr = group_id | (1<<7);
    set_color_temperature(group_addr, color_temp_kelvin);  
}

void DaliCommands::set_group_color_rgb(uint8_t group_id, uint8_t r, uint8_t g, uint8_t b, uint8_t dim){
    
    uint8_t group_addr = group_id | (1<<7);
    if((r==0) && (g==0) && (b==0)) {
        if(group_state) set_group_off(group_id);
    }else {
        set_color_rgb(group_addr, r, g, b, dim);
        if(!group_state) {
            set_group_on(group_id);
        }        
    }
    
}

void DaliCommands::set_broadcast_level(uint8_t value){
    
    daliCore.sendCommandPublic(0x80 | (0xff << 1), value);
    
}

void DaliCommands::set_broadcast_color_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t dim){
   
    
    if((r==0) & (g==0) & (b==0)) {
        send_broadcast(OFF_C);
        group_state = false;
    }else {
        set_color_rgb(0xff, r, g, b, dim);
        if(!group_state) {
            group_state = true;
            send_broadcast(ON_C);
        }        
    }
    
}

void DaliCommands::set_broadcast_color_rgb_2(uint8_t r, uint8_t g, uint8_t b, uint8_t dim){
 
    
    if((r==0) & (g==0) & (b==0)) {
        send_broadcast(OFF_C);
        group_state = false;
    }else {
        if(!group_state) {
            group_state = true;
            send_broadcast(ON_C);
        }
        set_color_rgb_2(0xff, r, g, b, dim);
    }
    
} 

void DaliCommands::set_broadcast_color_cct(uint16_t color_temp_kelvin){
    
    set_color_temperature(0xff, color_temp_kelvin);
    
}
///////////////////////////Dali Scene Functions////////////////////////
void DaliCommands::set_scene(uint8_t addr, uint8_t scene, uint8_t level)
{
    
    daliCore.sendCommandPublic(SET_DTR0, level);
    send_command_standard(SET_SCENE + scene, addr);
    send_command_standard(SET_SCENE + scene, addr);
    
}

void DaliCommands::set_level_scene(uint8_t addr, uint8_t scene , uint8_t scene_level)
{
    set_dim_value(addr, scene_level);
    // Store what is in the temperorary color as scene color and also scene level to DTR0
    daliCore.sendCommandPublic(SET_DTR0, scene_level);
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    
}

void DaliCommands::set_color_scene(uint8_t addr, uint8_t scene, uint8_t scene_level , uint16_t temp)
{
    set_color_temp(addr, temp);    
    daliCore.sendCommandPublic(SET_DTR0, scene_level);
    // Store what is in the temperorary color as scene color and also scene level to DTR0
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    
}
void DaliCommands::set_rgb_scene(uint8_t addr, uint8_t scene, uint8_t scene_level , uint8_t r, uint8_t g, uint8_t b)
{   
    set_rgb_2(addr, r, g, b, 255);  
    daliCore.sendCommandPublic(SET_DTR0, scene_level);
    // Store what is in the temperorary color as scene color and also scene level to DTR0
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    send_command_standard(STORE_DTR_AS_SCENE + scene, addr);
    
}
void DaliCommands::add_to_scene(uint8_t addr, uint8_t scene)
{
    
    send_command_standard(SET_SCENE + scene, addr);
    send_command_standard(SET_SCENE + scene, addr);
    
}

void DaliCommands::remove_from_scene(uint8_t addr, uint8_t scene)
{
    
    send_command_standard(REMOVE_FROM_SCENE + scene, addr);
    send_command_standard(REMOVE_FROM_SCENE + scene, addr);
    
}

void DaliCommands::go_to_scene(uint8_t addr, uint8_t scene)
{
    
    send_command_standard(GO_TO_SCENE + scene, addr);
    //vTaskDelay(DELAY_COMMAND_SEND / portTICK_PERIOD_MS);
    //send_command_standard(GO_TO_SCENE + scene, addr);
    
}

void DaliCommands::go_to_group_scene(uint8_t group_id, uint8_t scene)
{
    
    send_command_standard(GO_TO_SCENE + scene, group_id | (1<<7));
    //send_command_standard(GO_TO_SCENE + scene, group_id | (1<<7));
    
}

void DaliCommands:: send_broadcast(uint8_t status){
    
    daliCore.sendCommandPublic(BROADCAST_C, status);
    
}

int DaliCommands::initNodes(const uint8_t* addresses, uint8_t numAddresses)
{
return daliCore.initNodes(addresses, numAddresses);
}

int DaliCommands::scanAssignedShortAddresses(uint8_t* foundAddresses, uint8_t maxAddresses)
{
return daliCore.scanAssignedShortAddresses(foundAddresses, maxAddresses);
}


void DaliCommands::disableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) {
        gpio_intr_disable(rxPin);
    }
}

void DaliCommands::enableRxInterrupt() {
    if (rxPin != GPIO_NUM_NC) {
        gpio_intr_enable(rxPin);
    }
}

void DaliCommands::enable_query_mode() {
    receiver.set_query_mode(true);
}

void DaliCommands::disable_query_mode() {
    receiver.set_query_mode(false);
}

void DaliCommands::query(uint8_t shortAddress, uint8_t queryCommand) {
    
    daliCore.query(shortAddress, queryCommand);
    // disableRxInterrupt();
    // esp_rom_delay_us(2400); 
    //   // Set to 8-bit mode
    // enableRxInterrupt(); 
    // vTaskDelay(50 / portTICK_PERIOD_MS);
    // receiver.set_query_mode(false);  // Set to 16-bit mode
    
}



