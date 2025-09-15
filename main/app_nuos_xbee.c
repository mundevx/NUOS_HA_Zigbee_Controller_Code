
#include "esp_zigbee_core.h"
#include "zdo/esp_zigbee_zdo_command.h"
// In your Zigbee application code:
#define DIGI_XBEE_IO_CLUSTER_ID 0xE001

// Target XBee's address (replace with actual XBee's IEEE address)
//esp_zb_ieee_addr_t xbee_address = {0x00, 0x13, 0xA2, 0x00, 0x42, 0x1A, 0xCC, 0x34}; 
esp_zb_ieee_addr_t xbee_address = {0x34, 0xCC, 0x1A, 0x42, 0x00, 0xA2, 0x13, 0x00};
// Create a ZCL command payload (e.g., pin D0 = 0, value = 1 for ON)
// uint8_t payload[] = {0x00, 0x01}; 

// // Send the command
// esp_zb_zcl_cmd_custom_cluster_req(
//     xbee_address,
//     DIGI_XBEE_IO_CLUSTER_ID,
//     0xE8,  // XBee endpoint
//     0x01,  // Command ID (check Digi docs)
//     payload,
//     sizeof(payload),
//     0  // Disable default response
// );

// Send "ON" to XBee’s D0 pin
void send_on_to_xbee() {
    uint8_t payload[] = {0x00, 0x01}; // D0 = HIGH

	esp_zb_zcl_custom_cluster_cmd_req_t cmd_req = {
		.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		.cluster_id = DIGI_XBEE_IO_CLUSTER_ID,
		.custom_cmd_id = 0x01,
		.data.value = payload,
		.data.size = 2,
		.data.type = ESP_ZB_ZCL_ATTR_TYPE_ARRAY,
		.direction = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
		.profile_id = 0xC105,
		.zcl_basic_cmd.dst_endpoint = 0xE8,
		.zcl_basic_cmd.src_endpoint = 0,
	};
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[0] = xbee_address[0];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[1] = xbee_address[1];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[2] = xbee_address[2];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[3] = xbee_address[3];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[4] = xbee_address[4];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[5] = xbee_address[5];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[6] = xbee_address[6];
	cmd_req.zcl_basic_cmd.dst_addr_u.addr_long[7] = xbee_address[7];

	esp_zb_zcl_custom_cluster_cmd_req(&cmd_req);
		
}