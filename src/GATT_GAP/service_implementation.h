// September 21, 2024
// Pico W project to create a build with FreeRTOS and BTstack (Bluetooth LE)
// Testbed to start building a FreeRTOS project with BLE.

// Code based on project by:
// V. Hunter Adams (vha3@cornell.edu)
// Custom GATT Service implementation
// Modeled off examples from BTstack

// Replaced Cornell Protothreads with FreeRTOS tasks.


#include "btstack_defines.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "btstack_util.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"

#include "server_gattfile.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include "defs.h"
#include "ftos_ble_task.h"


// B_Bot Custom Service as defined in ../ab_ftos_ble/src/GATT_Service/server_fattfile.gatt
// Characteristic user descriptions (appear in LightBlue app)
// characteristic_a = Read-only Heartbeat Counter
// characteristic_b = Command to Pico, BLE input command
// characteristic_c = LED Status and Control
// characteristic_d = Read-only Pico temperature
// characteristic_e = Text to Pico, BLE input test message

// Characteristic user descriptions (appear in LightBlue app)
char characteristic_a[] = "Read-only Counter" ;
char characteristic_b[] = "Command to Pico" ;         // BLE input command
char characteristic_c[] = "LED Status and Control" ;
char characteristic_d[] = "Pico Temperature" ;
char characteristic_e[] = "Text to B-Bot" ;           // BLE input text message

// Create a struct for managing this service
typedef struct {

	// Connection handle for service
	hci_con_handle_t con_handle ;

	// Characteristic A information; Counter
	char * 		characteristic_a_value ;
	uint16_t 	characteristic_a_client_configuration ;
	char * 		characteristic_a_user_description ;

	// Characteristic B information; Pico Command
	char *  	characteristic_b_value ;                 // commands from client (phone) placed here
	uint16_t  	characteristic_b_client_configuration ;
	char *  	characteristic_b_user_description ;

	// Characteristic C information; Pico LED Control
	char *  	characteristic_c_value ;
	char *  	characteristic_c_user_description ;
	
	// Characteristic D information; Pico Temperature
	char * 		characteristic_d_value ;
	uint16_t 	characteristic_d_client_configuration ;
	char * 		characteristic_d_user_description ;
	
	// Characteristic E information; B-Bot Text
	char *  	characteristic_e_value ;                 // text from client (phone) placed here
	uint16_t  	characteristic_e_client_configuration ;
	char *  	characteristic_e_user_description ;

	// ********* Characteristic Handles *************
	// Characteristic A handles
	uint16_t  	characteristic_a_handle ;
	uint16_t 	characteristic_a_client_configuration_handle ;
	uint16_t 	characteristic_a_user_description_handle ;

	// Characteristic B handles
	uint16_t  	characteristic_b_handle ;
	uint16_t 	characteristic_b_client_configuration_handle ;
	uint16_t 	characteristic_b_user_description_handle ;

	// Characteristic C handles
	uint16_t  	characteristic_c_handle ;
	uint16_t 	characteristic_c_user_description_handle ;
	
	// Characteristic D handles
	uint16_t  	characteristic_d_handle ;
	uint16_t 	characteristic_d_client_configuration_handle ;
	uint16_t 	characteristic_d_user_description_handle ;

	// Characteristic B handles
	uint16_t  	characteristic_e_handle ;
	uint16_t 	characteristic_e_client_configuration_handle ;
	uint16_t 	characteristic_e_user_description_handle ;

	// ********* Callback functions *************
	btstack_context_callback_registration_t callback_a ;
	btstack_context_callback_registration_t callback_b ;
	btstack_context_callback_registration_t callback_d ;
	btstack_context_callback_registration_t callback_e ;

} custom_service_t ;

// Create a callback registration object, and an att service handler object
static att_service_handler_t 	service_handler ;          // BTstack defined structure
static custom_service_t 		service_object ;           // locally defined structure


// Callback functions for ATT notifications on characteristics
static void characteristic_a_callback(void * context){
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context ;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_a_handle, instance->characteristic_a_value, strlen(instance->characteristic_a_value)) ;
}

static void characteristic_b_callback(void * context) {
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context ;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_b_handle, instance->characteristic_b_value, strlen(instance->characteristic_b_value));
}

static void characteristic_d_callback(void * context){
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context ;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_d_handle, instance->characteristic_d_value, strlen(instance->characteristic_d_value)) ;
}

static void characteristic_e_callback(void * context) {
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context ;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_e_handle, instance->characteristic_e_value, strlen(instance->characteristic_e_value));
}


// Read callback (no client configuration handles on characteristics without Notify)
static uint16_t custom_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
	UNUSED(con_handle);

	// Characteristic A
	if (attribute_handle == service_object.characteristic_a_handle){
		return att_read_callback_handle_blob(service_object.characteristic_a_value, strlen(service_object.characteristic_a_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_a_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_a_user_description, strlen(service_object.characteristic_a_user_description), offset, buffer, buffer_size);
	}

	// Characteristic B
	if (attribute_handle == service_object.characteristic_b_handle){
		return att_read_callback_handle_blob(service_object.characteristic_b_value, strlen(service_object.characteristic_b_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_b_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_b_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_b_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_b_user_description, strlen(service_object.characteristic_b_user_description), offset, buffer, buffer_size);
	}

	// Characteristic C
	if (attribute_handle == service_object.characteristic_c_handle){
		return att_read_callback_handle_blob(service_object.characteristic_c_value, strlen(service_object.characteristic_c_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_c_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_c_user_description, strlen(service_object.characteristic_c_user_description), offset, buffer, buffer_size);
	}
	
	// Characteristic D
	if (attribute_handle == service_object.characteristic_d_handle){
		return att_read_callback_handle_blob(service_object.characteristic_d_value, strlen(service_object.characteristic_d_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_d_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_d_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_d_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_d_user_description, strlen(service_object.characteristic_d_user_description), offset, buffer, buffer_size);
	}
	
	// Characteristic E
	if (attribute_handle == service_object.characteristic_e_handle){
		return att_read_callback_handle_blob(service_object.characteristic_e_value, strlen(service_object.characteristic_e_value), offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_e_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_e_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_e_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_e_user_description, strlen(service_object.characteristic_e_user_description), offset, buffer, buffer_size);
	}

	return 0;
}

// Write callback
static int custom_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
	UNUSED(transaction_mode);
	UNUSED(offset);
	UNUSED(buffer_size);

	// Characteristic A
	// Enable/disable notifications
	if (attribute_handle == service_object.characteristic_a_client_configuration_handle){
		service_object.characteristic_a_client_configuration = little_endian_read_16(buffer, 0);
		service_object.con_handle = con_handle;
	}

	// Characteristic B
	// Enable/disable notificatins
	if (attribute_handle == service_object.characteristic_b_client_configuration_handle){
		service_object.characteristic_b_client_configuration = little_endian_read_16(buffer, 0);
		service_object.con_handle = con_handle;
	}

	// Write Command to Pico characteristic value
	if (attribute_handle == service_object.characteristic_b_handle) {
		custom_service_t * instance = &service_object ;
		buffer[buffer_size] = 0 ;
		//
		memset(service_object.characteristic_b_value, 0, sizeof(service_object.characteristic_b_value)) ;
		memcpy(service_object.characteristic_b_value, buffer, buffer_size) ;
		// Null-terminate the string
		service_object.characteristic_b_value[buffer_size] = 0 ;
		// clear ble_input buffer
		memset(blecmdtxt_ptr->ble_input, 0, sizeof(blecmdtxt_ptr->ble_input)) ;  // fill a block of memory with a specific value
		memcpy(blecmdtxt_ptr->ble_input, buffer, buffer_size) ;   // copy to buffer in defs.h, used in ftos_tasks.h
		uart_puts(UART_ID, "Command from client:\n\r");
		    
        // **************
        // Call to FreeRTOS task!!!!
        // **************
        // Send a notification to the handler task vBLEinput_HandlerTask() in main.c
        // Get xBLEinput_HandlerTask from main.c
        xTaskNotifyGive( get_xBLEinput_HandlerTask());      // Call to FreeRTOS task!!!!
		    
		// If client has enabled notifications, register a callback
		if (instance->characteristic_b_client_configuration) {
		    instance->callback_b.callback = &characteristic_b_callback ;
			instance->callback_b.context = (void*) instance ;
			att_server_register_can_send_now_callback(&instance->callback_b, instance->con_handle) ;
		}
	}

	// Characteristic C
	// Write LED status and control characteristic value
	if (attribute_handle == service_object.characteristic_c_handle) {
		custom_service_t * instance = &service_object ;
		buffer[buffer_size] = 0 ;
		if (!strcmp(buffer, "off")) {
			memset(service_object.characteristic_c_value, 0, sizeof(service_object.characteristic_c_value)) ;
			memcpy(service_object.characteristic_c_value, buffer, buffer_size) ;
			service_object.characteristic_c_value[buffer_size] = 0 ;
			gpio_put(LED_EX1, 0);
		}
		else if (!strcmp(buffer, "on")) {
			memset(service_object.characteristic_c_value, 0, sizeof(service_object.characteristic_c_value)) ;
			memcpy(service_object.characteristic_c_value, buffer, buffer_size) ;
			service_object.characteristic_c_value[buffer_size] = 0 ;
			gpio_put(LED_EX1, 1);       
		}
	}
	
	// Characteristic E
	// Enable/disable notificatins
	if (attribute_handle == service_object.characteristic_e_client_configuration_handle){
		service_object.characteristic_e_client_configuration = little_endian_read_16(buffer, 0);
		service_object.con_handle = con_handle;
	}

	// Write Text to Pico characteristic value
	if (attribute_handle == service_object.characteristic_e_handle) {
		custom_service_t * instance = &service_object ;
		buffer[buffer_size] = 0 ;
		//
		memset(service_object.characteristic_e_value, 0, sizeof(service_object.characteristic_e_value)) ;
		memcpy(service_object.characteristic_e_value, buffer, buffer_size) ;
		// Null-terminate the string
		service_object.characteristic_e_value[buffer_size] = 0 ;
/*
		uart_puts(UART_ID, "Charactertic_e text: ");
		uart_puts(UART_ID, service_object.characteristic_e_value);
		uart_puts(UART_ID, "\n\r");
*/
		// clear ble_input buffer to force xBLEinput_HandlerTask() to interpret this as a text message
		memset(blecmdtxt_ptr->ble_input, 0, sizeof(blecmdtxt_ptr->ble_input)) ;  // fill a block of memory with a specific value
		memcpy(blecmdtxt_ptr->ble_input, "bletxt", 6);            // set message flag used in ftos_tasks.h
		// clear client_message buffer used in ftos_tasks.h
		memset(blecmdtxt_ptr->client_message, 0, sizeof(blecmdtxt_ptr->client_message)) ;  // fill a block of memory with a specific value
		memcpy(blecmdtxt_ptr->client_message, buffer, buffer_size) ;   // copy to buffer in defs.h, used in ftos_tasks.h
/*
		uart_puts(UART_ID, "Text from client: ");
		uart_puts(UART_ID, blecmdtxt_ptr->client_message);
		uart_puts(UART_ID, "\n\r");
*/		    
        // **************
        // Call to FreeRTOS task!!!!
        // **************
        // Send a notification to the handler task vBLEinput_HandlerTask() in main.c
        // ble_input[] buffer should contain "bletxt" and client_message[] buffer should contain a message to display
        // Get xBLEinput_HandlerTask from main.c
        xTaskNotifyGive( get_xBLEinput_HandlerTask());      // Call to FreeRTOS task!!!!
		    
		// If client has enabled notifications, register a callback
		if (instance->characteristic_e_client_configuration) {
		    instance->callback_e.callback = &characteristic_e_callback ;
			instance->callback_e.context = (void*) instance ;
			att_server_register_can_send_now_callback(&instance->callback_e, instance->con_handle) ;
		}
	}	
	
	
	return 0;
}


/////////////////////////////////////////////////////////////////////////////
////////////////////////////// USER API /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Initialize our custom service handler
void custom_service_server_init(char * a_ptr, char * b_ptr, char * c_ptr, char * d_ptr, char * e_ptr){

	// Pointer to our service object
	custom_service_t * instance = &service_object ;

	// Assign characteristic value
	instance->characteristic_a_value = a_ptr ;
	instance->characteristic_b_value = b_ptr ;
	instance->characteristic_c_value = c_ptr ;
	instance->characteristic_d_value = d_ptr ;
	instance->characteristic_e_value = e_ptr ;

	// Assign characteristic user description
	instance->characteristic_a_user_description = characteristic_a ;
	instance->characteristic_b_user_description = characteristic_b ;
	instance->characteristic_c_user_description = characteristic_c ;
	instance->characteristic_d_user_description = characteristic_d ;
	instance->characteristic_e_user_description = characteristic_e ;


    // Read Heartbeat Counter Characteristic
	// Assign handle values (from generated gatt header)
	// handle names found at:/home/arbz/pico/ab_ftos_ble/build/src/generated/server_gattfile.h
	// Counter
	instance->characteristic_a_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE ;
	instance->characteristic_a_client_configuration_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE ;
	instance->characteristic_a_user_description_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE ;
    // Command to Pico Characteristic
	instance->characteristic_b_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE ;
	instance->characteristic_b_client_configuration_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE ;
	instance->characteristic_b_user_description_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE ;
    // LED status and controll characteristic
	instance->characteristic_c_handle = ATT_CHARACTERISTIC_0000FF13_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE ;
	instance->characteristic_c_user_description_handle = ATT_CHARACTERISTIC_0000FF13_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE ;
	// Pico Temperature
	instance->characteristic_d_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE ;
	instance->characteristic_d_client_configuration_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE ;
	instance->characteristic_d_user_description_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE ;
    // Text to Pico Characteristic
	instance->characteristic_e_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE ;
	instance->characteristic_e_client_configuration_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE ;
	instance->characteristic_e_user_description_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE ;

	// Service start and end handles (modeled off heartrate example)
	service_handler.start_handle = 0 ;
	service_handler.end_handle = 0xFFFF ;
	service_handler.read_callback = &custom_service_read_callback ;
	service_handler.write_callback = &custom_service_write_callback ;

	// Register the service handler
	att_server_register_service_handler(&service_handler);       // BTstack defined function
}

// Update Characteristic A value; called from main.c
void set_characteristic_a_value(int value){

	// Pointer to our service object
	custom_service_t * instance = &service_object ;

	// Update field value
	sprintf(instance->characteristic_a_value, "%d", value) ;

	// Are notifications enabled? If so, register a callback
	if (instance->characteristic_a_client_configuration){
		instance->callback_a.callback = &characteristic_a_callback;
		instance->callback_a.context  = (void*) instance;
		att_server_register_can_send_now_callback(&instance->callback_a, instance->con_handle);;
	}
}

// Update Characteristic D value; called from main.c
void set_characteristic_d_value(int value){

	// Pointer to our service object
	custom_service_t * instance = &service_object ;

	// Update field value
	sprintf(instance->characteristic_d_value, "%d", value) ;

	// Are notifications enabled? If so, register a callback
	if (instance->characteristic_d_client_configuration){
		instance->callback_d.callback = &characteristic_d_callback;
		instance->callback_d.context  = (void*) instance;
		att_server_register_can_send_now_callback(&instance->callback_d, instance->con_handle);;
	}
}
