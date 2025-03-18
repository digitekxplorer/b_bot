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

// Define maximum sizes for characteristic values (adjust as needed)
#define LED_STATUS_BUFFER_SIZE 16
#define MAX_A_VALUE_SIZE 16 // Example: Enough for a 32-bit integer + sign + null
#define MAX_B_VALUE_SIZE 64 // Example: Adjust based on expected command length
#define MAX_D_VALUE_SIZE 16 // Example: Enough for a 32-bit integer + sign + null
#define MAX_E_VALUE_SIZE 128 // Example: Adjust based on expected message length

#define BLECMD_BUFFER_SIZE 64
#define CLIENT_MESSAGE_BUFFER_SIZE 128


// B_Bot Custom Service as defined in ../ab_ftos_ble/src/GATT_Service/server_fattfile.gatt
// Characteristic user descriptions (appear in LightBlue app)
// characteristic_a = Read-only Heartbeat Counter
// characteristic_b = Command to Pico, BLE input command
// characteristic_c = LED Status and Control
// characteristic_d = Read-only Pico temperature
// characteristic_e = Text to Pico, BLE input test message

// Characteristic user descriptions (appear in LightBlue app)
char characteristic_a[] = "Pico Heartbeat Counter";
char characteristic_b[] = "Command to Pico";         // BLE input command
char characteristic_c[] = "LED Status and Control";
char characteristic_d[] = "Pico Temperature";
char characteristic_e[] = "Text to Pico";           // BLE input text message

// Create a struct for managing this service
typedef struct {

	// Connection handle for service
	hci_con_handle_t con_handle;

	// Characteristic A information; Counter
	char 		characteristic_a_value[MAX_A_VALUE_SIZE];  // Allocate directly
	uint16_t 	characteristic_a_client_configuration;
	char * 		characteristic_a_user_description;

	// Characteristic B information; Pico Command
	char  	    characteristic_b_value[MAX_B_VALUE_SIZE];  // Allocate directly
	uint16_t  	characteristic_b_client_configuration;
	char *  	characteristic_b_user_description;

	// Characteristic C information; Pico LED Control
    char        characteristic_c_value[LED_STATUS_BUFFER_SIZE]; // Allocate directly
    uint16_t    characteristic_c_client_configuration;
    char *      characteristic_c_user_description;

	// Characteristic D information; Pico Temperature
	char 		characteristic_d_value[MAX_D_VALUE_SIZE]; // Allocate directly
	uint16_t 	characteristic_d_client_configuration;
	char * 		characteristic_d_user_description;

	// Characteristic E information; B-Bot Text
	char  	    characteristic_e_value[MAX_E_VALUE_SIZE];  // Allocate directly
	uint16_t  	characteristic_e_client_configuration;
	char *  	characteristic_e_user_description;

	// ********* Characteristic Handles *************
	// Characteristic A handles
	uint16_t  	characteristic_a_handle;
	uint16_t 	characteristic_a_client_configuration_handle;
	uint16_t 	characteristic_a_user_description_handle;

	// Characteristic B handles
	uint16_t  	characteristic_b_handle;
	uint16_t 	characteristic_b_client_configuration_handle;
	uint16_t 	characteristic_b_user_description_handle;

	// Characteristic C handles
	uint16_t  	characteristic_c_handle;
    uint16_t 	characteristic_c_client_configuration_handle;
	uint16_t 	characteristic_c_user_description_handle;

	// Characteristic D handles
	uint16_t  	characteristic_d_handle;
	uint16_t 	characteristic_d_client_configuration_handle;
	uint16_t 	characteristic_d_user_description_handle;

	// Characteristic B handles
	uint16_t  	characteristic_e_handle;
	uint16_t 	characteristic_e_client_configuration_handle;
	uint16_t 	characteristic_e_user_description_handle;

	// ********* Callback functions *************
	btstack_context_callback_registration_t callback_a;
	btstack_context_callback_registration_t callback_b;
    btstack_context_callback_registration_t callback_c;
	btstack_context_callback_registration_t callback_d;
	btstack_context_callback_registration_t callback_e;

} custom_service_t;

// Create a callback registration object, and an att service handler object
static att_service_handler_t 	service_handler;          // BTstack defined structure
static custom_service_t 		service_object;           // locally defined structure


// Callback functions for ATT notifications on characteristics
static void characteristic_a_callback(void * context){
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_a_handle, 
	                  instance->characteristic_a_value, 
					  strnlen(instance->characteristic_a_value, MAX_A_VALUE_SIZE));
}

static void characteristic_b_callback(void * context) {
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context;
	// Send a notification
    att_server_notify(instance->con_handle, instance->characteristic_b_handle, 
	                  instance->characteristic_b_value, 
					  strnlen(instance->characteristic_b_value, MAX_B_VALUE_SIZE));
}

static void characteristic_c_callback(void * context) {

    custom_service_t * instance = (custom_service_t *) context;
    // Send a notification.  Use strlen, but ensure it doesn't exceed buffer size.
    att_server_notify(instance->con_handle, instance->characteristic_c_handle,
                      instance->characteristic_c_value,
                      strnlen(instance->characteristic_c_value, LED_STATUS_BUFFER_SIZE));
}



static void characteristic_d_callback(void * context){
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_d_handle, 
	                  instance->characteristic_d_value, 
					  strnlen(instance->characteristic_d_value, MAX_D_VALUE_SIZE));
}

static void characteristic_e_callback(void * context) {
	// Associate the void pointer input with our custom service object
	custom_service_t * instance = (custom_service_t *) context;
	// Send a notification
	att_server_notify(instance->con_handle, instance->characteristic_e_handle, 
	                  instance->characteristic_e_value, 
					  strnlen(instance->characteristic_e_value, MAX_E_VALUE_SIZE));
}


// Read callback (no client configuration handles on characteristics without Notify)
static uint16_t custom_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
	UNUSED(con_handle);

	// Characteristic A
	if (attribute_handle == service_object.characteristic_a_handle){
		return att_read_callback_handle_blob(service_object.characteristic_a_value, 
		                                     strnlen(service_object.characteristic_a_value, MAX_A_VALUE_SIZE), 
											 offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_a_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_a_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_a_user_description, strlen(service_object.characteristic_a_user_description), offset, buffer, buffer_size);
	}

	// Characteristic B
	if (attribute_handle == service_object.characteristic_b_handle){
		return att_read_callback_handle_blob(service_object.characteristic_b_value, 
		                                     strnlen(service_object.characteristic_b_value, MAX_B_VALUE_SIZE), 
											 offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_b_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_b_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_b_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_b_user_description, strlen(service_object.characteristic_b_user_description), offset, buffer, buffer_size);
	}

	// Characteristic C
    if (attribute_handle == service_object.characteristic_c_handle) {
        // Use strnlen to prevent reading beyond the allocated buffer.
        return att_read_callback_handle_blob(service_object.characteristic_c_value,
                                             strnlen(service_object.characteristic_c_value, LED_STATUS_BUFFER_SIZE),
                                             offset, buffer, buffer_size);
    }
	if (attribute_handle == service_object.characteristic_c_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_c_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_c_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_c_user_description, strlen(service_object.characteristic_c_user_description), offset, buffer, buffer_size);
	}

	// Characteristic D
	if (attribute_handle == service_object.characteristic_d_handle){
		return att_read_callback_handle_blob(service_object.characteristic_d_value, 
		                                     strnlen(service_object.characteristic_d_value, MAX_D_VALUE_SIZE), 
											 offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_d_client_configuration_handle){
		return att_read_callback_handle_little_endian_16(service_object.characteristic_d_client_configuration, offset, buffer, buffer_size);
	}
	if (attribute_handle == service_object.characteristic_d_user_description_handle) {
		return att_read_callback_handle_blob(service_object.characteristic_d_user_description, strlen(service_object.characteristic_d_user_description), offset, buffer, buffer_size);
	}

	// Characteristic E
	if (attribute_handle == service_object.characteristic_e_handle){
		return att_read_callback_handle_blob(service_object.characteristic_e_value, 
		                                     strnlen(service_object.characteristic_e_value, MAX_E_VALUE_SIZE), 
											 offset, buffer, buffer_size);
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
	//UNUSED(buffer_size); // No longer unused, as we use it for strncmp and null termination

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
		//buffer[buffer_size] = 0 ; // Redundant: memcpy + next line do this
		//
		memset(service_object.characteristic_b_value, 0, MAX_B_VALUE_SIZE) ;
		memcpy(service_object.characteristic_b_value, buffer, (buffer_size < MAX_B_VALUE_SIZE) ? buffer_size : (MAX_B_VALUE_SIZE -1) ) ;
		// Null-terminate the string
		service_object.characteristic_b_value[(buffer_size < MAX_B_VALUE_SIZE) ? buffer_size : (MAX_B_VALUE_SIZE -1) ] = 0 ;
		// clear ble_input buffer
		memset(blecmdtxt_ptr->ble_input, 0, BLECMD_BUFFER_SIZE) ;  // fill a block of memory with a specific value
		memcpy(blecmdtxt_ptr->ble_input, buffer, (buffer_size < BLECMD_BUFFER_SIZE) ? buffer_size : (BLECMD_BUFFER_SIZE - 1)) ;   // copy to buffer in defs.h, used in main.c
        blecmdtxt_ptr->ble_input[(buffer_size < BLECMD_BUFFER_SIZE) ? buffer_size : (BLECMD_BUFFER_SIZE - 1)] = '\0'; // Ensure null termination

#ifdef UART_LOG
        uart_puts(UART_ID, "Inside service_impl, Command from client:\n\r");
#endif
        blecmdtxt_ptr->is_cltCmd = true;    // we have command from client; used in main.c
        //blecmdtxt_ptr->is_cltTxt = false;    // do we have a client message
        //blecmdtxt_ptr->is_pcbLed = false;    // did client press LED on or off

        // **************
        // Call to FreeRTOS task!!!!
        // **************
        // Send a notification to the handler task vBLEinput_HandlerTask() in main.c
        // Get xBLEinput_HandlerTask from main.c
        xTaskNotifyGive( get_xBLEinput_HandlerTask());      // Call to FreeRTOS task!!!!

	    // If client has enabled notifications, register a callback
	    if (instance->characteristic_b_client_configuration) {
		    instance->callback_b.callback = &characteristic_b_callback;
			instance->callback_b.context = (void*) instance;
			att_server_register_can_send_now_callback(&instance->callback_b, instance->con_handle);
		}
	}

    // Characteristic C
    // Write LED status and control characteristic value
    if (attribute_handle == service_object.characteristic_c_handle) {

        // Check the received data.  BTstack *already* copies the data into the
        // characteristic's value buffer.  We just need to check what it is.
        if (strncmp((char *)buffer, "off", buffer_size) == 0) {
            veh_ptr->led_pcb_on = false;  // PCB LED OFF
        } else if (strncmp((char *)buffer, "on", buffer_size) == 0) {
            veh_ptr->led_pcb_on = true;  // PCB LED ON
        } else {
            // Handle unexpected input (optional, but good practice)
            #ifdef UART_LOG
            uart_puts(UART_ID, "Inside service_impl, Unexpected LED Control Command from client:\n\r");
            #endif
        }

        blecmdtxt_ptr->is_pcbLed = true;    // did client press LED on or off
        xTaskNotifyGive(get_xBLEinput_HandlerTask());      // Call to FreeRTOS task

        // If client has enabled notifications, send them NOW.  This is important!
        if (service_object.characteristic_c_client_configuration) {
             // Directly call the notification callback. This is much cleaner
             // and more efficient than re-registering the callback every time.
            characteristic_c_callback(&service_object);
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
		custom_service_t * instance = &service_object;
		//buffer[buffer_size] = 0 ; // Redundant

		memset(service_object.characteristic_e_value, 0, MAX_E_VALUE_SIZE);
		memcpy(service_object.characteristic_e_value, buffer, (buffer_size < MAX_E_VALUE_SIZE) ? buffer_size : (MAX_E_VALUE_SIZE -1));
		// Null-terminate the string
		service_object.characteristic_e_value[(buffer_size < MAX_E_VALUE_SIZE) ? buffer_size : (MAX_E_VALUE_SIZE -1) ] = 0;
/*
		uart_puts(UART_ID, "Charactertic_e text: ");
		uart_puts(UART_ID, service_object.characteristic_e_value);
		uart_puts(UART_ID, "\n\r");
*/
		// clear ble_input buffer to force xBLEinput_HandlerTask() to interpret this as a text message
//		memset(blecmdtxt_ptr->ble_input, 0, sizeof(blecmdtxt_ptr->ble_input)) ;  // fill a block of memory with a specific value
//		memcpy(blecmdtxt_ptr->ble_input, "bletxt", 6);            // set message flag used in main.c
                //blecmdtxt_ptr->is_cltCmd = false;    // we have text; used in main.c
                blecmdtxt_ptr->is_cltTxt = true;       // we have a client message
                //blecmdtxt_ptr->is_pcbLed = false;    // did client press LED on or off
#ifdef UART_LOG
                uart_puts(UART_ID, "Inside service_impl, message from client:\n\r");
#endif
		// clear client_message buffer used in main.c
		memset(blecmdtxt_ptr->client_message, 0, CLIENT_MESSAGE_BUFFER_SIZE) ;  // fill a block of memory with a specific value
		memcpy(blecmdtxt_ptr->client_message, buffer, (buffer_size < CLIENT_MESSAGE_BUFFER_SIZE) ? buffer_size : (CLIENT_MESSAGE_BUFFER_SIZE-1)) ;   // copy to buffer in defs.h, used in main.c
        blecmdtxt_ptr->client_message[(buffer_size < CLIENT_MESSAGE_BUFFER_SIZE) ? buffer_size : (CLIENT_MESSAGE_BUFFER_SIZE-1)] = '\0'; // Ensure null termination
/*
		uart_puts(UART_ID, "Text from client: ");
		uart_puts(UART_ID, blecmdtxt_ptr->client_message);
		uart_puts(UART_ID, "\n\r");
*/
        // **************
        // Call to FreeRTOS task!!!!
        // **************
        // Send a notification to the handler task vBLEinput_HandlerTask() in main.c
        // Get xBLEinput_HandlerTask from main.c
        xTaskNotifyGive( get_xBLEinput_HandlerTask());      // Call to FreeRTOS task!!!!

		// If client has enabled notifications, register a callback
		if (instance->characteristic_e_client_configuration) {
		    instance->callback_e.callback = &characteristic_e_callback;
			instance->callback_e.context = (void*) instance;
			att_server_register_can_send_now_callback(&instance->callback_e, instance->con_handle);
		}
	}


	return 0;
}


/////////////////////////////////////////////////////////////////////////////
////////////////////////////// USER API /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// Initialize our custom service handler
void custom_service_server_init(void){

	// Pointer to our service object
	custom_service_t * instance = &service_object;

    // Initialize buffers with empty strings (null termination)
    instance->characteristic_a_value[0] = '\0';
    instance->characteristic_b_value[0] = '\0';
    instance->characteristic_c_value[0] = '\0';
    instance->characteristic_d_value[0] = '\0';
    instance->characteristic_e_value[0] = '\0';


	// Assign characteristic user description
	instance->characteristic_a_user_description = characteristic_a;
	instance->characteristic_b_user_description = characteristic_b;
	instance->characteristic_c_user_description = characteristic_c;
	instance->characteristic_d_user_description = characteristic_d;
	instance->characteristic_e_user_description = characteristic_e;


    // Read Heartbeat Counter Characteristic
	// Assign handle values (from generated gatt header)
	// handle names found at:/home/arbz/pico/ab_ftos_ble/build/src/generated/server_gattfile.h
	// Counter
	instance->characteristic_a_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
	instance->characteristic_a_client_configuration_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_a_user_description_handle = ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;
    // Command to Pico Characteristic
	instance->characteristic_b_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
	instance->characteristic_b_client_configuration_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_b_user_description_handle = ATT_CHARACTERISTIC_0000FF12_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;
    // LED status and controll characteristic
	instance->characteristic_c_handle = ATT_CHARACTERISTIC_0000FF13_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
    instance->characteristic_c_client_configuration_handle = ATT_CHARACTERISTIC_0000FF13_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_c_user_description_handle = ATT_CHARACTERISTIC_0000FF13_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;
	// Pico Temperature
	instance->characteristic_d_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
	instance->characteristic_d_client_configuration_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_d_user_description_handle = ATT_CHARACTERISTIC_0000FF14_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;
    // Text to Pico Characteristic
	instance->characteristic_e_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE;
	instance->characteristic_e_client_configuration_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE;
	instance->characteristic_e_user_description_handle = ATT_CHARACTERISTIC_0000FF15_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE;

	// Service start and end handles (modeled off heartrate example)
	service_handler.start_handle = 0;
	service_handler.end_handle = 0xFFFF;
	service_handler.read_callback = &custom_service_read_callback;
	service_handler.write_callback = &custom_service_write_callback;

	// Register the service handler
	att_server_register_service_handler(&service_handler);       // BTstack defined function

/*
	// Register the service handler
	int result = att_server_register_service_handler(&service_handler);       // BTstack defined function
    if (result != 0) {
        printf("Error registering service handler: %d\n", result);
        // Handle the error appropriately.  Perhaps retry, or exit.
    }
*/
}

// Update Characteristic A value(Heartbeat counter); called from main.c
void set_characteristic_a_value(int value){

	// Pointer to our service object
	custom_service_t * instance = &service_object;

	// Update field value
    snprintf(instance->characteristic_a_value, MAX_A_VALUE_SIZE, "%d", value);

	// Are notifications enabled? If so, register a callback
	if (instance->characteristic_a_client_configuration){
		instance->callback_a.callback = &characteristic_a_callback;
		instance->callback_a.context  = (void*) instance;
		att_server_register_can_send_now_callback(&instance->callback_a, instance->con_handle);
	}
}

// Update Characteristic C value(Led Control); called from main.c
// String value = "on" or "off"

// Update Characteristic C value (Led Control); called from your main code.
void set_characteristic_c_value(const char *value) {
    custom_service_t *instance = &service_object;

    // Copy the string, making sure not to overflow the buffer.
    strncpy(instance->characteristic_c_value, value, LED_STATUS_BUFFER_SIZE - 1);
    instance->characteristic_c_value[LED_STATUS_BUFFER_SIZE - 1] = '\0'; // Ensure null-termination

    // Check if notifications are enabled, and send if they are.
    if (instance->characteristic_c_client_configuration) {
        characteristic_c_callback(instance);  // Use the callback function
    }
}



// Update Characteristic D value (Pico temperature); called from main.c
void set_characteristic_d_value(int value){

	// Pointer to our service object
	custom_service_t * instance = &service_object;

	// Update field value
    snprintf(instance->characteristic_d_value, MAX_D_VALUE_SIZE, "%d", value);

	// Are notifications enabled? If so, register a callback
	if (instance->characteristic_d_client_configuration){
		instance->callback_d.callback = &characteristic_d_callback;
		instance->callback_d.context  = (void*) instance;
		att_server_register_can_send_now_callback(&instance->callback_d, instance->con_handle);
	}
}
