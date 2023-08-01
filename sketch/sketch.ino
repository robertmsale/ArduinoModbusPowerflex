#include <ArduinoModbus.h>
#include <ArduinoRS485.h>

constexpr auto baudrate { 19200 };

constexpr auto bitduration { 1.f / baudrate };
constexpr auto preDelayBR { bitduration * 9.6f * 3.5f * 1e6 };
constexpr auto postDelayBR { bitduration * 9.6f * 3.5f * 1e6 };

const unsigned short no_fault = 0;
const unsigned short auxiliary_input = 2;
const unsigned short power_loss = 3;
const unsigned short undervoltage = 4;
const unsigned short overvoltage = 5;
const unsigned short motor_stalled = 6;
const unsigned short motor_overload = 7;
const unsigned short heatsink_overtemperature = 8;
const unsigned short control_module_overtemperature = 9;
const unsigned short hw_overcurrent = 12;
const unsigned short ground_fault = 13;
const unsigned short load_loss = 15;
const unsigned short output_phase_loss = 21;
const unsigned short analog_input_loss = 29;
const unsigned short auto_restart_tries = 33;
const unsigned short phase_u_to_ground_short = 38;
const unsigned short phase_v_to_ground_short = 39;
const unsigned short phase_w_to_ground_short = 40;
const unsigned short phase_uv_short = 41;
const unsigned short phase_uw_short = 42;
const unsigned short phase_vw_short = 43;
const unsigned short parameters_defaulted = 48;
const unsigned short safety_open = 59;
const unsigned short software_overcurrent = 63;
const unsigned short drive_overload = 64;
const unsigned short power_unit_fail = 70;
const unsigned short dsi_network_loss = 71;
const unsigned short option_card_network_loss = 72;
const unsigned short embedded_ether_net_ip_adapter_network_loss = 73;
const unsigned short auto_tune_fail = 80;
const unsigned short dsi_communication_loss = 81;
const unsigned short option_card_communication_loss = 82;
const unsigned short embedded_ether_net_ip_adapter_communication_loss = 83;
const unsigned short encoder_loss = 91;
const unsigned short function_loss = 94;
const unsigned short parameter_checksum_error = 100;
const unsigned short external_storage = 101;
const unsigned short control_module_connect_error = 105;
const unsigned short incompatible_control_power_module = 106;
const unsigned short unrecognized_control_power_module = 107;
const unsigned short mismatched_control_power_module = 109;
const unsigned short keypad_membrane = 110;
const unsigned short safety_hardware = 111;
const unsigned short microprocessor_failure = 114;
const unsigned short io_board_fail = 122;
const unsigned short flash_update_required = 125;
const unsigned short non_recoverable_error = 126;
const unsigned short dsi_flash_update_required = 127;

const String msg_no_fault = String("No Fault");
const String msg_auxiliary_input = String("Auxiliary Input");
const String msg_power_loss = String("Power Loss");
const String msg_undervoltage = String("Undervoltage");
const String msg_overvoltage = String("Overvoltage");
const String msg_motor_stalled = String("Motor Stalled");
const String msg_motor_overload = String("Motor Overload");
const String msg_heatsink_overtemperature = String("Heatsink Overtemperature");
const String msg_control_module_overtemperature = String("Control Module Overtemperature");
const String msg_hw_overcurrent = String("HW Overcurrent (300%)");
const String msg_ground_fault = String("Ground Fault");
const String msg_load_loss = String("Load Loss");
const String msg_output_phase_loss = String("Output Phase Loss");
const String msg_analog_input_loss = String("Analog Input Loss");
const String msg_auto_restart_tries = String("Auto Restart Tries");
const String msg_phase_u_to_ground_short = String("Phase U to Ground Short");
const String msg_phase_v_to_ground_short = String("Phase V to Ground Short");
const String msg_phase_w_to_ground_short = String("Phase W to Ground Short");
const String msg_phase_uv_short = String("Phase UV Short");
const String msg_phase_uw_short = String("Phase UW Short");
const String msg_phase_vw_short = String("Phase VW Short");
const String msg_parameters_defaulted = String("Parameters Defaulted");
const String msg_safety_open = String("Safety Open");
const String msg_software_overcurrent = String("Software Overcurrent");
const String msg_drive_overload = String("Drive Overload");
const String msg_power_unit_fail = String("Power Unit Fail");
const String msg_dsi_network_loss = String("DSI Network Loss");
const String msg_option_card_network_loss = String("Option Card Network Loss");
const String msg_embedded_ether_net_ip_adapter_network_loss = String("Embedded EtherNet/IP Adapter Network Loss ");
const String msg_auto_tune_fail = String("AutoTune Fail");
const String msg_dsi_communication_loss = String("DSI Communication Loss");
const String msg_option_card_communication_loss = String("Option Card Communication Loss");
const String msg_embedded_ether_net_ip_adapter_communication_loss = String("Embedded EtherNet/IP Adapter Communication Loss ");
const String msg_encoder_loss = String("Encoder Loss");
const String msg_function_loss = String("Function Loss");
const String msg_parameter_checksum_error = String("Parameter Checksum Error");
const String msg_external_storage = String("External Storage");
const String msg_control_module_connect_error = String("Control Module Connect Error");
const String msg_incompatible_control_power_module = String("Incompatible Control-Power Module");
const String msg_unrecognized_control_power_module = String("Unrecognized Control-Power Module");
const String msg_mismatched_control_power_module = String("Mismatched Control-Power Module");
const String msg_keypad_membrane = String("Keypad Membrane");
const String msg_safety_hardware = String("Safety Hardware");
const String msg_microprocessor_failure = String("Microprocessor Failure");
const String msg_io_board_fail = String("I/O Board Fail");
const String msg_flash_update_required = String("Flash Update Required");
const String msg_non_recoverable_error = String("Non Recoverable Error");
const String msg_dsi_flash_update_required = String("DSI Flash Update Required");

String get_msg_from_fault_code(short code) {
	switch (code) {
		case no_fault: return msg_no_fault;
		case auxiliary_input: return msg_auxiliary_input;
		case power_loss: return msg_power_loss;
		case undervoltage: return msg_undervoltage;
		case overvoltage: return msg_overvoltage;
		case motor_stalled: return msg_motor_stalled;
		case motor_overload: return msg_motor_overload;
		case heatsink_overtemperature: return msg_heatsink_overtemperature;
		case control_module_overtemperature: return msg_control_module_overtemperature;
		case hw_overcurrent: return msg_hw_overcurrent;
		case ground_fault: return msg_ground_fault;
		case load_loss: return msg_load_loss;
		case output_phase_loss: return msg_output_phase_loss;
		case analog_input_loss: return msg_analog_input_loss;
		case auto_restart_tries: return msg_auto_restart_tries;
		case phase_u_to_ground_short: return msg_phase_u_to_ground_short;
		case phase_v_to_ground_short: return msg_phase_v_to_ground_short;
		case phase_w_to_ground_short: return msg_phase_w_to_ground_short;
		case phase_uv_short: return msg_phase_uv_short;
		case phase_uw_short: return msg_phase_uw_short;
		case phase_vw_short: return msg_phase_vw_short;
		case parameters_defaulted: return msg_parameters_defaulted;
		case safety_open: return msg_safety_open;
		case software_overcurrent: return msg_software_overcurrent;
		case drive_overload: return msg_drive_overload;
		case power_unit_fail: return msg_power_unit_fail;
		case dsi_network_loss: return msg_dsi_network_loss;
		case option_card_network_loss: return msg_option_card_network_loss;
		case embedded_ether_net_ip_adapter_network_loss: return msg_embedded_ether_net_ip_adapter_network_loss;
		case auto_tune_fail: return msg_auto_tune_fail;
		case dsi_communication_loss: return msg_dsi_communication_loss;
		case option_card_communication_loss: return msg_option_card_communication_loss;
		case embedded_ether_net_ip_adapter_communication_loss: return msg_embedded_ether_net_ip_adapter_communication_loss;
		case encoder_loss: return msg_encoder_loss;
		case function_loss: return msg_function_loss;
		case parameter_checksum_error: return msg_parameter_checksum_error;
		case external_storage: return msg_external_storage;
		case control_module_connect_error: return msg_control_module_connect_error;
		case incompatible_control_power_module: return msg_incompatible_control_power_module;
		case unrecognized_control_power_module: return msg_unrecognized_control_power_module;
		case mismatched_control_power_module: return msg_mismatched_control_power_module;
		case keypad_membrane: return msg_keypad_membrane;
		case safety_hardware: return msg_safety_hardware;
		case microprocessor_failure: return msg_microprocessor_failure;
		case io_board_fail: return msg_io_board_fail;
		case flash_update_required: return msg_flash_update_required;
		case non_recoverable_error: return msg_non_recoverable_error;
		case dsi_flash_update_required: return msg_dsi_flash_update_required;
	}
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Modbus RTU Client");

  RS485.setDelays(preDelayBR, postDelayBR);

  if (!ModbusRTUClient.begin(baudrate, SERIAL_8E1)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1);
  }
 }

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Reading register values...");

  if (!ModbusRTUClient.requestFrom(1, INPUT_REGISTERS, 0x2121, 1)) {
    Serial.print("Failed! ");
    Serial.println(ModbusRTUClient.lastError());
  } else {
    Serial.println("Success!");

    while (ModbusRTUClient.available()) {
      auto res = ModbusRTUClient.read();
      Serial.println(get_msg_from_fault_code((short)res));
    }
  }
  sleep(5000);
}
