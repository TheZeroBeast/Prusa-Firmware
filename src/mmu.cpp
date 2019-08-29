//mmu.cpp

#include "planner.h"
#include "language.h"
#include "lcd.h"
#include "uart2.h"
#include "temperature.h"
#include "Configuration_prusa.h"
#include "fsensor.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "sound.h"
#include "printers.h"
#include <avr/pgmspace.h>
#include "io_atmega2560.h"
#include "AutoDeplete.h"

#ifdef TMC2130
#include "tmc2130.h"
#endif //TMC2130

#define CHECK_FINDA ((IS_SD_PRINTING || is_usb_printing) && (mcode_in_progress != 600) && !saved_printing && e_active())

#define MMU_TODELAY 100
#define MMU_TIMEOUT 10
#define MMU_CMD_TIMEOUT 300000ul //5min timeout for mmu commands (except P0)
#define MMU_P0_TIMEOUT 3000ul    //timeout for P0 command: 3seconds

#ifdef MMU_HWRESET
#define MMU_RST_PIN 76
#endif //MMU_HWRESET

#ifdef MMU_DEBUG
static const auto DEBUG_PUTCHAR = putchar;
static const auto DEBUG_PUTS_P = puts_P;
static const auto DEBUG_PRINTF_P = printf_P;
#else //MMU_DEBUG
#define DEBUG_PUTCHAR(c)
#define DEBUG_PUTS_P(str)
#define DEBUG_PRINTF_P(__fmt, ...)
#endif //MMU_DEBUG

namespace
{ // MMU2S States
enum class S : uint_least8_t
{
  FindaInit,
  GetActExt,
  GetBN,
  GetVer,
  Init,
  Disabled,
  Idle,
  GetFinda,
  Wait,
  Setup
};
}

bool mmu_enabled = false;
bool mmu_ready = false;
bool mmu_fil_loaded = false; //if true: blocks execution of duplicit T-codes
bool isMMUPrintPaused = false;
//bool mmuIR_SENSORLoading = false;
int lastLoadedFilament = -10;
uint16_t mmu_power_failures = 0;
void shutdownE0(bool shutdown = true);

uint16_t toolChanges = 0;
uint8_t mmuE0BackupCurrents[2] = {0, 0};
#define TXTimeout 60 //60ms
//uint8_t unload_filament_type[3] = {0, 0, 0};
uint8_t mmu_filament_types[5] = {0, 0, 0, 0, 0};
void mmu_unload_synced(uint16_t _filament_type_speed);

//idler ir sensor
static bool mmu_idl_sens = 0;
bool ir_sensor_detected = false;

//if IR_SENSOR defined, always returns true
//otherwise check for ir sensor and returns true if idler IR sensor was detected, otherwise returns false
bool check_for_ir_sensor()
{
  return true;
}

static S mmu_state = S::Disabled; //int8_t mmu_state = 0;
MmuCmd mmu_cmd = MmuCmd::None;
MmuCmd mmu_last_cmd = MmuCmd::None;
uint8_t mmu_extruder = 0;

//! This variable probably has no meaning and is planed to be removed
uint8_t tmp_extruder = 0;
int8_t mmu_finda = -1;
int16_t mmu_version = -1;
int16_t mmu_buildnr = -1;
uint32_t mmu_last_request = 0;
uint32_t mmu_last_response = 0;

//initialize mmu2 unit - first part - should be done at begining of startup process
void mmu_init(void)
{
#ifdef MMU_HWRESET
  digitalWrite(MMU_RST_PIN, HIGH);
  pinMode(MMU_RST_PIN, OUTPUT); //setup reset pin
#endif                          //MMU_HWRESET
  uart2_init();                 //init uart2
  _delay_ms(10);                //wait 10ms for sure
  mmu_reset();                  //reset mmu (HW or SW), do not wait for response
  mmu_state = S::Init;
  PIN_INP(IR_SENSOR_PIN); //input mode
  PIN_SET(IR_SENSOR_PIN); //pullup
}

//! @brief Is nozzle hot enough to move extruder wheels and do we have idler sensor?
//!
//! Do load steps only if temperature is higher then min. temp for safe extrusion and
//! idler sensor present.
//! Otherwise "cold extrusion prevented" would be send to serial line periodically
//! and watchdog reset will be triggered by lack of keep_alive processing.
//!
//! @retval true temperature is high enough to move extruder
//! @retval false temperature is not high enough to move extruder, turned
//!         off E-stepper to prevent over-heating and allow filament pull-out if necessary
bool can_extrude()
{
  if ((degHotend(active_extruder) < EXTRUDE_MINTEMP) || !ir_sensor_detected)
  {
    disable_e0();
    delay_keep_alive(100);
    return false;
  }
  return true;
}

void mmu_loop(void)
{
  uint8_t filament = 0;
#ifdef MMU_DEBUG
  printf_P(PSTR("MMU loop, state=%d\n"), (int)mmu_state);
#endif //MMU_DEBUG

  // Copy volitale vars as local
  String tData; cli();
  tData[0] = rxData1;
  tData[1] = rxData2;
  tData[2] = rxData3;
  tData[3] = rxData4;
  tData[4] = rxData5;
  bool confPayload = confirmedPayload;
  sei();

  if (txACKNext)
    uart2_txACK();
  if (txNAKNext)
    uart2_txACK(false);
  if (txRESEND)
    uart2_txPayload(lastTxPayload, true);
  if (confPayload)
    mmu_last_response = _millis();
  else
  {
    tData = "";
  }

  // All Notification & Status Updates between MK3S/MMU2S
  if (mmu_state > S::Disabled)
  {
    // Undates Active Extruder when every MMU is changed. eg. manual ex selection fro feed_filament
    if (tData.startsWith("AE") && (tData[2] < 5))
    {
      printf_P(PSTR("MMU => MK3 'OK Active Extruder:%d'\n"), tmp_extruder + 1);
      tmp_extruder = tData[2];
      mmu_extruder = tmp_extruder;
    }
    else if (tData == "SETUP")
      mmu_state = S::Setup;
    else if (tData.startsWith("ZZZ"))
    { // Clear MK3 Messages
      //********************
      lcd_setstatuspgm(_T(WELCOME_MSG));
      lcd_return_to_status();
      lcd_update_enable(true);
    }
    else if (tData.startsWith("ZZR"))
    { // Advise MMU2S to reconnect
      lcd_setstatuspgm(_T(WELCOME_MSG));
      lcd_return_to_status();
      lcd_update_enable(true);
      tmp_extruder = MMU_FILAMENT_UNKNOWN;
      mmu_extruder = MMU_FILAMENT_UNKNOWN;
      mmu_enabled = false;
      mmu_reset();
      mmu_state = S::Init;
    }
    else if (tData.startsWith("ZL1"))
    {                                        // MMU Loading Failed
                                             //********************
      lcd_setstatus("MMU Load Failed @MMU"); // 20 Chars
#ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedLoad1\n"));
#endif // OCTO_NOTIFICATIONS_ON
    }
    else if (tData.startsWith("ZL2"))
    {                                        // MMU Loading Failed
                                             //********************
      lcd_setstatus("MMU Load Failed @MK3"); // 20 Chars
#ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedLoad2\n"));
#endif // OCTO_NOTIFICATIONS_ON
      mmu_idl_sens = false;
      mmu_state = S::Wait;
    }
    else if (tData.startsWith("ZU"))
    {                                        // MMU Unloading Failed
                                             //********************
      lcd_setstatus(" MMU Unload Failed  "); // 20 Chars
#ifdef OCTO_NOTIFICATIONS_ON
      printf_P(PSTR("// action:mmuFailedUnload\n"));
#endif // OCTO_NOTIFICATIONS_ON
    }
    else if (tData.startsWith("Z1"))
    {                                        // MMU Filament Loaded
                                             //********************
      lcd_setstatus("ERR: Filament Loaded"); // 20 Chars
    }
    else if (tData.startsWith("X1"))
    {                                        // MMU Setup Menu
                                             //********************
      lcd_setstatus("   MMU Setup Menu   "); // 20 Chars
    }
    else if (tData.startsWith("X2"))
    {                                        // MMU Adj Bowden Len
                                             //********************
      lcd_setstatus("MMU Adj BowLen/Bond "); // 20 Chars
    }
    else if (tData.startsWith("X5"))
    {                                        // MMU Setup Menu: Unlock EEPROM
                                             //********************
      lcd_setstatus(" MMU Unlock EEPROM  "); // 20 Chars
    }
    else if (tData.startsWith("X6"))
    {                                        // MMU Setup Menu: Clr Unlocked EEPROM
                                             //********************
      lcd_setstatus("Clr EEPROM(unlocked)"); // 20 Chars
    }
    else if (tData.startsWith("X7"))
    {                                   // MMU Setup Menu: Exit
                                        //********************
      lcd_setstatus("Exit Setup Menu"); // 20 Chars
      lcd_return_to_status();
      lcd_update_enable(true);
    }
    else if (tData.startsWith("V"))
    { // MMU Adj Bowden Len: Loaded Message
      //********************
      lcd_update_enable(false);
      lcd_clear(); //********************
      lcd_set_cursor(0, 0);
      lcd_puts_P(_i("L: +2mm (Extrude)   "));
      lcd_set_cursor(0, 1);
      lcd_puts_P(_i("M:Unload(Save/Retry)"));
      lcd_set_cursor(0, 2);
      lcd_puts_P(_i("R: -2mm (Retract)   "));
      lcd_set_cursor(0, 3);
      lcd_puts_P(_i("Current mm: "));
      lcd_set_cursor(12, 3);
      lcd_print((tData[1] << 8) | (tData[2]));
    }
    else if (tData.startsWith("W"))
    { // MMU Adj Bowden Len: Unloaded Message
      //********************
      lcd_update_enable(false);
      lcd_clear(); //********************
      lcd_set_cursor(0, 0);
      lcd_puts_P(_i("L:   Save & Exit    "));
      lcd_set_cursor(0, 1);
      lcd_puts_P(_i("M:BowLen Load2Check "));
      lcd_set_cursor(0, 2);
      lcd_puts_P(_i("                    ")); //R:Set BondTech Steps"));
      lcd_set_cursor(0, 3);
      lcd_puts_P(_i("Current mm: "));
      lcd_set_cursor(12, 3);
      lcd_print((tData[1] << 8) | (tData[2]));
    }
    else if (tData.startsWith("T"))
    { // MMU Report ToolChange Count
      toolChanges = ((tData[1] << 8) | (tData[2]));
      printf_P(PSTR("MMU => MK3 '@toolChange:%d'\n"), toolChanges);
    }
  } // End of mmu_state > S::Disabled

  switch (mmu_state)
  {
  case S::Disabled:
    return;
  case S::Init:
    if (tData.startsWith("STR"))
    {
      #ifdef MMU_DEBUG
      puts_P(PSTR("MMU => MK3 'start'"));
      puts_P(PSTR("MK3 => MMU 'S1'"));
      #endif //MMU_DEBUG
      uart2_txPayload((unsigned char *)"S1---");
      mmu_state = S::GetVer;
    }
    else if (mmu_last_response + MMU_CMD_TIMEOUT < _millis())
    { //30sec after reset disable mmu
      puts_P(PSTR("MMU not responding - DISABLED"));
      mmu_state = S::Disabled;
    }       // End of if STR
    return; // Exit method.
  case S::GetVer:
    if (tData.startsWith("OK"))
    {
      mmu_version = ((tData[2] << 8) | (tData[3]));
      printf_P(PSTR("MMU => MK3 '%d'\n"), mmu_version);
      #ifdef MMU_DEBUG
      puts_P(PSTR("MK3 => MMU 'S2'"));
      #endif //MMU_DEBUG
      uart2_txPayload((unsigned char *)"S2---");
      mmu_state = S::GetBN;
    }
    return; // Exit method.
  case S::GetBN:
    if (tData.startsWith("OK"))
    {
      mmu_buildnr = ((tData[2] << 8) | (tData[3]));
      printf_P(PSTR("MMU => MK3 '%d'\n"), mmu_buildnr);
      bool version_valid = mmu_check_version();
      if (!version_valid)
        mmu_show_warning();
      else
        puts_P(PSTR("MMU version valid"));
      uart2_txPayload((unsigned char *)"S3---");
      mmu_state = S::GetActExt;
    }
    return; // Exit method.
  case S::GetActExt:
    if (tData.startsWith("OK"))
    {
      tmp_extruder = tData[2];
      mmu_extruder = tmp_extruder;
      printf_P(PSTR("MMU => MK3 'Active Extruder:%d'\n"), tmp_extruder + 1);
      #ifdef MMU_DEBUG
      puts_P(PSTR("MK3 => MMU 'P0'"));
      #endif //MMU_DEBUG
      uart2_txPayload((unsigned char *)"P0---");
      mmu_state = S::FindaInit;
    }
    return; // Exit method.
  case S::FindaInit:
    if (tData.startsWith("PK"))
    {
      mmu_finda = tData[2];
      #ifdef MMU_DEBUG
      printf_P(PSTR("MMU => MK3 'PK%d'\n"), mmu_finda);
      #endif //MMU_DEBUG
      puts_P(PSTR("MMU - ENABLED"));
      mmu_enabled = true;
      mmu_state = S::Idle;
    }
    return; // Exit method.
  case S::Idle:
    if (mmu_cmd != MmuCmd::None)
    {
      if ((mmu_cmd >= MmuCmd::T0) && (mmu_cmd <= MmuCmd::T4))
      {
        filament = mmu_cmd - MmuCmd::T0;
        printf_P(PSTR("MK3 => MMU 'T%d'\n"), filament);
        unsigned char tempTxCMD[5] = {'T', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempTxCMD);
        //mmuIR_SENSORLoading = true;
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::L0) && (mmu_cmd <= MmuCmd::L4))
      {
        filament = mmu_cmd - MmuCmd::L0;
        printf_P(PSTR("MK3 => MMU 'L%d'\n"), filament);
        unsigned char tempLxCMD[5] = {'L', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempLxCMD);
        mmu_state = S::Wait;
      }
      else if (mmu_cmd == MmuCmd::C0)
      {
        printf_P(PSTR("MK3 => MMU 'C0'\n"));
        uart2_txPayload((unsigned char *)"C0---");
        mmu_state = S::Wait;
        mmu_idl_sens = true;
      }
      else if (mmu_cmd == MmuCmd::U0)
      {
        if (PIN_GET(IR_SENSOR_PIN) == 0) //filament is still detected by idler sensor, printer helps with unlading
        {
          if (can_extrude())
          {
            printf_P(PSTR("Unload 1\n"));
            current_position[E_AXIS] = current_position[E_AXIS] - MMU_LOAD_FEEDRATE * MMU_LOAD_TIME_MS * 0.001;
            plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
            st_synchronize();
          }
        }
        printf_P(PSTR("MK3 => MMU 'U0'\n"));
        uart2_txPayload((unsigned char *)"U0---");
        //toolChanges = 0;
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::E0) && (mmu_cmd <= MmuCmd::E4))
      {
        filament = mmu_cmd - MmuCmd::E0;
        printf_P(PSTR("MK3 => MMU 'E%d'\n"), filament);
        unsigned char tempExCMD[5] = {'E', (uint8_t)filament, BLK, BLK, BLK};
        uart2_txPayload(tempExCMD);
        mmu_state = S::Wait;
      }
      else if (mmu_cmd == MmuCmd::R0)
      {
        printf_P(PSTR("MK3 => MMU 'R0'\n"));
        uart2_txPayload((unsigned char *)"R0---");
        mmu_state = S::Wait;
      }
      else if ((mmu_cmd >= MmuCmd::F0) && (mmu_cmd <= MmuCmd::F4))
      {
        uint8_t extruder = mmu_cmd - MmuCmd::F0;
        unsigned char tempTxCMD[5] = {'F', extruder, mmu_filament_types[extruder], BLK, BLK};
        printf_P(PSTR("MK3 => MMU 'F%d %d'\n"), extruder, mmu_filament_types[extruder]);
        uart2_txPayload(tempTxCMD);
        mmu_state = S::Wait;
      }
      mmu_cmd = MmuCmd::None;
    }
    else if (((mmu_last_response + 300) < _millis()) && !mmu_idl_sens)
    {
      uart2_txPayload((unsigned char *)"P0---");
      mmu_state = S::GetFinda;
    }
    return; // Exit method.
  case S::GetFinda:
    if (mmu_idl_sens)
    {
      if (PIN_GET(IR_SENSOR_PIN) == 0 && mmu_idl_sens)
      {
        uart2_txPayload((unsigned char *)"IRSEN");
        printf_P(PSTR("MK3 => MMU 'Filament seen at extruder'\n"));
        // RMMTODO - May need to get MMU2S to stop until C0 is received.
        mmu_idl_sens = false;
      }
    }
    if (tData.startsWith("IRSEN"))
    {
      printf_P(PSTR("MMU => MK3 'waiting for filament @ MK3 IR Sensor'\n"));
      //mmu_load_step(false);
      //mmu_fil_loaded = true;
      mmu_idl_sens = true;
    }
    if (tData.startsWith("PK"))
    {
      mmu_finda = tData[2];
#ifdef MMU_DEBUG
      printf_P(PSTR("MMU => MK3 'PK%d'\n"), mmu_finda);
#endif //MMU_DEBUG
      if (!mmu_finda && CHECK_FSENSOR && fsensor_enabled)
      {
#ifdef OCTO_NOTIFICATIONS_ON
        printf_P(PSTR("// action:m600\n"));
#endif // OCTO_NOTIFICATIONS_ON
        fsensor_stop_and_save_print();
        enquecommand_front_P(PSTR("PRUSA fsensor_recover")); //then recover
        ad_markDepleted(mmu_extruder);
        if (lcd_autoDepleteEnabled() && !ad_allDepleted())
        {
          enquecommand_front_P(PSTR("M600 AUTO")); //save print and run M600 command
        }
        else
        {
          enquecommand_front_P(PSTR("M600")); //save print and run M600 command
        }
      }
      mmu_state = S::Idle;
      if (mmu_cmd == MmuCmd::None)
        mmu_ready = true;
    }
    else if (((mmu_last_request + MMU_P0_TIMEOUT) < _millis()))
      mmu_state = S::Idle; // RMM was Wait
    return;                // Exit method.
  case S::Wait:
    if (mmu_idl_sens)
    {
      if (PIN_GET(IR_SENSOR_PIN) == 0 && mmu_idl_sens)
      {
        uart2_txPayload((unsigned char *)"IRSEN");
        printf_P(PSTR("MK3 => MMU 'Filament seen at extruder'\n"));
        // RMMTODO - May need to get MMU2S to stop until C0 is received.
        mmu_idl_sens = false;
      }
    }
    if (tData == "IRSEN")
    {
      printf_P(PSTR("MMU => MK3 'waiting for filament @ MK3 IR Sensor'\n"));
      //mmu_load_step(false);
      //mmu_fil_loaded = true;
      mmu_idl_sens = true;
    }
    if (tData.startsWith("U"))
    {
      mmu_unload_synced((tData[1] << 8) | (tData[2]));
      printf_P(PSTR("MMU => MK3 Unload Feedrate: %d%d\n"), tData[1], tData[2]);
    }
    else if (tData.startsWith("OK"))
    {
      printf_P(PSTR("MMU => MK3 'ok'\n"));
      mmu_ready = true;
      mmu_state = S::Idle;
    }
    return; // Exit method.
  }
} // End of mmu_loop() method.

void mmu_reset(void)
{
#ifdef MMU_HWRESET //HW - pulse reset pin
  digitalWrite(MMU_RST_PIN, LOW);
  _delay_us(50);
  printf_P(PSTR("MK3S => MMU2S : HWR RESET\n"));
  digitalWrite(MMU_RST_PIN, HIGH);
#else //SW - send X0 command
  uart2_txPayload((unsigned char *)"X0---");
#endif
} // End of mmu_reset() method.

void mmu_set_filament_type(uint8_t extruder, uint8_t filament)
{
  printf_P(PSTR("MK3 => 'F%d %d'\n"), extruder, filament);
  unsigned char tempFx[5] = {'F', extruder, filament, BLK, BLK};
  uart2_txPayload(tempFx);
} // End of mmu_set_filament_type() method.

//! @brief Enqueue MMUv2 command
//!
//! Call manage_response() after enqueuing to process command.
//! If T command is enqueued, it disables current for extruder motor if TMC2130 driver present.
//! If T or L command is enqueued, it marks filament loaded in AutoDeplete module.
void mmu_command(MmuCmd cmd)
{
  if (((cmd >= MmuCmd::T0) && (cmd <= MmuCmd::T4)) || ((cmd >= MmuCmd::E0) && (cmd <= MmuCmd::E4)))
  {
    disable_e0();
  }
  mmu_last_cmd = mmu_cmd;
  mmu_cmd = cmd;
  mmu_ready = false;
}

void mmu_unload_synced(uint16_t _filament_type_speed)
{
  st_synchronize();
  current_position[E_AXIS] -= 70;
  plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], _filament_type_speed, active_extruder);
  st_synchronize();
  disable_e0();
}

static void get_response_print_info(uint8_t move)
{
  printf_P(PSTR("mmu_get_response - begin move: "), move);
  switch (move)
  {
  case MMU_LOAD_MOVE:
    printf_P(PSTR("load\n"));
    break;
  case MMU_UNLOAD_MOVE:
    printf_P(PSTR("unload\n"));
    break;
  case MMU_TCODE_MOVE:
    printf_P(PSTR("T-code\n"));
    break;
  case MMU_NO_MOVE:
    printf_P(PSTR("no move\n"));
    break;
  default:
    printf_P(PSTR("error: unknown move\n"));
    break;
  }
}

bool mmu_get_response(void)
{
  //	printf_P(PSTR("mmu_get_response - begin\n"));
	KEEPALIVE_STATE(IN_PROCESS);
	while (mmu_cmd != MmuCmd::None)
	{
		delay_keep_alive(100);
	}
	while (!mmu_ready)
	{
    mmu_loop;
    /*if ((txRESEND) || (pendingACK && ((startTXTimeout + TXTimeout) < millis()))) {
      txRESEND         = false;
      confirmedPayload = false;
      startRxFlag      = false;
      uart2_txPayload(lastTxPayload);
    }*/
    if (((mmu_state == S::Wait) || (mmu_state == S::Idle) || mmu_idl_sens) && ((mmu_last_request + MMU_CMD_TIMEOUT) > millis())) {
      delay_keep_alive(100);
    } else {
      break;
    }
	}
	bool ret = mmu_ready;
	mmu_ready = false;
  //	printf_P(PSTR("mmu_get_response - end %d\n"), ret?1:0);
	return ret;
}

//! @brief Wait for active extruder to reach temperature set
//!
//! This function is blocking and showing lcd_wait_for_heater() screen
//! which is constantly updated with nozzle temperature.
void mmu_wait_for_heater_blocking()
{
  while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
  {
    delay_keep_alive(1000);
    lcd_wait_for_heater();
  }
}

void manage_response(bool move_axes, bool turn_off_nozzle)
{
	bool response = false;
	mmu_print_saved = false;
	bool lcd_update_was_enabled = false;
	float hotend_temp_bckp = degTargetHotend(active_extruder);
	float z_position_bckp = current_position[Z_AXIS];
	float x_position_bckp = current_position[X_AXIS];
	float y_position_bckp = current_position[Y_AXIS];
	uint8_t screen = 0; //used for showing multiscreen messages
	while(!response)
	{
		  response = mmu_get_response(); //wait for "ok" from mmu
		  if (!response) { //no "ok" was received in reserved time frame, user will fix the issue on mmu unit
        shutdownE0();  // Drop E0 Currents to 0.
			  if (!mmu_print_saved) { //first occurence, we are saving current position, park print head in certain position and disable nozzle heater
          uint8_t mmu_fail = eeprom_read_byte((uint8_t *)EEPROM_MMU_FAIL);
          uint16_t mmu_fail_tot = eeprom_read_word((uint16_t *)EEPROM_MMU_FAIL_TOT);
          if (mmu_fail < 255)
            eeprom_update_byte((uint8_t *)EEPROM_MMU_FAIL, mmu_fail + 1);
          if (mmu_fail_tot < 65535)
            eeprom_update_word((uint16_t *)EEPROM_MMU_FAIL_TOT, mmu_fail_tot + 1);
				  if (lcd_update_enabled) {
					  lcd_update_was_enabled = true;
					  lcd_update_enable(false);
				  }
				  st_synchronize();
				  mmu_print_saved = true;
#ifdef OCTO_NOTIFICATIONS_ON
          printf_P(PSTR("// action:mmuAttention\n"));
#endif // OCTO_NOTIFICATIONS_ON
				  printf_P(PSTR("MMU not responding\n"));
          KEEPALIVE_STATE(PAUSED_FOR_USER);
				  hotend_temp_bckp = degTargetHotend(active_extruder);
				  if (move_axes) {
					  z_position_bckp = current_position[Z_AXIS];
					  x_position_bckp = current_position[X_AXIS];
					  y_position_bckp = current_position[Y_AXIS];

					  //lift z
					  current_position[Z_AXIS] += Z_PAUSE_LIFT;
					  if (current_position[Z_AXIS] > Z_MAX_POS)
              current_position[Z_AXIS] = Z_MAX_POS;
            plan_buffer_line_curposXYZE(15, active_extruder);
            st_synchronize();

					  //Move XY to side
					  current_position[X_AXIS] = X_PAUSE_POS;
					  current_position[Y_AXIS] = Y_PAUSE_POS;
            plan_buffer_line_curposXYZE(50, active_extruder);
					  st_synchronize();
				  }
				  if (turn_off_nozzle) {
					  //set nozzle target temperature to 0
					  setAllTargetHotends(0);
				  }
			  }

			  //first three lines are used for printing multiscreen message; last line contains measured and target nozzle temperature
			  if (screen == 0) { //screen 0
				  lcd_display_message_fullscreen_P(_i("MMU needs user attention."));
				  screen++;
			  }
			  else {  //screen 1
				  if((degTargetHotend(active_extruder) == 0) && turn_off_nozzle) lcd_display_message_fullscreen_P(_i("Press the knob to resume nozzle temperature."));
				  else lcd_display_message_fullscreen_P(_i("Fix the issue and then press button on MMU unit."));
				  screen=0;
			  }

			  lcd_set_degree();
			  lcd_set_cursor(0, 4); //line 4
			  //Print the hotend temperature (9 chars total) and fill rest of the line with space
			  int chars = lcd_printf_P(_N("%c%3d/%d%c"), LCD_STR_THERMOMETER[0],(int)(degHotend(active_extruder) + 0.5), (int)(degTargetHotend(active_extruder) + 0.5), LCD_STR_DEGREE[0]);
			  lcd_space(9 - chars);


			  //5 seconds delay
			  for (uint8_t i = 0; i < 50; i++) {
				  if (lcd_clicked()) {
					  setTargetHotend(hotend_temp_bckp, active_extruder);
					  break;
				  }
				  delay_keep_alive(100);
			  }
		  }
		  else if (mmu_print_saved) {
			  printf_P(PSTR("MMU starts responding\n"));
        KEEPALIVE_STATE(IN_HANDLER);
			  if (turn_off_nozzle)
			  {
				lcd_clear();
				setTargetHotend(hotend_temp_bckp, active_extruder);
				if (((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)) {
					lcd_display_message_fullscreen_P(_i("MMU OK. Resuming temperature..."));
					delay_keep_alive(3000);
				}
				while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
				{
					delay_keep_alive(1000);
					lcd_wait_for_heater();
				}
			  }
			  if (move_axes) {
				  lcd_clear();
				  lcd_display_message_fullscreen_P(_i("MMU OK. Resuming position..."));
				  current_position[X_AXIS] = x_position_bckp;
				  current_position[Y_AXIS] = y_position_bckp;
        plan_buffer_line_curposXYZE(50, active_extruder);
				  st_synchronize();
				  current_position[Z_AXIS] = z_position_bckp;
        plan_buffer_line_curposXYZE(15, active_extruder);
				  st_synchronize();
			  }
			  else {
				  lcd_clear();
				  lcd_display_message_fullscreen_P(_i("MMU OK. Resuming..."));
				  delay_keep_alive(1000); //delay just for showing MMU OK message for a while in case that there are no xyz movements
			  }
		  }
	}
	if (lcd_update_was_enabled) lcd_update_enable(true);
  shutdownE0(false);  // Reset E0 Currents.
}

void shutdownE0(bool shutdown)
{
  if (shutdown && ((tmc2130_current_h[E_AXIS] != 0) && (tmc2130_current_r[E_AXIS] != 0))) {
      mmuE0BackupCurrents[0] = tmc2130_current_h[E_AXIS];
      mmuE0BackupCurrents[1] = tmc2130_current_r[E_AXIS];
      tmc2130_set_current_h(E_AXIS, 0);
      tmc2130_set_current_r(E_AXIS, 0);
      printf_P(PSTR("E-AXIS Disabled.\n"));
  } else if (!shutdown && ((tmc2130_current_h[E_AXIS] == 0) && (tmc2130_current_r[E_AXIS] == 0))) {
      tmc2130_set_current_h(E_AXIS, mmuE0BackupCurrents[0]);
      tmc2130_set_current_r(E_AXIS, mmuE0BackupCurrents[1]);
      printf_P(PSTR("E-AXIS Enabled.\n"));
  }
}

//! @brief load filament to nozzle of multimaterial printer
//!
//! This function is used only only after T? (user select filament) and M600 (change filament).
//! It is not used after T0 .. T4 command (select filament), in such case, gcode is responsible for loading
//! filament to nozzle.
//!
void mmu_load_to_nozzle()
{
  st_synchronize();

  bool saved_e_relative_mode = axis_relative_modes[E_AXIS];
  if (!saved_e_relative_mode)
    axis_relative_modes[E_AXIS] = true;
  if (ir_sensor_detected)
  {
    current_position[E_AXIS] += 3.0f;
  }
  else
  {
    current_position[E_AXIS] += 7.2f;
  }
  float feedrate = 562;
  plan_buffer_line_curposXYZE(feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 14.4f;
  feedrate = 871;
  plan_buffer_line_curposXYZE(feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 36.0f;
  feedrate = 1393;
  plan_buffer_line_curposXYZE(feedrate / 60, active_extruder);
  st_synchronize();
  current_position[E_AXIS] += 14.4f;
  feedrate = 871;
  plan_buffer_line_curposXYZE(feedrate / 60, active_extruder);
  st_synchronize();
  if (!saved_e_relative_mode)
    axis_relative_modes[E_AXIS] = false;
}

void mmu_M600_wait_and_beep()
{
  //Beep and wait for user to remove old filament and prepare new filament for load

  disable_e0();
  float hotend_temp_bckp = degTargetHotend(active_extruder);
  setAllTargetHotends(0);
  KEEPALIVE_STATE(PAUSED_FOR_USER);

  int counterBeep = 0;
  lcd_display_message_fullscreen_P(_i("Remove old filament and press the knob to start loading new filament."));
  bool bFirst = true;

  while (!lcd_clicked())
  {
    manage_heater();
    manage_inactivity(true);

#if BEEPER > 0
    if (counterBeep == 500)
    {
      counterBeep = 0;
    }
    SET_OUTPUT(BEEPER);
    if (counterBeep == 0)
    {
      if ((eSoundMode == e_SOUND_MODE_LOUD) || ((eSoundMode == e_SOUND_MODE_ONCE) && bFirst))
      {
        bFirst = false;
        WRITE(BEEPER, HIGH);
      }
    }
    if (counterBeep == 20)
    {
      WRITE(BEEPER, LOW);
    }

    counterBeep++;
#endif //BEEPER > 0

    delay_keep_alive(4);
  }
  setTargetHotend(hotend_temp_bckp, active_extruder);
  lcd_clear();
  while ((degTargetHotend(active_extruder) - degHotend(active_extruder)) > 5)
  {
    delay_keep_alive(1000);
    lcd_wait_for_heater();
  }
  WRITE(BEEPER, LOW);
}

//! @brief load filament for mmu v2
//! @par nozzle_temp nozzle temperature to load filament
void mmu_M600_load_filament(bool automatic, float nozzle_temp)
{
  tmp_extruder = mmu_extruder;
  if (!automatic)
  {
#ifdef MMU_M600_SWITCH_EXTRUDER
    bool yes = lcd_show_fullscreen_message_yes_no_and_wait_P(_i("Do you want to switch extruder?"), false);
    if (yes)
      tmp_extruder = choose_extruder_menu();
#endif //MMU_M600_SWITCH_EXTRUDER
  }
  else
  {
    tmp_extruder = ad_getAlternative(tmp_extruder);
  }
  lcd_update_enable(false);
  lcd_clear();
  lcd_set_cursor(0, 1);
  lcd_puts_P(_T(MSG_LOADING_FILAMENT));
  lcd_print(" ");
  lcd_print(tmp_extruder + 1);

  setTargetHotend(nozzle_temp, active_extruder);
  mmu_wait_for_heater_blocking();

  mmu_command(MmuCmd::T0 + tmp_extruder);

  manage_response(false, true); //, MMU_LOAD_MOVE);
  mmu_command(MmuCmd::C0);
  //mmu_continue_loading(is_usb_printing || (lcd_commands_type == LcdCommands::Layer1Cal));
  mmu_extruder = tmp_extruder; //filament change is finished
  mmu_load_to_nozzle();
  load_filament_final_feed();
  st_synchronize();
}

void extr_adj(uint8_t extruder) //loading filament for SNMM
{
  MmuCmd cmd = MmuCmd::L0 + extruder;
  if (extruder > (MmuCmd::L4 - MmuCmd::L0))
  {
    printf_P(PSTR("Filament out of range %d \n"), extruder);
    return;
  }
  mmu_command(cmd);

  //show which filament is currently loaded

  lcd_update_enable(false);
  lcd_clear();
  lcd_set_cursor(0, 1);
  lcd_puts_P(_T(MSG_LOADING_FILAMENT));
  //if(strlen(_T(MSG_LOADING_FILAMENT))>18) lcd.setCursor(0, 1);
  //else lcd.print(" ");
  lcd_print(" ");
  lcd_print(extruder + 1);

  // get response
  manage_response(false, false);

  lcd_update_enable(true);

  //lcd_return_to_status();
}

struct E_step
{
  float extrude;   //!< extrude distance in mm
  float feed_rate; //!< feed rate in mm/s
};
static const E_step ramming_sequence[] PROGMEM =
    {
        {1.0, 1000.0 / 60},
        {1.0, 1500.0 / 60},
        {2.0, 2000.0 / 60},
        {1.5, 3000.0 / 60},
        {2.5, 4000.0 / 60},
        {-15.0, 5000.0 / 60},
        {-14.0, 1200.0 / 60},
        {-6.0, 600.0 / 60},
        {10.0, 700.0 / 60},
        {-10.0, 400.0 / 60},
        {-50.0, 2000.0 / 60},
};

//! @brief Unload sequence to optimize shape of the tip of the unloaded filament
static void filament_ramming()
{
  for (uint8_t i = 0; i < (sizeof(ramming_sequence) / sizeof(E_step)); ++i)
  {
    current_position[E_AXIS] += pgm_read_float(&(ramming_sequence[i].extrude));
    plan_buffer_line_curposXYZE(pgm_read_float(&(ramming_sequence[i].feed_rate)), active_extruder);
    st_synchronize();
  }
}

//! @brief Unload sequence to optimize shape of the tip of the unloaded filament
void mmu_filament_ramming()
{
  for (uint8_t i = 0; i < (sizeof(ramming_sequence) / sizeof(E_step)); ++i)
  {
    current_position[E_AXIS] += pgm_read_float(&(ramming_sequence[i].extrude));
    plan_buffer_line_curposXYZE(pgm_read_float(&(ramming_sequence[i].feed_rate)), active_extruder);
    st_synchronize();
  }
}

//! @brief Rotate extruder idler to catch filament
//! @par synchronize
//!  * true blocking call
//!  * false non-blocking call
void mmu_load_step(bool synchronize)
{
  current_position[E_AXIS] = current_position[E_AXIS] + MMU_LOAD_FEEDRATE * 0.1;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  if (synchronize)
    st_synchronize();
}

//-//
void extr_unload_()
{
  //if(bFilamentAction)
  if (0)
  {
    bFilamentAction = false;
    extr_unload();
  }
  else
  {
    eFilamentAction = FilamentAction::MmuUnLoad;
    bFilamentFirstRun = false;
    if (target_temperature[0] >= EXTRUDE_MINTEMP)
    {
      bFilamentPreheatState = true;
      mFilamentItem(target_temperature[0], target_temperature_bed);
    }
    //     else menu_submenu(mFilamentMenu);
    else
      mFilamentMenu();
  }
}

//! @brief show which filament is currently unloaded
void extr_unload_view()
{
  lcd_clear();
  lcd_set_cursor(0, 1);
  lcd_puts_P(_T(MSG_UNLOADING_FILAMENT));
  lcd_print(" ");
  if (mmu_extruder == MMU_FILAMENT_UNKNOWN)
    lcd_print(" ");
  else
    lcd_print(mmu_extruder + 1);
}

void extr_unload()
{ //unload just current filament for multimaterial printers

  if (degHotend0() > EXTRUDE_MINTEMP)
  {
    st_synchronize();
		//show which filament is currently unloaded
    lcd_setstatuspgm(_T(WELCOME_MSG));
    lcd_return_to_status();
    lcd_update_enable(true);
    char msg[20];
    sprintf_P(msg, PSTR("MMU Unloading Fil:%d"), (mmu_extruder + 1));
                       //********************
    lcd_setstatus(msg); // 20 Chars
    mmu_filament_ramming();
    mmu_command(MmuCmd::U0);
    // get response
    manage_response(false, true); //, MMU_UNLOAD_MOVE);
    lcd_setstatuspgm(_T(WELCOME_MSG));
    lcd_return_to_status();
  }
  else
  {
    show_preheat_nozzle_warning();
  }
}

void load_all()
{
  enquecommand_P(PSTR("M701 E0"));
  enquecommand_P(PSTR("M701 E1"));
  enquecommand_P(PSTR("M701 E2"));
  enquecommand_P(PSTR("M701 E3"));
  enquecommand_P(PSTR("M701 E4"));
}

bool mmu_check_version()
{
  return (mmu_buildnr >= MMU_REQUIRED_FW_BUILDNR);
}

void mmu_show_warning()
{
  printf_P(PSTR("MMU2 firmware version invalid. Required version: build number %d or higher."), MMU_REQUIRED_FW_BUILDNR);
  kill(_i("Please update firmware in your MMU2. Waiting for reset."));
}

void lcd_mmu_load_to_nozzle(uint8_t filament_nr)
{
  menu_back();
  bFilamentAction = false; // NOT in "mmu_load_to_nozzle_menu()"
  if (degHotend0() > EXTRUDE_MINTEMP)
  {
    tmp_extruder = filament_nr;
    lcd_setstatuspgm(_T(WELCOME_MSG));
    lcd_return_to_status();
    lcd_update_enable(true);
    char msg[20];
    sprintf_P(msg, PSTR("MMU Loading Ext:%d"), (tmp_extruder + 1));
                      //********************
    lcd_setstatus(msg); // 20 Chars 
    mmu_command(MmuCmd::T0 + tmp_extruder);
    manage_response(true, true); //, MMU_TCODE_MOVE);
    mmu_command(MmuCmd::C0);
    //mmu_continue_loading(false);
    mmu_extruder = tmp_extruder; //filament change is finished
    //marlin_rise_z();
    mmu_load_to_nozzle();
    load_filament_final_feed();
    st_synchronize();
    custom_message_type = CustomMsg::FilamentLoading;
    lcd_setstatuspgm(_T(MSG_LOADING_FILAMENT));
    lcd_return_to_status();
    lcd_update_enable(true);
    lcd_load_filament_color_check();
    lcd_setstatuspgm(_T(WELCOME_MSG));
    custom_message_type = CustomMsg::Status;
  }
  else
  {
    show_preheat_nozzle_warning();
  }
}

//! @brief Fits filament tip into heatbreak?
//!
//! If PTFE tube is jammed, this causes filament to be unloaded and no longer
//! being detected by the pulley IR sensor.
//! @retval true Fits
//! @retval false Doesn't fit
static bool can_load()
{
  current_position[E_AXIS] += 60;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  current_position[E_AXIS] -= 52;
  plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
  st_synchronize();

  uint_least8_t filament_detected_count = 0;
  const float e_increment = 0.2;
  const uint_least8_t steps = 6.0 / e_increment;
  DEBUG_PUTS_P(PSTR("MMU can_load:"));
  for (uint_least8_t i = 0; i < steps; ++i)
  {
    current_position[E_AXIS] -= e_increment;
    plan_buffer_line_curposXYZE(MMU_LOAD_FEEDRATE, active_extruder);
    st_synchronize();
    if (0 == PIN_GET(IR_SENSOR_PIN))
    {
      ++filament_detected_count;
      DEBUG_PUTCHAR('O');
    }
    else
    {
      DEBUG_PUTCHAR('o');
    }
  }
  if (filament_detected_count > steps - 4)
  {
    DEBUG_PUTS_P(PSTR(" succeeded."));
    return true;
  }
  else
  {
    DEBUG_PUTS_P(PSTR(" failed."));
    return false;
  }
}

void mmu_eject_filament(uint8_t filament, bool recover)
{
  if (filament < 5)
  {

    if (degHotend0() > EXTRUDE_MINTEMP)
    {
      st_synchronize();

      {
        LcdUpdateDisabler disableLcdUpdate;
        lcd_clear();
        lcd_set_cursor(0, 1);
        lcd_puts_P(_i("Ejecting filament"));
        filament_ramming();
        mmu_command(MmuCmd::E0 + filament);
        manage_response(false, false); //, MMU_UNLOAD_MOVE);
        if (recover)
        {
          lcd_show_fullscreen_message_and_wait_P(_i("Please remove filament and then press the knob."));
          mmu_command(MmuCmd::R0);
          manage_response(false, false);
        }
      }
    }
    else
    {
      show_preheat_nozzle_warning();
    }
  }
  else
  {
    puts_P(PSTR("Filament nr out of range!"));
  }
}

static void increment_load_fail()
{
  uint8_t mmu_load_fail = eeprom_read_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL);
  uint16_t mmu_load_fail_tot = eeprom_read_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT);
  if (mmu_load_fail < 255)
    eeprom_update_byte((uint8_t *)EEPROM_MMU_LOAD_FAIL, mmu_load_fail + 1);
  if (mmu_load_fail_tot < 65535)
    eeprom_update_word((uint16_t *)EEPROM_MMU_LOAD_FAIL_TOT, mmu_load_fail_tot + 1);
}
