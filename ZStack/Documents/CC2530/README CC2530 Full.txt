Texas Instruments, Inc.

ZStack-CC2530 Release Notes

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

ZigBee 2007 Release
Version 2.3.1-1.4.0
August 20, 2010


Notices:
 - ZStack-CC2530 has been certified for ZigBee/ZigBee-PRO compliance.

 - Z-Stack supports the ZigBee 2007 Specification, including features such
   as PanID Conflict Resolution, Frequency Agility, and Fragmentation. The
   ZigBee 2007 Specification (www.zigbee.org) defines two ZigBee stack
   profiles, ZigBee and ZigBee-Pro. ZStack-2.3.1 provides support for both
   of these profiles. See the Z-Stack Developer's Guide for details.

 - Z-Stack now supports an IAR project to build "ZigBee Network Processor"
   (ZNP) devices. CC2530-based ZNP devices can be used with any host MCU
   that provides either an SPI or UART port to add ZigBee-Pro communication
   to existing or new designs. See the "CC2530ZNP Interface Specification"
   and "Z-Stack User's Guide for CC2530 ZigBee-PRO Network Processor -
   Sample Applications" documents for details and using the ZNP.

 - The library files have been built/tested with EW8051 version 7.51A/W32
   (7.51.1.3) and may not work with other versions of the IAR tools. You
   can obtain the 7.51A installer and patches from the IAR website.

 - When programming a target for the first time with this release, make sure
   that you select the "Erase Flash" in the "Debugger->Texas Instruments->
   Download" tab in the project options. When programming completes, it is
   recommended that the "Erase Flash" box gets un-checked so that NV items
   are retained during later re-programming.

 - Please review the document "Upgrading To Z-Stack v2.3." for information
   about moving existing v2.2 and v2.3.0 applications to v2.3.1.


Changes:
 - Enabled the user to "tune" Frequency Agility start-up by providing access to
   the ZDNWKMGR_MIN_TRANSMISSIONS compile option in f8wConfig.cfg. This
   option is now used to initialize a new NV item, ZCD_NV_NWKMGR_MIN_TX,
   so that Frequency Agility can be configured and retained. In addition, setting
   this parameter to zero disables Frequency Agility. [3396]

 - Improved consistency of naming for fragmentation-related compile flags
   and global variables. These variables and flags have changed: [3340]
       from: APS_DEFAULT_WINDOW_SIZE
           to: APSF_DEFAULT_WINDOW_SIZE
       from: APS_DEFAULT_INTERFRAME_DELAY
           to: APSF_DEFAULT_INTERFRAME_DELAY
       from: zgApscMaxWindowSize
           to: zgApsfMaxWindowSize
       from: zgApsInterframeDelay
           to: zgApsfInterframeDelay

 - Enabled user "tuning" of timing jitter for Link Status messages by creating
   the LINK_STATUS_JITTER_MASK pre-processor definition in f8wConfig.cfg
   files. This allows for randomized time separation of Link Status messages
   from multiple devices in a ZigBee-Pro network. [3326]

 - Enhanced the MT SYS_RESET_REQ command to support both "hard" and
   "soft" resets. Soft reset is useful when serial communication H/W needs to
   be unaffected by the reset operation. See section 3.8.1.1 of the "Z-Stack
   Monitor and Test API" document for use of this command. [3268]

 - Improved operation of the Over Air Download (OAD) bootloader - it will
   continually retry "instantiation" of the downloaded image if the checksum
   validation fails. [3262]

 - Simplified flash memory write code on 8051-based devices by removing
   the "run from SRAM" mechanism. Changing to the preferred DMA transfer
   method allows for removal of special (troublesome to maintain) memory
   sections from the linker control file. [3243]

 - Added capability for ZCL to process messages that are received on the
   broadcast endpoint - ZCL now gets the Endpoint Descriptor's endpoint
   instead of the broadcast endpoint. [3239]

 - Enhanced throughput on 8051-based devices by changing the code bank
   register access setting in IAR project files. The "Register Mask" parameter
   was changed from 0x07 to 0xFF for slightly faster bank switching. [3231]

 - Updated the TX power table and RSSI offset for use with the CC2591 (per
   AN086, TI document SWRA308A). See the "mac_radio_defs.c" file. [3228]

 - Added a new compiler flag, MT_SYS_KEY_MANAGEMENT, to control access
   (read/write/set) to security key data (for test purposes) via MT command.
   MT access to key data is now disabled by default. [3224]

 - The Monitor-Test (MT) API has been extended to provide a "manual" data
   poll request to allow a host device to control polling, instead of via timer
   on the target device. See section 3.10.1.12 of the "Z-Stack Monitor and
   Test API" document for use of the UTIL_DATA_REQ command. [3213]

 - Changed default configurations of ZigBee Network Processor (ZNP) devices
   to provide for firmware upgrades via serial bootloader. SPI, UART and USB
   modes are supported. See section 4.5 of "Z-Stack User's Guide for CC2530
   ZigBee-Pro Network Processor - Sample Applications" for details on usage
   of the bootloader with ZNP. [3204]

 - Added capability to zmac_cb.c to adjust the LQI value that gets passed up
   to the NWK layer. ZMacLqiAdjust() gets called from MAC_CbackEvent() on
   MAC_MCPS_DATA_IND and MAC_MCPS_DATA_CNF events that allows for
   adjustment of the MAC's LQI value (related to signal strength) according to
   packet correlation (releated to signal quality). Refer to section 3.5.2 of the
   "Z-Stack API" and section 13 of the "Z-Stack Developer's Guide" document 
   for details. [3203]

 - Significantly increased reliability of message delivery in large networks
   (tested with >400 routers) by optimizing/reducing broadcast traffic and
   improving management of link status. See "Upgrading To Z-Stack v2.3."
   for specific information on these improvements. [3185]

 - The ZCL API has been extended to add capability to retrieve a pointer to
   the raw AF incoming message from within a ZCL callback function. This
   function can be used to obtain message data that is not passed up to the
   callback, such as LQI or RSSI. Refer to section 3.22 of the "Z-Stack ZCL
   API" document for information on using zcl_getRawAFMsg(). [3183]

 - The Monitor-Test (MT) API has been extended to provide a serial I/O
   loop-back command for testing of a device via the MT interface. Refer
   to section 3.10.1.1 of the "Z-Stack Monitor and Test API" document
   for details of the UTIL_TEST_LOOPBACK command. [3171]

 - Support has been added for commissioning security-related data items at
   the time devices are programmed "at the factory". Certicom certificates,
   consisting of 'Device Private Key', 'Implicit Certificate', and 'CA Public Key'
   may now be programmed into the Lock Bits page of the CC2530 device.
   See section 3.3 of the "Smart Energy Sample Application User's Guide" for
   details on using this feature. [3148]

 - The Monitor-Test (MT) API has been extended to support fragmentation 
   for sending and receiving large messages (tested up to 640 bytes) via the
   MT interface. Refer to sections 3.2.1.6 and 3.2.1.7 of the "Z-Stack Monitor
   and Test API" document on details of using the MT commands. Refer to
   section 9.9 of the "Z-Stack Developer's Guide" for a discussion of usage
   of the fragmentation feature. [3072]

 - A security vulnerability, in which security keys could be read from RAM of
   a halted device, has been closed. Security keys are now stored only in
   non-volatile memory (NV) and read from NV at point-of-use. To support
   security key storage in NV, the following reserved NV ID codes have been
   modified/added: TrustCenter Link Keys (0x0101...0x01FF), APS Link Keys
   (0x0200...0x02FF), Master Keys (0x0300...0x03FF). This moves the start
   of User Defined NV ID codes to 0x0400. It should be noted that security
   keys are now read from NV for each encryption or decryption operation,
   which may result in slower lower throughput under heavy traffic. Use of 
   "hot" NV items (in the OSAL_Nv.c module) minimizes this effect. [3069]

 - Added capability to reduce non-volatile (NV) memory 'fatigue' for networks
   with mobile or rapidly purged reduced functionality devices (RFD). Setting a
   new compile option, ZDO_NV_SAVE_RFDs, to FALSE to disable calls to the
   NLME_UpdateNV() function when the join indication is received. [3034]

 - Power consumption during CSMA has been reduced. The "RX-on" time
   was reduced by one backoff period, except when the requested backoff
   time is zero. [1601]


Bug Fixes:
 - Fixed a problem in which fragmenation would not work properly when the
   INTER_PAN feature was enabled. [3398]

 - Corrected a problem in ZCL for Smart Energy that prevented fragmentation
   from working properly for the zclSE_Pricing_Send_PublishPrice() and the
   zclSE_Message_Send_DisplayMessage() functions.  [3398]

 - Fixed a problem that could occur during processing of a route response that
   resulted in inadvertant, variable long delays (190-440 msec) for subsequent
   message transmissions. [3397]

 - Fixed sample application project files (.ewp) to specify the proper default
   pathname to device configuration files - changing the target device in the
   EW8051 IDE (General Options->Target->Device information->Device) now
   opens the folder containing the device configuration files. [3356]

 - Fixed the return values for the following MT_UTIL commands - they were
   incorrectly identified as a NWK responses: [3348]
      - MT_UTIL_ADDRMGR_EXT_ADDR_LOOKUP
      - MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP
      - MT_UTIL_ASSOC_COUNT
      - MT_UTIL_ASSOC_FIND_DEVICE
      - MT_UTIL_ASSOC_GET_WITH_ADDRESS

 - Modified the bindAddEntry() function to return NULL when a new clusterId
   could not be added to an existing binding table entry. [3339]

 - Fixed two memory leaks in ZCL Key Establishment that is used in Smart
   Energy joining - trouble would be observed after multiple join/leave cycles.
   Affected modules include: zcl_key_establish.c, ZDObject.c, and the esp.c
   file in the SE SampleApp. [3336]

 - Corrected the problem that ignored the DEFAULT_KEY compile option in the
   f8wConfig.cfg file or IDE pre-processor settings. [3325]

 - Corrected errors in the MT_SapiReadCfg() function, including a possible use
   of NULL pointer on memory allocation failure, rejection of valid requests and
   acceptance of invalid requests based on the provided configuration ID code,
   and return of "garbage" data on error responses. [3315]

 - Fixed a problem which disallowed loading a trust center link key (TCLK) to
   the non-volatile (NV) memory of a device in the HOLD_AUTO_START state
   of operation. [3301]

 - Fixed ZCL problems when handling requests for group/scene membership
   from a device that isn't part of the specified group/scene.  [3292]

 - Corrected potential read-modify-write problems relating to interrupt status
   registers, which could result in missed interrupts. Affected modules include 
   mac_mcu, mac_csp_tx, mac_radio_defs. hal_dma, hal_key, hal_sleep, and
   hal_timer. [3284]

 - Fixed an obscure problem in ZDApp.c where an unbraced 'else' statement,
   related to the BLINK_LEDS compile flag, could result in an excluded line of
   code during compilation. Since BLINK_LEDS is specified in "hal_board_cfg"
   files, it was removed from the ZStack ZigBee device configuration files
   (f8wCoord.cfg, f8wRouter.cfg, f8wEndev.cfg). [3277]

 - Fixed the MT return value for the UTIL_APSME_LINK_KEY_DATA_GET
   command - it was incorrectly identified as a NWK response. [3276]

 - The non-volatile memory driver (OSAL_Nv.c) has been upgraded to close
   vulnerability to corruption of NV memory if a device reset occured during
   compaction to a "clean" NV page. Device reset can occur by cycling power,
   voltage drops below brown-out threshold, or program assert. [3267]

 - Reverted inadvertant changes to MT_ZDO response/comfirm/indication
   codes that were introduced with the ZStack-2.3.0 release. The following
   MT_ZDO return codes were changed to provide backward compatibility
   with ZStack-2.2.2 and earlier versions: [3265]
       0x90 --> 0x87  :  MT_ZDO_COMPLEX_DESC_RSP
       0x91 --> 0x88  :  MT_ZDO_USER_DESC_RSP
       0x94 --> 0x89  :  MT_ZDO_USER_DESC_CONF
       0x95 --> 0x8A  :  MT_ZDO_SERVER_DISC_RSP
       0xC1 --> 0xC0  :  MT_ZDO_STATE_CHANGE_IND
       0x93 --> 0xC1  :  MT_ZDO_END_DEVICE_ANNCE_IND
       0xC0 --> 0xC3  :  MT_ZDO_STATUS_ERROR_RSP
       0xC3 --> 0xC4  :  MT_ZDO_SRC_RTG_IND
       0xC4 --> 0xC7  :  MT_ZDO_NWK_DISCOVERY_CNF

 - Fixed a memory leak in the MT_ZdoSendMsgCB() function which failed to
   de-allocate memory used to format an MT callback message. [3256]

 - Fixed an MT_ZDO error response when a Simple Descriptor Request used
   a "ZDP_NOT_ACTIVE" endpoint. [3245]

 - Fixed the GPIO_SET behavior for all 4 GPIOs on ZNP devices. [3238]

 - Lowered the maximum DMA data payload for ZigBee Network Processor
   (ZNP) devices from 253 to 250 to correct a problem where the ZNP would
   hang in the NP_SPI_WAIT_RX state if the payload length was 253. [3218]

 - Corrected a problem where a device would respond to a Match Descriptor
   Request when both input and output cluster lists were empty. [3215]

 - The "mac_cfg.c" file was moved out of the MAC library in order to permit
   user configuration of queue size parameters (MAC_CFG_TX_DATA_MAX,
   MAC_CFG_TX_MAX, MAC_CFG_RX_DATA_MAX) when tuning is needed
   for larger networks. Overrides for these parameters were also added to
   the ZStack ZigBee device configuration files (f8wCoord.cfg, f8wRouter.cfg,
   f8wEndev.cfg) to increase MAC frame buffers for larger networks. [3212]

 - The "mac_pib.c" file was moved out of the MAC library in order to permit
    compile-time use of the HAL_PA_LNA option. The MAC library previously
    was built with this option disabled. [3210]

 - Filtering of unsecured packets in a secure device has been extended to
   reject all but APS_CMD and APS_ACK frames that are used in link key
   establishment and entity authentication functions. [3207]

 - Packet sizes for some NWK and APS command frames were not being
   validated before attempting to process the frame. Command frames that
   fail expected size checking are now rejected. [3206]

 - Fixed a problem where a device, built with NV_RESTORE disabled, would
   drop a Transport Key command from a Coordinator that always sent a
   frame counter value of zero to a joining device. [3200]

 - Fixed a problem where a NULL pointer could get used during a search for
   an empty slot in the neighbor table. [3199]

 - Corrected a problem with Link Status messages when the neighbor table 
   was larger that 32 in a non-secure network - the first Link Status message
   would be incorrectly formatted (the "cnt" field was wrong). [3198]

 - Fixed two bounds checks in the non-volatile (NV) memory driver that could
   cause corrupted data at the end of an NV page and possibly fail to perform
   a page compaction when necessary. [3188]

 - Fixed a problem where an End-Device (security enabled) would stop the
   joining process at the Rejoin Request when a Link Status message was
   recieved before the Rejoin Response. End-Devices now filter out incoming
   Link Status messages. [3184]

 - Fixed a rare problem in which routers would re-broadcast an invalid NWK
   command, even though ignored by the destination device. [3177]

 - Corrected a problem where devices using APS security would not be able
   to commuicate anymore after one of them was reset. The device that was
   reset would lose the frame counter - it is now saved into non-volatile (NV)
   memory and restored when the device is reset. [3158]

 - Fixed a problem in which the MAC could drop valid beacons during an
   active/passive scan process. This could occasionally result in not using the
   "best" beacon because it never made it to the selection algorithm. [3124]

 - Fixed a problem in which an Over-Air-Download transfer via dongle would
   slow down from ~60ms to ~500ms per packet after ~1 minute of operation,
   making a 240K OAD upgrade take ~10 times longer than expected. [3107]


Memory Sizes:
 - The CC2530 has 256K bytes of Flash memory to store executable program
   and non-volatile (NV) memory, and 8K bytes of RAM for program stack and
   data variables. Actual usage of Flash and RAM memory is variable, of course,
   and dependent on the specific application. Developers can adjust various
   parameters, such as, program stack size and dynamic memory heap size
   to meet their specific needs.

 - The following table provides a comparison of Flash and RAM sizes for one
   of the sample applications provided with ZStack - SampleApp that is found
   in the installed ..\Projects\zstack\Samples\SampleApp\CC2530DB folder. In
   most ZStack sample applications, generic heap settings are used which have
   been selected to accomodate a wide range of applications. For this example,
   heap settings were: Coordinator/Router = 3K bytes, EndDevice = 2K bytes.
   See the "Heap Memory Management" document for details on profiling heap
   memory usage.

 - Memory sizes are shown below for the 3 ZigBee device types, with/without
   ZigBee-PRO, with/without Security, and compiled to run on the SmartRF05EB
   board with CC2530EM module. See the Z-Stack User's Guide for more details.

                             Coordinator        Router         EndDevice
    PRO   SECURE     Flash/RAM      Flash/RAM      Flash/RAM
   ===  ======   ========   ========   ========
    Off        Off        132.0K/6.5K    131.2K/6.5K    109.1K/5.1K
    Off        On        140.0K/6.6K    140.1K/6.6K    116.1K/5.2K
    On        Off        142.4K/6.6K    141.6K/6.6K    122.0K/5.2K
    On        On        150.3K/6.7K    150.5K/6.7K    121.8K/5.2K


Known Issues:
 - Corruption of the call-stack can occur if the MAX_BINDING_CLUSTER_IDS
   compile option is changed from the default value. Do not change the value
   of this parameter in the f8wConfig.cfg file. [3394]

 - The maximum time that a device can sleep and maintain proper system
   timing is approximately 20.97 seconds (65535*320us) - this results from
   a 16-bit counter for 320us ticks from the MAC timer. It is recommended
   (for End-Devices using POWER_SAVING) that the application provide an
   event that continuously runs on a 20-second OSAL timer if the polling
   period is greater that 20 seconds. [3004]

 - To disable security at build time, use the "SECURE=0" compile option. Do
   not attempt to disable security by setting the SECURITY_LEVEL to zero.

 - SerialApp is not reliable when used for high-speed, large file transfers -
   the receiving application occasionally drops a byte.

 - The ZDO Complex Descriptor is not supported.

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

For technical support please contact:

Texas Instruments, Inc.
Low Power RF
lpwsupport@ti.com
