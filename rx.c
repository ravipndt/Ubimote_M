
#include "ubimotesez_config.h"
#include "bsp.h"
#include "bsp_key.h"
#include "hal_int.h"
#include "basic_rf.h"
#include "hal_rf.h"
#include "hal_timer_32k.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "bsp_led.h"
#include "uartstdio.h"
#include "hw_ioc.h"
#include "ioc.h"
#include "string.h"
#include "sys_ctrl.h"

moteCfg_t moteConfig = {0};                                 //typedef struct                    this structure is given in ubimotesez_config.h
                                                            //{
                                                            //unsigned char mode;             //!< PER test mode. [RX|TX]
                                                            //unsigned char channel;          //!< PER test IEEE channel [11,26]
                                                            // unsigned char txPower;          //!< PER test TX power
                                                            // unsigned char gainMode;         //!< Gain mode
                                                            //} moteCfg_t;

basicRfCfg_t basicRfConfig;                                 //typedef struct {                  this structure is given in basic_rf.h
    														//uint16 myAddr;
    														//uint16 panId;
    														//uint8 channel;
    														//uint8 ackRequest;
   															//	#ifdef SECURITY_CCM
    														//	uint8* securityKey;
    														//	uint8* securityNonce;
    														//	#endif
															//	} basicRfCfg_t;

volatile unsigned char timer_experied = 0;
uint8_t payload[48];

static void receive(void);
static void UARTlibinit(void);

int main(void){

	// Initialize board
	bspInit(BSP_SYS_CLK_SPD);                           ////! Default system clock speed        ////#define BSP_SYS_CLK_SPD         32000000UL
	
	                                                    ////* @brief    This function initializes the CC2538 clocks and I/O for use on
														//				*           SmartRF06EB.
														//				*
														//				*           The function assumes that an external crystal oscillator is
														//				*           available to the CC2538. The CC2538 system clock is set to the
														//				*           frequency given by input argument \e ui32SysClockSpeed. The I/O
														//				*           system clock is set configured to the same value as the system
														//				*           clock.
														//				*
														//				*           If the value of \e ui32SysClockSpeed is invalid, the system clock
														//				*           is set to the highest allowed value.
														//				*
														//				* @param    ui32SysClockSpeed   is the system clock speed in Hz; it must be one
														//				*                               of the following:
														//				*           \li \b SYS_CTRL_32MHZ
														//				*           \li \b SYS_CTRL_16MHZ
														//				*           \li \b SYS_CTRL_8MHZ
														//				*           \li \b SYS_CTRL_4MHZ
														//				*           \li \b SYS_CTRL_2MHZ
														//				*           \li \b SYS_CTRL_1MHZ
														//				*           \li \b SYS_CTRL_500KHZ
														//				*           \li \b SYS_CTRL_250KHZ
														//				*
														//				* @return   None
														//				******************************************************************************/
														//				void
														//				bspInit(uint32_t ui32SysClockSpeed)
														//				{
														//				    uint32_t ui32SysDiv;
																		
																		    //
																		    // Disable global interrupts
																		    //
														//				    bool bIntDisabled = IntMasterDisable();
																		
																		    //
																		    // Determine sys clock divider and realtime clock
																		    //
														//				    switch(ui32SysClockSpeed)
														//				    {
														//				    case SYS_CTRL_250KHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_250KHZ;
														//				        break;
														//				    case SYS_CTRL_500KHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_500KHZ;
														//				        break;
														//				    case SYS_CTRL_1MHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_1MHZ;
														//				        break;
														//				    case SYS_CTRL_2MHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_2MHZ;
														//				        break;
														//				    case SYS_CTRL_4MHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_4MHZ;
														//				        break;
														//				    case SYS_CTRL_8MHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_8MHZ;
														//				        break;
														//				    case SYS_CTRL_16MHZ:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_16MHZ;
														//				        break;
														//				    case SYS_CTRL_32MHZ:
														//				    default:
														//				        ui32SysDiv = SYS_CTRL_SYSDIV_32MHZ;
														//				        break;
														//				    }
																		
																		    //
																		    // Set system clock (no ext 32k osc, no internal osc)
																		    //
																			//if(ui32SysClockSpeed == SYS_CTRL_32MHZ){
																		    		//SysCtrlClockSet(false, false, SYS_CTRL_SYSDIV_32MHZ);
																			//} else if(ui32SysClockSpeed == SYS_CTRL_16MHZ) {
																			//	SysCtrlClockSet(false, false, SYS_CTRL_16MHZ);
																			//} else {
														//						SysCtrlClockSet(false, false, ui32SysDiv);
																			//}
																		
																		    //
																		    // Set IO clock to the same as system clock
																		    //
																		    //SysCtrlIOClockSet(SYS_CTRL_SYSDIV_32MHZ);
														//				    SysCtrlIOClockSet(ui32SysDiv);
																		
																		    //
																		    // LEDs (off, output low)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_ALL);   //#define BSP_LED_BASE     GPIO_C_BASE            //#define BSP_LED_ALL                (BSP_LED_1 | \
																																														                                 BSP_LED_2 | \
																																														                                 BSP_LED_3)     //!< Bitmask of all LEDs
																		
																		    //GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);                                                       ////! Writes a value to the specified pin(s)
																																												//!
																																												//! \param ui32Port is the base address of the GPIO port.
																																												//! \param ui8Pins is the bit-packed representation of the pin(s).
																																												//! \param ui8Val is the value to write to the pin(s).
																																												//!
																																												//! Writes the corresponding bit values to the output pin(s) specified by
																																												//! \e ui8Pins.  Writing to a pin configured as an input pin has no effect.
																																												//!
																																												//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																												//! set identifies the pin to be accessed, and where bit 0 of the byte
																																												//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																												//!
																																												//! \return None
																																												//
																																												//*****************************************************************************
																																												//void
																																												//GPIOPinWrite(uint32_t ui32Port, uint8_t ui8Pins, uint8_t ui8Val)
																																												//{
																																												    //
																																												    // Check the arguments.
																																												    //
																																												//    ASSERT(GPIOBaseValid(ui32Port));
																																												
																																												    //
																																												    // Write the pins.
																																												    //
																																												//    HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = ui8Val;                     
																																												//}                                                                                                                          //#define GPIO_O_DATA             0x00000000  // This is the data register. In 
																																																																													                                            // software control mode, values 
																																																																													                                            // written in the GPIODATA register 
																																																																													                                            // are transferred onto the GPOUT 
																																																																													                                            // pins if the respective pins have 
																																																																													                                            // been configured as outputs 
																																																																													                                            // through the GPIODIR register. A 
																																																																													                                            // read from GPIODATA returns the 
																																																																													                                            // last bit value written if the 
																																																																													                                            // respective pins are configured 
																																																																													                                            // as output, or it returns the 
																																																																													                                            // value on the corresponding input 
																																																																													                                            // GPIN bit when these are 
																																																																													                                            // configured as inputs. 
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
																		   
														//				    bspLedInit();                                                                                       // @brief       This function initializes GPIO pins connected to LEDs. LEDs are
																																												//	*           initialized to be off. The function bspInit() does the same LED
																																												//	*           initialization as this function.
																																												//	*
																																												//	* @return   None
																																												//	******************************************************************************/
																																												//	void
																																												//	bspLedInit(void)
																																												//	{
																																													    //
																																													    // Set GPIO pins as output low
																																													    //
																																												//	    GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_ALL);
																																												//	    GPIOPinWrite(BSP_LED_BASE, BSP_LED_ALL, 0);
																																												//	    bspLedSet(BSP_LED_ALL);                                //
																																												//	}
														
																		    //
																		    // Keys (input pullup)
																		    //
																		    //GPIOPinTypeGPIOInput(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
																		    //IOCPadConfigSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, IOC_OVERRIDE_PUE);
																		    //GPIOPinTypeGPIOInput(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);
																		    //IOCPadConfigSet(BSP_KEY_SEL_BASE, BSP_KEY_SELECT, IOC_OVERRIDE_PUE);
														//				    bspKeyInit(BSP_KEY_MODE_POLL);
																		
																		    //
																		    // Turn off 3.3-V domain (lcd/sdcard power, output low)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_3V3_EN_BASE, BSP_3V3_EN);
																		    //GPIOPinWrite(BSP_3V3_EN_BASE, BSP_3V3_EN, 0);
																		
																		    //
																		    // LCD CSn (output high)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_LCD_CS_BASE, BSP_LCD_CS);
																		    //GPIOPinWrite(BSP_LCD_CS_BASE, BSP_LCD_CS, BSP_LCD_CS);
																		
																		    //
																		    // SD Card reader CSn (output high)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_SDCARD_CS_BASE, BSP_SDCARD_CS);
																		    //GPIOPinWrite(BSP_SDCARD_CS_BASE, BSP_SDCARD_CS, BSP_SDCARD_CS);
																		
																		    //
																		    // Accelerometer (PWR output low, CSn output high)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_ACC_PWR_BASE, BSP_ACC_PWR);
																		    //GPIOPinWrite(BSP_ACC_PWR_BASE, BSP_ACC_PWR, 0);
																		    //GPIOPinTypeGPIOOutput(BSP_ACC_CS_BASE, BSP_ACC_CS);
																		    //GPIOPinWrite(BSP_ACC_CS_BASE, BSP_ACC_CS, BSP_ACC_CS);
																		
																		    //
																		    // Ambient light sensor (off, output low)
																		    //
																		    //GPIOPinTypeGPIOOutput(BSP_ALS_PWR_BASE, BSP_ALS_PWR);
																		    //GPIOPinWrite(BSP_ALS_PWR_BASE, BSP_ALS_PWR, 0);
																		
																		    //
																		    // UART Backchannel (TXD/RXD/CTS/RTS input pullup)
																		    //
														//				    GPIOPinTypeGPIOInput(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD));
														//				    IOCPadConfigSet(BSP_UART_BUS_BASE, (BSP_UART_TXD | BSP_UART_RXD),
														//				                    IOC_OVERRIDE_PUE);
																		    //bspUartOpen(eBaudRate115200);
																		    //bspUartBufInit(uartTX,121,uartRX,121);
																		    //GPIOPinTypeGPIOInput(BSP_UART_CTS_BASE, BSP_UART_CTS);
																		    //IOCPadConfigSet(BSP_UART_CTS_BASE, BSP_UART_CTS, IOC_OVERRIDE_PUE);
																		    //GPIOPinTypeGPIOInput(BSP_UART_RTS_BASE, BSP_UART_RTS);
																		    //IOCPadConfigSet(BSP_UART_RTS_BASE, BSP_UART_RTS, IOC_OVERRIDE_PUE);
																		
																		    //
																		    // Re-enable interrupt if initially enabled.
																		    //
														//				    if(!bIntDisabled)
														//				    {
														//				        IntMasterEnable();
														//				    }
														//				}

	// Initialize keys and key interrupts
	bspKeyInit(BSP_KEY_MODE_ISR);                      ////#define BSP_KEY_MODE_ISR              1          //bspKeyInit() function is given in bsp_key.c 
	                                                                                                                 // @brief    This function initializes key GPIO as input pullup and disables
																													//		*     interrupts. If \e ui8Mode is \b BSP_KEY_MODE_POLL, key presses are
																													//		*     handled using polling and active state debounce. Functions starting
																													//		*     with \b bspKeyInt then do nothing.
																													//		*
																													//		*     If \e ui8Mode is \b BSP_KEY_MODE_ISR, key presses are handled by
																													//		*     interrupts, and debounce is implemented using a timer.
																													//		*
																													//		* @param    ui8Mode is the operation mode; must be one of the following:
																													//		*                   \li \b BSP_KEY_MODE_POLL for polling-based handling
																													//		*                   \li \b BSP_KEY_MODE_ISR for interrupt-based handling
																													//		* @return   None
																													//		******************************************************************************/
																													//		void
																													//		bspKeyInit(uint8_t ui8Mode)
																													//		{
																															    //
																															    // Store mode
																															    //
																													//		    ui8BspKeyMode = ui8Mode;
																															
																															    //
																															    // Initialize keys on GPIO port C (input pullup)
																															    //
																															    //GPIOPinTypeGPIOInput(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);                 //#define BSP_KEY_DIR_BASE        GPIO_C_BASE     //!< Base for left/right/up/down    // #define BSP_KEY_DIR_ALL        (BSP_KEY_LEFT|  \
																																	                                 																																								BSP_KEY_RIGHT| \
																																	                                 																																								BSP_KEY_UP|    \
																																	                                 																																								BSP_KEY_DOWN)  //!< Bitmask of all dir. keys
																																																				        //////! Configures pin(s) for use as GPIO inputs
																																																						//!
																																																						//! \param ui32Port is the base address of the GPIO port.
																																																						//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																						//!
																																																						//! The GPIO pins must be properly configured in order to function correctly as
																																																						//! GPIO inputs.  This function provides the proper configuration for those
																																																						//! pin(s).
																																																						//!
																																																						//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																						//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																						//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																						//!
																																																						//! \return None
																																																						//
																																																						//*****************************************************************************
																																																						//void
																																																						//GPIOPinTypeGPIOInput(uint32_t ui32Port, uint8_t ui8Pins)
																																																						//{
																																																						    //
																																																						    // Check the arguments.
																																																						    //
																																																						 //   ASSERT(GPIOBaseValid(ui32Port));
																																																						
																																																						    //
																																																						    // Make the pin(s) be inputs.
																																																						    //
																																																						 //   GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_IN);                                                              ////! Sets the direction and mode of the specified pin(s)
																																																																																				//!
																																																																																				//! \param ui32Port is the base address of the GPIO port.
																																																																																				//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																																																				//! \param ui32PinIO is the pin direction and/or mode.
																																																																																				//!
																																																																																				//! This function sets the specified pin(s) on the selected GPIO port
																																																																																				//! as either an input or output under software control or sets the
																																																																																				//! pin to be under hardware control.
																																																																																				//!
																																																																																				//! The parameter \e ui32PinIO is an enumerated data type that can be one of
																																																																																				//! the following values:
																																																																																				//!
																																																																																				//! - \b GPIO_DIR_MODE_IN
																																																																																				//! - \b GPIO_DIR_MODE_OUT
																																																																																				//! - \b GPIO_DIR_MODE_HW
																																																																																				//!
																																																																																				//! where \b GPIO_DIR_MODE_IN specifies that the pin will be programmed as
																																																																																				//! a software controlled input, \b GPIO_DIR_MODE_OUT specifies that the pin
																																																																																				//! will be programmed as a software controlled output, and
																																																																																				//! \b GPIO_DIR_MODE_HW specifies that the pin will be placed under
																																																																																				//! hardware control.
																																																																																				//!
																																																																																				//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																																																				//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																																																				//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																																																				//!
																																																																																				//! \return None
																																																																																				//
																																																																																				//*****************************************************************************
																																																																																				//void
																																																																																				//GPIODirModeSet(uint32_t ui32Port, uint8_t ui8Pins,
																																																																																				//               uint32_t ui32PinIO)
																																																																																				//{
																																																																																				
																																																																																				    //
																																																																																				    // Check the arguments.
																																																																																				    //
																																																																																				//    ASSERT(GPIOBaseValid(ui32Port));
																																																																																				 //   ASSERT((ui32PinIO == GPIO_DIR_MODE_IN) || (ui32PinIO == GPIO_DIR_MODE_OUT) ||
																																																																																				 //          (ui32PinIO == GPIO_DIR_MODE_HW));
																																																																																				
																																																																																				    //
																																																																																				    // Set the pin direction and mode.
																																																																																				    //
																																																																																				//    HWREG(ui32Port + GPIO_O_DIR)   = ((ui32PinIO & GPIO_DIR_MODE_OUT) ?                                        
																																																																																				 //                                     (HWREG(ui32Port + GPIO_O_DIR) | ui8Pins) :
																																																																																				 //                                     (HWREG(ui32Port + GPIO_O_DIR) & ~(ui8Pins)));              //#define GPIO_O_DIR              0x00000400       // The DIR register is the data 
																																																																																																												                                                     // direction register. All bits are 
																																																																																																												                                                     // cleared by a reset; therefore, 
																																																																																																												                                                     // the GPIO pins are input by 
																																																																																																												                                                     // default.
																																																																																				 
																																																																																				//    HWREG(ui32Port + GPIO_O_AFSEL) = ((ui32PinIO & GPIO_DIR_MODE_HW) ?                  
																																																																																				//                                      (HWREG(ui32Port + GPIO_O_AFSEL) | ui8Pins) :
																																																																																				//                                      (HWREG(ui32Port + GPIO_O_AFSEL) & ~(ui8Pins)));          
																																																																																				//}                                                                                              //#define GPIO_O_AFSEL            0x00000420       // The AFSEL register is the mode 
																																																																																																													                                                // control select register. Writing 
																																																																																																													                                                // 1 to any bit in this register 
																																																																																																													                                                // selects the hardware 
																																																																																																													                                                // (peripheral) control for the 
																																																																																																													                                                // corresponding GPIO line. All 
																																																																																																													                                                // bits are cleared by a reset, 
																																																																																																													                                                // therefore no GPIO line is set to 
																																																																																																													                                                // hardware control by default. 
																																																						
																																																						    //
																																																						    // Set the pad(s) to no override of the drive type.
																																																						    //
																																																						  //  IOCPadConfigSet(ui32Port, ui8Pins, IOC_OVERRIDE_DIS);
																																																					    //	}
																																    
																								
																																//IOCPadConfigSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, IOC_OVERRIDE_PUE);    //#define IOC_OVERRIDE_PUE  0x00000004    // PAD Config Override Pull Up Enable
																															
																															    //
																															    // Initialize SELECT key on GPIO port A (input pullup)
																															    //
																													//		    GPIOPinTypeGPIOInput(BSP_USER_KEY_BASE, BSP_USER_KEY);                    // #define BSP_USER_KEY_BASE	GPIO_A_BASE	//!< Base for User key                  ///#define BSP_USER_KEY		BSP_KEY_5    
																													//		    IOCPadConfigSet(BSP_USER_KEY_BASE, BSP_USER_KEY, IOC_OVERRIDE_PUE);       //// #define BSP_USER_KEY_BASE	GPIO_A_BASE	//!< Base for User key             ///#define BSP_USER_KEY		BSP_KEY_5            /////#define IOC_OVERRIDE_PUE  0x00000004    // PAD Config Override Pull Up Enable
																															
																															                                                                                        ////////! Set desired drive type on the pad for the desired port pin(s).
																																																					//!
																																																					//! \param ui32Port is the base address of the GPIO port.
																																																					//! \param ui8Pins is the bit-packed representation of the port pin(s).
																																																					//! \param ui32PinDrive is the drive configuration of the desired port
																																																					//! pin.
																																																					//!
																																																					//! This function sets the desired pin drive type for the desired pin(s)
																																																					//! on the selected GPIO port.
																																																					//!
																																																					//! The \e ui32PinDrive parameter controls the configuration of the pin drive on
																																																					//! the pad for the desired pin(s). The parameter is the logical OR of any of
																																																					//! the following:
																																																					//!
																																																					//! - \b IOC_OVERRIDE_OE
																																																					//! - \b IOC_OVERRIDE_PUE
																																																					//! - \b IOC_OVERRIDE_PDE
																																																					//! - \b IOC_OVERRIDE_ANA
																																																					//! - \b IOC_OVERRIDE_DIS
																																																					//!
																																																					//! where \b IOC_OVERRIDE_OE is the output enable bit connected directly
																																																					//! to the output enable pin for the IO driver cell, after it is ORed
																																																					//! with any OE signal from the desired peripheral.  The OE is driven from the
																																																					//! SSI, I2C and GPT peripherals.  \b IOC_OVERRIDE_PUE is the enable bit for
																																																					//! the pull-up.  \b IOC_OVERRIDE_PDE is the enable bit for the pull-down.
																																																					//! \b IOC_OVERRIDE_ANA must be set for the analog signal.
																																																					//!
																																																					//! The pin(s) in \e ui8Pins are specified using a bit-packed byte, where each
																																																					//! bit that is set identifies the pin to be accessed, and where bit 0 of the
																																																					//! byte represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so
																																																					//! on.
																																																					//!
																																																					//! \note PC0 through PC3 are bidirectional high-drive pad-cells. They do not
																																																					//! support on-die pullup or pulldown resistors or analog connectivity.
																																																					//! For these four pins the \e ui32PinDrive parameter must be set to either
																																																					//! \b IOC_OVERRIDE_OE or IOC_OVERRIDE_DIS.
																																																					//!
																																																					//! \return None
																																																					//
																																																					//*****************************************************************************
																																																					//void
																																																					//IOCPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins,
																																																					//                uint32_t ui32PinDrive)
																																																					//{
																																																					//    uint32_t ui32OverrideRegAddr;
																																																					//    uint32_t ui32PinNo;
																																																					//    uint32_t ui32PinBit;
																																																					
																																																					    //
																																																					    // Check the arguments
																																																					    //
																																																					//   ASSERT((ui32Port == GPIO_A_BASE) || (ui32Port == GPIO_B_BASE) ||
																																																					//           (ui32Port == GPIO_C_BASE) || (ui32Port == GPIO_D_BASE));
																																																					//    ASSERT(ui8Pins != 0);
																																																					//    ASSERT((ui32PinDrive == IOC_OVERRIDE_OE)  ||
																																																					//           (ui32PinDrive == IOC_OVERRIDE_PUE) ||
																																																					//           (ui32PinDrive == IOC_OVERRIDE_PDE) ||
																																																					//           (ui32PinDrive == IOC_OVERRIDE_ANA) ||
																																																					//           (ui32PinDrive == IOC_OVERRIDE_DIS));
																																																					    //
																																																					    // PC0-PC3 does not support on-die pullup, pulldown or analog connectivity.
																																																					    //
																																																					//    ASSERT(!((ui32Port == GPIO_C_BASE) && ((ui8Pins & 0xf) > 0) &&
																																																					//             ((ui32PinDrive == IOC_OVERRIDE_PUE) ||
																																																					//              (ui32PinDrive == IOC_OVERRIDE_PDE) ||
																																																					//              (ui32PinDrive == IOC_OVERRIDE_ANA))));
																																																					
																																																					    //
																																																					    // Initialize to default value
																																																					    //
																																																					//    ui32OverrideRegAddr = IOC_PA0_SEL;          //#define IOC_PA0_SEL             0x400D4000  // Peripheral select control for PA0
                                            																																																																		 
																																																					
																																																					    //
																																																					    // Look for specified port pins to be configured, multiple pins are allowed
																																																					    //
																																																					//    for(ui32PinNo = 0; ui32PinNo < 8; ui32PinNo++)
																																																					//    {
																																																					//        ui32PinBit = (ui8Pins >> ui32PinNo) & 0x00000001;
																																																					//        if(ui32PinBit != 0)
																																																					//        {
																																																					            //
																																																					            // Find register addresses for configuring specified port pin
																																																					            //
																																																					//            switch(ui32Port)
																																																					//            {
																																																					//            case GPIO_A_BASE:
																																																					//                ui32OverrideRegAddr = g_pui32IOCPortAOverrideReg[ui32PinNo];
																																																					//                break;
																																																					
																																																					//            case GPIO_B_BASE:
																																																					//                ui32OverrideRegAddr = g_pui32IOCPortBOverrideReg[ui32PinNo];
																																																					//                break;
																																																					
																																																					//            case GPIO_C_BASE:
																																																					//                ui32OverrideRegAddr = g_pui32IOCPortCOverrideReg[ui32PinNo];
																																																					//                break;
																																																					
																																																					//            case GPIO_D_BASE:
																																																					//                ui32OverrideRegAddr = g_pui32IOCPortDOverrideReg[ui32PinNo];
																																																					//                break;
																																																					
																																																					//            default:
																																																					//                // Default to port A pin 0
																																																					//                ui32OverrideRegAddr = IOC_PA0_OVER;          //#define IOC_PA0_OVER            0x400D4080  // This is the overide configuration register for each pad.
																																																			
																																																					//                break;
																																																					//            }
																																																					
																																																					            //
																																																					            // Set desired pin drive for the desired port pin
																																																					            //
																																																					//            HWREG(ui32OverrideRegAddr) = ui32PinDrive;
																																																					//        }
																																																					//    }
																																																					//}
																															                                                                       
																									
																															    //
																															    // Disable interrupts
																															    //
																															    //GPIOPinIntDisable(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);                      ////! Disables interrupts for the specified pin(s)
																																																			//!
																																																			//! \param ui32Port is the base address of the GPIO port.
																																																			//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																			//!
																																																			//! Masks the interrupt for the specified pin(s)
																																																			//!
																																																			//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																			//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																			//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																			//!
																																																			//! \return None
																																																			//
																																																			//*****************************************************************************
																																																			//void
																																																			//GPIOPinIntDisable(uint32_t ui32Port, uint8_t ui8Pins)
																																																			//{
																																																			    //
																																																			    // Check the arguments.
																																																			    //
																																																			 //   ASSERT(GPIOBaseValid(ui32Port));
																																																			
																																																			    //
																																																			    // Disable the interrupts.
																																																			    //
																																																			//    HWREG(ui32Port + GPIO_O_IE) &= ~(ui8Pins);          //#define GPIO_O_IE               0x00000410          // The IE register is the 
																																																																					                                    	// interrupt mask register. Bits 
																																																																					                                        // set to high in IE allow the 
																																																																					                                        // corresponding pins to trigger 
																																																																					                                        // their individual interrupts and 
																																																																					                                        // the combined GPIOINTR line. 
																																																																					                                        // Clearing a bit disables 
																																																																					                                        // interrupt triggering on that 
																																																																					                                        // pin.                     
																																																		//	}
																			
																													//		    GPIOPinIntDisable(BSP_USER_KEY_BASE, BSP_USER_KEY);                          ////// #define BSP_USER_KEY_BASE	GPIO_A_BASE	//!< Base for User key             ///#define BSP_USER_KEY		BSP_KEY_5
																															
																													//		    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
																													//		    {
																															        //
																															        // Connect bspKeyPushedISR() to key pins
																															        //
																															        //ioPinIntRegister(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, &bspKeyDirPushedISR);        //  * @brief    Register an interrupt handler to the GPIO pin (or pins) specified                                                                              
																																																						//				*           by bitmask \e ui8Pins on GPIO port given by \e ui32Base.
																																																						//				* 			This function registers a general ISR to the GPIO port and then
																																																						//				*           assigns the ISR specified by \e pfnIntHandler to the given pins.
																																																						//				*
																																																						//				* @param    ui32Base        is the base address of the GPIO port.
																																																						//				* @param    ui8Pins         is the bit-packed representation of the pin (or
																																																						//				*                           pins).
																																																						//				* @param    pfnIntHandler   is a pointer to the interrupt handler function.
																																																						//				*
																																																						//				* @return   None
																																																						//				******************************************************************************/
																																																						//				void
																																																						//				ioPinIntRegister(uint32_t ui32Base, uint8_t ui8Pins,
																																																						//				                 void (*pfnIntHandler)(void))
																																																						//				{
																																																						//				    volatile uint_fast8_t ui8Cnt;
																																																										
																																																										    //
																																																										    // Disable global interrupts
																																																										    //
																																																						//				    bool bIntDisabled = IntMasterDisable();
																																																										
																																																										    //
																																																										    // If function pointer is not null, register that pin has a custom
																																																										    // handler to the fast access 'ui8PortXPinHasIsr' variables.
																																																										    //
																																																						//				    if(pfnIntHandler)
																																																						//				    {
																																																						//				        switch(ui32Base)
																																																						//				        {
																																																						//				        case GPIO_A_BASE:
																																																						//				            if(!ui8PortAPinHasIsr)
																																																						//				            {
																																																						//				                GPIOPortIntRegister(GPIO_A_BASE, &ioPortAIsr);
																																																						//				            }
																																																						//				            ui8PortAPinHasIsr |=  ui8Pins;
																																																						//				            break;
																																																						//				        case GPIO_B_BASE:
																																																						//				            if(!ui8PortBPinHasIsr)
																																																						//				            {
																																																						//				                GPIOPortIntRegister(GPIO_B_BASE, &ioPortBIsr);
																																																						//				            }
																																																						//				            ui8PortBPinHasIsr |=  ui8Pins;
																																																						//				            break;
																																																						//				        case GPIO_C_BASE:
																																																						//				            if(!ui8PortCPinHasIsr)
																																																						//				            {
																																																						//				                GPIOPortIntRegister(GPIO_C_BASE, &ioPortCIsr);
																																																						//				            }
																																																						//				            ui8PortCPinHasIsr |=  ui8Pins;
																																																						//				            break;
																																																						//				        case GPIO_D_BASE:
																																																						//				            if(!ui8PortDPinHasIsr)
																																																						//				            {
																																																						//				                GPIOPortIntRegister(GPIO_D_BASE, &ioPortDIsr);
																																																						//				            }
																																																						//				            ui8PortDPinHasIsr |=  ui8Pins;
																																																						//				            break;
																																																						//				        }
																																																					    //				    }
																																																										
																																																										    //
																																																										    // Iterate over port pins and store handler into the correct lookup table.
																																																										    //
																																																						//				    for(ui8Cnt = 0; ui8Cnt < 8; ui8Cnt++)
																																																						//				    {
																																																						//				        if(ui8Pins & (1 << ui8Cnt))
																																																						//				        {
																																																						//				            switch(ui32Base)
																																																						//				            {
																																																						//				            case GPIO_A_BASE:
																																																						//				                pfnPortAIsr[ui8Cnt] = pfnIntHandler;
																																																						//				                break;
																																																						//				            case GPIO_B_BASE:
																																																						//				                pfnPortBIsr[ui8Cnt] = pfnIntHandler;
																																																						//				                break;
																																																						//				            case GPIO_C_BASE:
																																																						//				                pfnPortCIsr[ui8Cnt] = pfnIntHandler;
																																																						//				                break;
																																																						//				            case GPIO_D_BASE:
																																																						//				                pfnPortDIsr[ui8Cnt] = pfnIntHandler;
																																																						//				                break;
																																																						//				            default:
																																																						//				                break;
																																																						//				            }
																																																						//				        }
																																																						//				    }
																																																										
																																																										    //
																																																										    // Clear interrupts on specified pins
																																																										    //
																																																						//				    GPIOPinIntClear(ui32Base, ui8Pins);
																																																										
																																																										    //
																																																										    // If interrupt was enabled, re-enable
																																																										    //
																																																						//				    if(!bIntDisabled)
																																																						//				    {
																																																						//				        IntMasterEnable();
																																																						//				    }
																																																						//				}
																													                                                                                                    
																													                                                                                                    
																													                                                                                                    
																													                                                                                                    //@brief    Interrupt Service Routine for an activated directional key.
																																																						//			*           Stores the pin where the interrupt occured, disables the interrupt
																																																						//			*           on that pin and starts the debouncing by use of WDT.
																																						
																																																						//			* @return   None
																																																									//		******************************************************************************/
																																																									//		static void
																																																									//		bspKeyDirPushedISR(void)
																																																									//		{
																																																									//		    register uint_fast8_t ui8IrqBm;
																																																									//		    bool bIntDisabled;
																																																											
																																																											    //
																																																											    // Disable WDT.
																																																											    //
																																																									//		    bspKeyTimerDisable();
																																																											
																																																											    //
																																																											    // Get mask of directional keys from interrupt flag status bitmask.
																																																											    //
																																																									//		    ui8IrqBm = (GPIOPinIntStatus(BSP_KEY_DIR_BASE, true) & BSP_KEY_DIR_ALL);
																																																											
																																																											    //
																																																											    // Critical section: Disable global interrupts, update volatile
																																																											    // variables and re-enable global interrupts.
																																																											    //
																																																									//		    bIntDisabled = IntMasterDisable();
																																																									//		    bspKeysPressed |= ui8IrqBm;
																																																									//		    bspKeyIntDisabledMask |= ui8IrqBm;
																																																									//		    if(!bIntDisabled)
																																																									//		    {
																																																									//		        IntMasterEnable();
																																																									//		    }
																																																											
																																																											    //
																																																											    // Disable interrupts on keys where interrupt flag was set.
																																																											    //
																																																									//		    GPIOPinIntDisable(BSP_KEY_DIR_BASE,
																																																									//		                      (bspKeyIntDisabledMask & BSP_KEY_DIR_ALL));
																																																											
																																																											    //
																																																											    // Run custom ISR if any (unrolled for speed)
																																																											    //
																																																									//		    if((ui8IrqBm & BSP_KEY_LEFT)  && (bspKeysIsrTable[1] != 0))
																																																									//		    {
																																																									//		        (*bspKeysIsrTable[1])();
																																																									//		    }
																																																									//		    if((ui8IrqBm & BSP_KEY_RIGHT) && (bspKeysIsrTable[2] != 0))
																																																									//		    {
																																																									//		        (*bspKeysIsrTable[2])();
																																																									//		    }
																																																									//		    if((ui8IrqBm & BSP_KEY_UP)    && (bspKeysIsrTable[3] != 0))
																																																									//		    {
																																																									//		        (*bspKeysIsrTable[3])();
																																																									//		    }
																																																									//		    if((ui8IrqBm & BSP_KEY_DOWN)  && (bspKeysIsrTable[4] != 0))
																																																									//		    {
																																																									//		        (*bspKeysIsrTable[4])();
																																																									//		    }
																																																											
																																																											    //
																																																											    // Start the debounce timer
																																																											    //
																																																									//		    bspKeyTimerIntRegister(&bspKeyTimerISR);
																																																									//		    bspKeyTimerEnable();
																																																									//		}
																													 
																				
																													//		        ioPinIntRegister(BSP_USER_KEY_BASE, BSP_USER_KEY, &bspKeySelPushedISR);                           //* @brief    Interrupt Service Routine for the select key. The select key is
																																																											//		*           separated from the directional keys because it's located on a
																																																											//		*           different GPIO port.
																																																											//		*
																																																											//		* @see      bspKeyDirPushedISR
																																																											//		*
																																																											//		* @return   None
																																																											//		******************************************************************************/
																																																											//		static void
																																																											//		bspKeySelPushedISR(void)
																																																											//		{
																																																											//		    uint_fast8_t ui8IrqBm;
																																																											//		    bool bIntDisabled;
																																																													
																																																													    //
																																																													    // Disable debounce timer
																																																													    //
																																																											//		    bspKeyTimerDisable();                                                                          //* @brief    Disable debounce timer.
																																																																																					   //	*
																																																																																					   //	* @return   None
																																																																																					   //	******************************************************************************/
																																																																																					   //	static void
																																																																																					   //	bspKeyTimerDisable(void)
																																																																																					   //	{
																																																																																					   //	    HWREG(SMWDTHROSC_WDCTL) &= ~SMWDTHROSC_WDCTL_EN;
																																																																																					   //	}

																																																													    //
																																																													    // Get interrupt flag status from select port
																																																													    //
																																																											//		    ui8IrqBm = (GPIOPinIntStatus(BSP_USER_KEY_BASE, true) & BSP_USER_KEY);                                 ////! Gets interrupt status for the specified GPIO port
																																																																																								//!
																																																																																								//! \param ui32Port is the base address of the GPIO port.
																																																																																								//! \param bMasked specifies whether masked or raw interrupt status is
																																																																																								//! returned.
																																																																																								//!
																																																																																								//! If \e bMasked is set as \b true, then the masked interrupt status is
																																																																																								//! returned; otherwise, the raw interrupt status is returned.
																																																																																								//!
																																																																																								//! \return Returns a bit-packed byte, where each bit that is set identifies
																																																																																								//! an active masked or raw interrupt, and where bit 0 of the byte
																																																																																								//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																																																								//! Bits 31:8 should be ignored.
																																																																																								//
																																																																																								//*****************************************************************************
																																																																																								//uint32_t
																																																																																								//GPIOPinIntStatus(uint32_t ui32Port, bool bMasked)
																																																																																							//	{
																																																																																								    //
																																																																																								    // Check the arguments.
																																																																																								    //
																																																																																							//	    ASSERT(GPIOBaseValid(ui32Port));
																																																																																								
																																																																																								    //
																																																																																								    // Return the interrupt status.
																																																																																								    //
																																																																																							//	    if(bMasked)
																																																																																							//	    {
																																																																																							//	        return(HWREG(ui32Port + GPIO_O_MIS));                        \\#define GPIO_O_MIS              0x00000418  // The MIS register is the masked 
																																																																																																										                                            // interrupt status register. Bits 
																																																																																																										                                            // read high in MIS reflect the 
																																																																																																										                                            // status of input lines triggering 
																																																																																																										                                            // an interrupt. Bits read as low 
																																																																																																										                                            // indicate that either no 
																																																																																																										                                            // interrupt has been generated, or 
																																																																																																										                                            // the interrupt is masked. MIS is 
																																																																																																										                                            // the state of the interrupt after 
																																																																																																										                                            // masking. 
																																																																																							
																																																																																							//	    }
																																																																																							//	    else
																																																																																							//	    {
																																																																																							//	        return(HWREG(ui32Port + GPIO_O_RIS));                        
																																																																																						//		    }
																																																																																							//	}                                                                     \\#define GPIO_O_RIS              0x00000414  // The RIS register is the raw 
																																																																																																			                                            							// interrupt status register. Bits 
																																																																																																			                                            							// read high in RIS reflect the 
																																																																																																			                                            							// status of interrupts trigger 
																																																																																																			                                            							// conditions detected (raw, before 
																																																																																																			                                            							// masking), indicating that all 
																																																																																																			                                            							// the requirements are met, before 
																																																																																																			                                            							// they are finally allowed to 
																																																																																																			                                            							// trigger by IE. Bits read as 0 
																																																																																																			                                            							// indicate that corresponding 
																																																																																																			                                            							// input pins have not initiated an 
																																																																																																			                                            							// interrupt. 
																																																													
																																																													    //
																																																													    // Critical section: Disable global interrupts, update volatile
																																																													    // variables and re-enable global interrupts.
																																																													    //
																																																											//		    bIntDisabled = IntMasterDisable();
																																																											//		    bspKeysPressed |= ui8IrqBm;
																																																											//		    bspKeyIntDisabledMask |= ui8IrqBm;
																																																											//		    if(!bIntDisabled)
																																																											//		    {
																																																											//		        IntMasterEnable();
																																																											//		    }
																																																													
																																																													    //
																																																													    // Disable interrupts on keys where interrupt flag was set.
																																																													    //
																																																											//		    GPIOPinIntDisable(BSP_USER_KEY_BASE, (bspKeyIntDisabledMask & BSP_USER_KEY));
																																																													
																																																													    //
																																																													    // Run custom ISR if any
																																																													    //
																																																											//		    if((ui8IrqBm & BSP_USER_KEY) && (bspKeysIsrTable[0] != 0))
																																																											//		    {
																																																											//		        (*bspKeysIsrTable[0])();
																																																											//		    }
																																																													
																																																													    //
																																																													    // Start debounce timer
																																																													    //
																																																											//		    bspKeyTimerIntRegister(&bspKeyTimerISR);                                                           //
																																																											//		    bspKeyTimerEnable();
																																																											//		}
																													
														
																															
																															        //
																															        // Set trigger type
																															        //
																															        //GPIOIntTypeSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, GPIO_RISING_EDGE);                                  //#define GPIO_RISING_EDGE        0x00000004  // Interrupt on rising edge                              ////! Sets the interrupt type for the specified pin(s)
																																																																																							//!
																																																																																							//! \param ui32Port is the base address of the GPIO port.
																																																																																							//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																																																							//! \param ui32IntType specifies the type of interrupt trigger mechanism.
																																																																																							//!
																																																																																							//! This function sets up the various interrupt trigger mechanisms for the
																																																																																							//! specified pin(s) on the selected GPIO port.
																																																																																							//!
																																																																																							//! The parameter \e ui32IntType is an enumerated data type that can be one of
																																																																																							//! the following values:
																																																																																							//!
																																																																																							//! - \b GPIO_FALLING_EDGE
																																																																																							//! - \b GPIO_RISING_EDGE
																																																																																							//! - \b GPIO_BOTH_EDGES
																																																																																							//! - \b GPIO_LOW_LEVEL
																																																																																							//! - \b GPIO_HIGH_LEVEL
																																																																																							//!
																																																																																							//! where the different values describe the interrupt detection mechanism
																																																																																							//! (edge or level) and the particular triggering event (falling, rising,
																																																																																							//! or both edges for edge detect, low or high for level detect).
																																																																																							//!
																																																																																							//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																																																							//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																																																							//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																																																							//!
																																																																																							//! \note To avoid any spurious interrupts, the user must
																																																																																							//! ensure that the GPIO inputs remain stable for the duration of
																																																																																							//! this function.
																																																																																							//!
																																																																																							//! \return None
																																																																																							//
																																																																																							//*****************************************************************************
																																																																																							//void
																																																																																							//GPIOIntTypeSet(uint32_t ui32Port, uint8_t ui8Pins,
																																																																																							//               uint32_t ui32IntType)
																																																																																						//	{
																																																																																							
																																																																																							    //
																																																																																							    // Check the arguments.
																																																																																							    //
																																																																																						//	    ASSERT(GPIOBaseValid(ui32Port));
																																																																																						//	    ASSERT((ui32IntType == GPIO_FALLING_EDGE) ||
																																																																																						//	           (ui32IntType == GPIO_RISING_EDGE) || (ui32IntType == GPIO_BOTH_EDGES) ||
																																																																																						//	           (ui32IntType == GPIO_LOW_LEVEL)  || (ui32IntType == GPIO_HIGH_LEVEL));
																																																																																							
																																																																																							    //
																																																																																							    // Set the pin interrupt type.
																																																																																							    //
																																																																																						//	    HWREG(ui32Port + GPIO_O_IBE) = ((ui32IntType & 1) ?
																																																																																						//	                                    (HWREG(ui32Port + GPIO_O_IBE) | ui8Pins) :
																																																																																						//	                                    (HWREG(ui32Port + GPIO_O_IBE) & ~(ui8Pins)));                     //#define GPIO_O_IBE              0x00000408       // The IBE register is the 
																																																																																																																		                                            // interrupt both-edges register. 
																																																																																																																		                                            // When the corresponding bit in IS 
																																																																																																																		                                            // is set to detect edges, bits set 
																																																																																																																		                                            // to high in IBE configure the 
																																																																																																																		                                            // corresponding pin to detect both 
																																																																																																																		                                            // rising and falling edges, 
																																																																																																																		                                            // regardless of the corresponding 
																																																																																																																		                                            // bit in the IEV (interrupt event 
																																																																																																																		                                            // register). Clearing a bit 
																																																																																																																		                                            // configures the pin to be 
																																																																																																																		                                            // controlled by IEV.  
																																																																																						//	    HWREG(ui32Port + GPIO_O_IS) = ((ui32IntType & 2) ?
																																																																																						//	                                   (HWREG(ui32Port + GPIO_O_IS) | ui8Pins) :
																																																																																						//	                                   (HWREG(ui32Port + GPIO_O_IS) & ~(ui8Pins)));                      //#define GPIO_O_IS               0x00000404         // The IS register is the 
                                            																																																																																																																	     // interrupt sense register. 
																																																																																					
																																																																																						//	    HWREG(ui32Port + GPIO_O_IEV) = ((ui32IntType & 4) ?
																																																																																						//	                                    (HWREG(ui32Port + GPIO_O_IEV) | ui8Pins) :
																																																																																						//	                                    (HWREG(ui32Port + GPIO_O_IEV) & ~(ui8Pins)));                   
																																																																																						//	}                                                                                                        //#define GPIO_O_IEV              0x0000040C        // The IEV register is the 
																																																																																																																			                                            // interrupt event register. Bits 
																																																																																																																			                                            // set to high in IEV configure the 
																																																																																																																			                                            // corresponding pin to detect 
																																																																																																																			                                            // rising edges or high levels, 
																																																																																																																			                                            // depending on the corresponding 
																																																																																																																			                                            // bit value in IS. Clearing a bit 
																																																																																																																			                                            // configures the pin to detect 
																																																																																																																			                                            // falling edges or low levels, 
																																																																																																																			                                            // depending on the corresponding 
																																																																																																																			                                            // bit value in IS. 
																																																																																			
																													//		        GPIOIntTypeSet(BSP_USER_KEY_BASE, BSP_USER_KEY, GPIO_RISING_EDGE);
																															
																															        //
																															        // Disable timer
																															        //
																													//		        HWREG(SMWDTHROSC_WDCTL) &= ~SMWDTHROSC_WDCTL_EN;
																													//		    }
																													//		}
	
	
	bspKeyIntEnable(BSP_USER_KEY);                              ////#define BSP_USER_KEY		      BSP_KEY_5                      // * @brief    This function enables interrupts on specified key GPIO pins.
																																	//		*
																																	//		* @note     If bspKeyInit() was initialized with argument \b BSP_KEY_MODE_POLL,
																																	//		*           this function does nothing.
																																	//		*
																																	//		* @param    ui8Keys     is an ORed bitmask of keys (for example BSP_KEY_1).
																																	//		*
																																	//		* @return   None
																																	//		******************************************************************************/
																																	//		void
																																	//		bspKeyIntEnable(uint8_t ui8Keys)
																																	//		{
																																	//		    if(ui8BspKeyMode == BSP_KEY_MODE_ISR)
																																	//		    {
																																			        //
																																			        // Enable interrupt for pins on "dir" port:
																																			        //
																																			        //GPIOPinIntEnable(BSP_KEY_DIR_BASE, (ui8Keys & BSP_KEY_DIR_ALL));
																																			
																																			        //
																																			        // Enable interrupt for pins on "select" port:
																																			        //
																																	//		        GPIOPinIntEnable(BSP_KEY_SEL_BASE, (ui8Keys & BSP_KEY_SELECT));
																																	//		    }
																																	//		}        
	UARTlibinit();

		moteConfig.mode = MOTE_MODE_RX;                         ////#define MOTE_MODE_RX                  1
    	moteConfig.channel = CHANNEL;                          ////#define CHANNEL			             0x19
    	moteConfig.txPower = 5;          		               // Index 0. Max output   //              
    	moteConfig.gainMode = MOTE_GAIN_MODE_NONE; 	           // No PA/LNA                             #define MOTE_GAIN_MODE_NONE               42

	// Enable interrupts
    	halIntOn();                                            // A function HAL_INT_ON(); is given in hal_int.c , again a macro is defined in hal_int.h header file  ///#define HAL_INT_ON(x) st( IntMasterEnable(); )

	// Config basicRF
    	basicRfConfig.panId = PAN_ID;                          //#define PAN_ID   0xCDAC     how to set panid is given in hal_rf.c  //void halRfSetPanId(unsigned short panId)
																								                                    //		{
    																									                            //	HWREG(RFCORE_FFSM_PAN_ID0) = LO_UINT16(panId);
    																									                            //   HWREG(RFCORE_FFSM_PAN_ID1) = HI_UINT16(panId);
																											                        //}
																											                        
    	basicRfConfig.ackRequest = false;                     //
	
	receive();

	
	return 0;
}

static void receive(void){

	signed short ssRssi;
	int i;

	basicRfConfig.myAddr = RX_ADDR;                           //#define RX_ADDR      0xCCEB      this RX_ADDR, receiver address we will find at ubimotesez_config.h header file
    	
		
		basicRfConfig.channel = moteConfig.channel;            //#define CHANNEL			        0x19
    	
		
		if(basicRfInit(&basicRfConfig)==FAILED){               // it goes to basic_rf.c file , it Initialise basic RF datastructures. Sets channel, short address and
                                                               //PAN id in the chip and configures interrupt on packet reception.
		
		                                                      //uint8 basicRfInit(basicRfCfg_t* pRfConfig)
																//{
  																//if (halRfInit()==FAILED)                               //   halRfInit() function is used to Power up, sets default tuning settings, enables autoack. it is given in hal_rf.c
    															                                                        //  @return   Returns SUCCESS (for interface compatibility)
																                                                        //  unsigned char halRfInit(void)
																														//	{
																													 //
																														// Some of the below settings are indeed the reset value.
																														//
																																
																														// Enable RF core clocks in active mode (not necessary on CC2538 PG1.0)
																														// HWREG(SYS_CTRL_RCGCRFC) = 1;
																																
																														// Enable auto ack and auto crc
																														// HWREG(RFCORE_XREG_FRMCTRL0) = (HWREG(RFCORE_XREG_FRMCTRL0) | (AUTO_ACK |  \
																																                                  AUTO_CRC));
																																
																														// Recommended RX settings
																														// HWREG(RFCORE_XREG_FRMFILT0) = 0x0D; // Enable frame filtering = 0x0D,
																																                                        // disable = 0x0C
																																  //  HWREG(RFCORE_XREG_AGCCTRL1) = 0x15;                                          \\#define RFCORE_XREG_AGCCTRL1    0x400886C8  // AGC reference level 
																																   // HWREG(RFCORE_XREG_FSCTRL)   = 0x5A;                                          \\#define RFCORE_XREG_FSCTRL      0x400886B0  // Tune frequency synthesizer 
																																//
																																    // Recommended TX settings (only those not already set for RX)
																																//    HWREG(RFCORE_XREG_TXFILTCFG)= 0x09;                                          \\#define RFCORE_XREG_TXFILTCFG   0x400887E8  // TX filter configuration 
																																 //   HWREG(ANA_REGS_O_IVCTRL)    = 0x0B;                                          \\#define ANA_REGS_O_IVCTRL       0x00000004  // Analog control register 
																																 //   HWREG(RFCORE_XREG_FRMCTRL1) = 0x00; // STXON does not affect RXENABLE[6]     \\#define RFCORE_XREG_FRMCTRL1    0x40088628  // Frame handling 
																																 //   HWREG(RFCORE_XREG_MDMTEST1) = 0x08;                                          \\#define RFCORE_XREG_MDMTEST1    0x400886E4  // Test Register for Modem 
																																 //   HWREG(RFCORE_XREG_FSCAL1)   = 0x01;                                          \\#define RFCORE_XREG_FSCAL1      0x400886B8  // Tune frequency calibration 
																																
																																
																																    // Enable random generator
																																    // Not implemented
																																
																																 //   if(halRfEmModule != HAL_RF_CC2538EM)
																																 //   {
																																 //       // Configure PA/LNA
																																 //       halRfPaLnaInit();
																																 //   }                                                                         //  * @brief    This function initializes the CC2538 to control the CC2592 PA/LNA
																																																				//	*           signals. CC2538 GPIO connected to CC2592 HGM is configured as high.
																																																				//	*           CC2538 RX(TX) active status signals are mappe to CC2592 EN(PAEN)
																																																				//	*           signals.
																																																				//	*
																																																				//	* @return   None
																																																				//	******************************************************************************/
																																																				//	static void halRfPaLnaInit(void)
																																																				//	{
																																																				//	    // Configure CC2538 PD2 (CC2592 HGM) as GPIO output
																																																				//	    HWREG(GPIO_D_BASE + GPIO_O_DIR)   |= (0x04);                    //#define GPIO_O_DIR              0x00000400    // The DIR register is the data 
																																																																							                                            // direction register. All bits are 
																																																																							                                            // cleared by a reset; therefore, 
																																																																							                                            // the GPIO pins are input by 
																																																																							                                            // default. 
																																																				
																																																				//	    HWREG(GPIO_D_BASE + GPIO_O_AFSEL) &= (0x04);                  //#define GPIO_O_AFSEL            0x00000420      // The AFSEL register is the mode 
																																																																							                                            // control select register. Writing 
																																																																							                                            // 1 to any bit in this register 
																																																																							                                            // selects the hardware 
																																																																							                                            // (peripheral) control for the 
																																																																							                                            // corresponding GPIO line. All 
																																																																							                                            // bits are cleared by a reset, 
																																																																							                                            // therefore no GPIO line is set to 
																																																																							                                            // hardware control by default. 
																																																		
																																																				//	    HWREG(IOC_PD2_OVER) = 0;                                       //#define IOC_PD2_OVER            0x400D40E8          // This is the overide 
																																																																								                                            // configuration register for each 
																																																																								                                            // pad.               
																																																				
																																																					    // Set CC2592 to HGM
																																																				//	    halRfSetGain(HAL_RF_GAIN_HIGH);
																																																					
																																																					    // Use CC2538 RF status signals to control CC2592 LNAEN and PAEN.
																																																					    // CC2538 PC2 is connected to CC2592 LNAEN
																																																					    // CC2538 PC3 is connected to CC2592 PAEN
																																																				//	    HWREG(RFCORE_XREG_RFC_OBS_CTRL0) = 0x11; // rfc_obs_sig0 = rx_active
																																																				//	    HWREG(CCTEST_OBSSEL2)            = 0x80; // rfc_obs_sig0 => PC2
																																																				//	    HWREG(RFCORE_XREG_RFC_OBS_CTRL1) = 0x10; // rfc_obs_sig1 = tx_active
																																																				//	    HWREG(CCTEST_OBSSEL3)            = 0x81; // rfc_obs_sig1 => PC3
																																																				//	}
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																 
																																
																																    // Set RF interrupt priority to maximum
																																 //   IntPrioritySet(INT_RFCORERTX, 0);
																																
																																    // Register halRfIsr() as RX interrupt function
																																  //  IntRegister(INT_RFCORERTX, &halRfIsr);
																																
																																    // Enable RX interrupt
																																 //   halRfEnableRxInterrupt();
																																
																																 //   return SUCCESS;
																																//}
												
																//	return FAILED;

  																//	halIntOff();

  																	// Set the protocol configuration
  																//	pConfig = pRfConfig;
  																//	rxi.pPayload   = NULL;

  																//	txState.receiveOn = TRUE;
  																//	txState.frameCounter = 0;

  																	// Set channel
  																//	halRfSetChannel(pConfig->channel);

  																	// Write the short address and the PAN ID to the CC2520 RAM
  																//	halRfSetShortAddr(pConfig->myAddr);
  																//	halRfSetPanId(pConfig->panId);

  																	// if security is enabled, write key and nonce
																//	#ifdef SECURITY_CCM
  																//	basicRfSecurityInit(pConfig);
																//	#endif

  																	// Set up receive interrupt (received data or acknowlegment)
  																//	halRfRxInterruptConfig(basicRfRxFrmDoneIsr);
  																	//halRfRxInterruptConfig(snifferRxFrmDoneIsr);
  																//	halIntOn();

  																//	return SUCCESS;
																//	}
	
		UARTprintf("rf init failed\n\r");
		while(1);
    	}
	
	if(moteConfig.gainMode != MOTE_GAIN_MODE_NONE){               //#define MOTE_GAIN_MODE_NONE      42

        	// Set gain mode
        	halRfSetGain(moteConfig.gainMode);                   // first we will go to halRfsetGain in  hal_rf.c file , Function sets the gain mode. Only applicable for units with
																//	CC2590/91. This function assumes that CC2538 GPIO pins have been
																//	configured as GPIO output, for example, by using halRfInit();
																
																//  void halRfSetGain(unsigned char gainMode)
																//	{
																//	    if (gainMode == HAL_RF_GAIN_LOW)                              //#define HAL_RF_GAIN_LOW        0
    															//		{
       															// Setting low gain mode
     															//	HAL_PA_LNA_RX_LGM();                                             //#define HAL_PA_LNA_RX_LGM()         st(HWREG(GPIO_D_BASE + (GPIO_O_DATA +     \
                                                                                                                                                                                                (0x04 << 2))) = 0;)
     															//		   if(halRfEmModule == HAL_RF_CC2538_CC2592EM)
     															//		   {
           														// rssiOffset = RSSI_OFFSET_LNA_CC2592_LOWGAIN;
     															//		   }
    															//	    else if(halRfEmModule == HAL_RF_CC2538_CC2591EM)
    															//    {
          														//	  rssiOffset = RSSI_OFFSET_LNA_CC2591_LOWGAIN;
    															//	    }
 																//	   }
  																//	  else
																//	    {
   																//	     // Setting high gain mode
    															//    HAL_PA_LNA_RX_HGM();
															    //    if(halRfEmModule == HAL_RF_CC2538_CC2592EM)
															    //    {
															     //       rssiOffset = RSSI_OFFSET_LNA_CC2592_HIGHGAIN;
															    //    }
															   //     else if(halRfEmModule == HAL_RF_CC2538_CC2591EM)
															    //    {
															     //       rssiOffset = RSSI_OFFSET_LNA_CC2591_HIGHGAIN;
															     //   }
															   // }
																//	}		
    															//	}

	// Start RX
	basicRfReceiveOn();                                         // Turns on receiver on radio
																// void basicRfReceiveOn(void)
																//	{
  															    //	txState.receiveOn = TRUE;
  															    //	halRfReceiveOn();      // this halRfReceiveOn() function is used to turn receiver on & can be given as 
  															                               //void halRfReceiveOn(void)
																						  //{
   																						// Make sure the RX FIFO is empty and enter RX
  																						//ISFLUSHRX();       // this ISFLUSHRX() is a macro which is given in hal_rf.c & whose value is given as #define ISFLUSHRX()  st(HWREG(RFST) = 0x000000ED;)
   																						//ISRXON();          // this ISRXON() is a macro which is given in hal_rf.c whose value is given as #define ISRXON()  st(HWREG(RFST) = 0x000000E3;)
																						//	}
																//	}
	while(1) {
        	
		while(!basicRfPacketIsReady());                        // basicRfPacketIsReady() this function is used to Check if a new packet is ready to be read by next higher layer
        													   //uint8 basicRfPacketIsReady(void)
															   //	{
  															   //		return rxi.isReady;       // rxi.isReady is fetched from structure basicRfRxInfo_t; given in basic_rf.c
															   //		}
		
		
			if(basicRfReceive((unsigned char*)payload, 103, &ssRssi) > 0) {          //basicRfReceive() function Copies the payload of the last incoming packet into a buffer
            		                                                                //Copies the payload of the last incoming packet into a buffer

																					//	* @param    pRxData     Pointer to data buffer to fill. This buffer must be
																					//	*                       allocated by higher layer.
																					//	* @param    len         Number of bytes to read in to buffer
																					//	* @param    pRssi       Pointer to variable holding packet RSSI. NULL if RSSI
																					//	*                       is not to be stored.
																					//	*           rxi         File scope variable holding the information of the last
																					//	*                       incoming packet
																					//	*
																					//	* @return   uint8 - Number of bytes actually copied into buffer
																					//	******************************************************************************/
																					//	uint8 basicRfReceive(uint8* pRxData, uint8 len, int16* pRssi)
																					//	{
																					//	  uint8 chunkSize, i;
																						
																					//	  halIntOff();                  // void halIntOff(void)
																														//	{
   																														//		 HAL_INT_OFF();
																														//	}
																						
																						  // Critical region start
																						  // TODO: Copy data using DMA?
																					//	  chunkSize = MIN(rxi.length, len);   // length is a member of the structure basicRfRxInfo_t given in basic_rf.h
																					//	  for(i = 0; i < chunkSize; i++) {
																				    //	  *pRxData++ = rxi.pPayload[i];       // payload is a member of the structure basicRfRxInfo_t given in basic_rf.h
																					//	  }
																						
																					//	  if(pRssi != NULL) {
																					//	    *pRssi = rxi.rssi - halRfGetRssiOffset();
																					//	  }
																					//	  rxi.isReady = FALSE;
																						
																					//	  // Critical region end
																					//	  halIntOn();          // Enable global interrupts.
																					                           //void halIntOn(void)
																												//	{
    																											//	HAL_INT_ON();
																												//		}

																					//	  return chunkSize;
																					//	}
            		
            		
			bspLedToggle(BSP_LED_ALL);                                              // This function toggles LED(s) specified by \e ui8Leds. The function
																					//	*           assumes that LED pins have been initialized by, for example,
																					//	*           bspLedInit().
																					// @param    ui8Leds      ORed bitmask of LEDs (for example \b BSP_LED_1).
																					//	*
																					//	* @return   None
																					
																					        //void
																							//bspLedToggle(uint8_t ui8Leds)
																							//{
																							    //
																							    // Get current pin values of selected bits
																							    //
																							 //   uint32_t ui32Toggle = GPIOPinRead(BSP_LED_BASE, ui8Leds);                                  ////! Reads the values present of the specified pin(s)
																																																//!
																																																//! \param ui32Port is the base address of the GPIO port.
																																																//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																//!
																																																//! The values at the specified pin(s) are read, as specified by \e ui8Pins.
																																																//! Values are returned for both input and output pin(s), and the value
																																																//! for pin(s) that are not specified by \e ui8Pins are set to 0.
																																																//!
																																																//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																//!
																																																//! \return Returns a bit-packed byte providing the state of the specified
																																																//! pin, where bit 0 of the byte represents GPIO port pin 0, bit 1 represents
																																																//! GPIO port pin 1, and so on.  Any bit that is not specified by \e ui8Pins
																																																//! is returned as a 0.  Bits 31:8 should be ignored.
																																																//
																																																//*****************************************************************************
																																																//uint32_t
																																																//GPIOPinRead(uint32_t ui32Port, uint8_t ui8Pins)
																																																//{
																																																    //
																																																    // Check the arguments.
																																																    //
																																															//	    ASSERT(GPIOBaseValid(ui32Port));
																																																
																																																    //
																																																    // Return the pin value(s).
																																																    //
																																															//	    return(HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))));                  //#define GPIO_O_DATA             0x00000000  // This is the data register. In 
																																																																					                                            // software control mode, values 
																																																																					                                            // written in the GPIODATA register 
																																																																					                                            // are transferred onto the GPOUT 
																																																																					                                            // pins if the respective pins have 
																																																																					                                            // been configured as outputs 
																																																																					                                            // through the GPIODIR register. A 
																																																																					                                            // read from GPIODATA returns the 
																																																																					                                            // last bit value written if the 
																																																																					                                            // respective pins are configured 
																																																																					                                            // as output, or it returns the 
																																																																					                                            // value on the corresponding input 
																																																																					                                            // GPIN bit when these are 
																																																																					                                            // configured as inputs. 
																																													
																																															//	}
																							    //
																							    // Invert selected bits
																							    //
																							 //   ui32Toggle = (~ui32Toggle) & ui8Leds;
																							
																							    //
																							    // Set GPIO
																							    //
																							 //   GPIOPinWrite(BSP_LED_BASE, ui8Leds, ui32Toggle);                                                                                              ////! Writes a value to the specified pin(s)
																																																												//!
																																																												//! \param ui32Port is the base address of the GPIO port.
																																																												//! \param ui8Pins is the bit-packed representation of the pin(s).
																																																												//! \param ui8Val is the value to write to the pin(s).
																																																												//!
																																																												//! Writes the corresponding bit values to the output pin(s) specified by
																																																												//! \e ui8Pins.  Writing to a pin configured as an input pin has no effect.
																																																												//!
																																																												//! The pin(s) are specified using a bit-packed byte, where each bit that is
																																																												//! set identifies the pin to be accessed, and where bit 0 of the byte
																																																												//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																																																												//!
																																																												//! \return None
																																																												//
																																																												//*****************************************************************************
																																																												//void
																																																												//GPIOPinWrite(uint32_t ui32Port, uint8_t ui8Pins, uint8_t ui8Val)
																																																												//{
																																																												    //
																																																												    // Check the arguments.
																																																												    //
																																																												//    ASSERT(GPIOBaseValid(ui32Port));
																																																												
																																																												    //
																																																												    // Write the pins.
																																																												    //
																																																												//    HWREG(ui32Port + (GPIO_O_DATA + (ui8Pins << 2))) = ui8Val;
																																																												//}

																							//}
																
			for(i=1;i<payload[0];i++){
					UARTprintf("%c", payload[i]);
			}
			UARTprintf("\n\r");
        	}
    	}
}

static void UARTlibinit(void){

    	// Map UART signals to the correct GPIO pins and configure them as
    	// hardware controlled.
    	
    	//To Configure the device for external operation with a specific peripheral , the chosen input & 
		//output should be routed to and from the peripheral by configuring the muxing matrix using the IOCPinConfigPeriphOutput() & IOCPinConfigPeriphInput()
    	
    	IOCPinConfigPeriphOutput(GPIO_A_BASE, GPIO_PIN_1,                                                
                             IOC_MUX_OUT_SEL_UART0_TXD);    // command you will find in ioc.c            /// //! Mux desired on-chip peripheral output signal to the desired port pin(s).
																										//!
																										//! \param ui32Port is the base address of the GPIO port.
																										//! \param ui8Pins is the bit-packed representation of the port pin(s).
																										//! \param ui32OutputSignal is the desired peripheral output signal to drive the
																										//! desired port pin(s).
																										//!
																										//! This function routes the desired on-chip peripheral signal to the
																										//! desired pin(s) on the selected GPIO port.  Functions are available within
																										//! the GPIO device driver that can set the peripheral signal to be under
																										//! hardware control. The IOCPadConfigSet() function can be used to set the pin
																										//! drive type on the desired port pin.
																											//!
																										//! The \e ui32OutputSignal parameter is an enumerated type that controls which
																										//! peripheral output signal to route to the desired port pin(s).
																										//! This parameter can have any of the following values:
																										//!
																										//! - \b IOC_MUX_OUT_SEL_UART0_TXD     //#define IOC_MUX_OUT_SEL_UART0_TXD    0x00000000  // iuarttxd_uart0
																										//! - \b IOC_MUX_OUT_SEL_UART1_RTS
																										//! - \b IOC_MUX_OUT_SEL_UART1_TXD
																										//! - \b IOC_MUX_OUT_SEL_SSI0_TXD
																										//! - \b IOC_MUX_OUT_SEL_SSI0_CLKOUT
																										//! - \b IOC_MUX_OUT_SEL_SSI0_FSSOUT
																										//! - \b IOC_MUX_OUT_SEL_SSI0_STXSER_EN
																										//! - \b IOC_MUX_OUT_SEL_SSI1_TXD
																										//! - \b IOC_MUX_OUT_SEL_SSI1_CLKOUT
																										//! - \b IOC_MUX_OUT_SEL_SSI1_FSSOUT
																										//! - \b IOC_MUX_OUT_SEL_SSI1_STXSER_EN
																										//! - \b IOC_MUX_OUT_SEL_I2C_CMSSDA
																										//! - \b IOC_MUX_OUT_SEL_I2C_CMSSCL
																										//! - \b IOC_MUX_OUT_SEL_GPT0_ICP1
																										//! - \b IOC_MUX_OUT_SEL_GPT0_ICP2
																										//! - \b IOC_MUX_OUT_SEL_GPT1_ICP1
																										//! - \b IOC_MUX_OUT_SEL_GPT1_ICP2
																										//! - \b IOC_MUX_OUT_SEL_GPT2_ICP1
																										//! - \b IOC_MUX_OUT_SEL_GPT2_ICP2
																										//! - \b IOC_MUX_OUT_SEL_GPT3_ICP1
																										//! - \b IOC_MUX_OUT_SEL_GPT3_ICP2
																										//!
																										//! The pin(s) in \e ui8Pins are specified using a bit-packed byte, where each
																										//! bit that is set identifies the pin to be accessed, and where bit 0 of the
																										//! byte represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so
																										//! on.
																										//! 
																										//! \return None
																										//
																										//*****************************************************************************
																										//void
																										//IOCPinConfigPeriphOutput(uint32_t ui32Port, uint8_t ui8Pins,
																									    //                         uint32_t ui32OutputSignal)
																										//{
																										//    uint32_t ui32PortAddr;
																										//    uint32_t ui32PinNo;
																										//    uint32_t ui32PinBit;
																																
																										//
																										// Check the arguments
																										//
																									    //    ASSERT((ui32Port == GPIO_A_BASE) || (ui32Port == GPIO_B_BASE) ||                 //
																									    //           (ui32Port == GPIO_C_BASE) || (ui32Port == GPIO_D_BASE));
																										//    ASSERT(ui8Pins != 0);
																																
																										//
																										// Initialize to default value
																										//
																										//    ui32PortAddr = IOC_PA0_SEL;
																																
																										 //
																										// Look for specified port pins to be configured, multiple pins are allowed
																										//
																										 //for(ui32PinNo = 0; ui32PinNo < 8; ui32PinNo++)
																										// {
																										 //     ui32PinBit = (ui8Pins >> ui32PinNo) & 0x00000001;
																										 //    if(ui32PinBit != 0)
																										 //    {
																										 //
																										// Find register addresses for configuring specified port pin
																										//
																										//        switch(ui32Port)
																										//        {
																										//        case GPIO_A_BASE:
																										//           ui32PortAddr = g_pui32IOCPortASignSelectReg[ui32PinNo];
																										//           break;
																																
																										//        case GPIO_B_BASE:
																									    //            ui32PortAddr = g_pui32IOCPortBSignSelectReg[ui32PinNo];
																										//            break;
																																
																										//        case GPIO_C_BASE:
																									    //            ui32PortAddr = g_pui32IOCPortCSignSelectReg[ui32PinNo];
																										//            break;
																																
																										//         case GPIO_D_BASE:
																										//           ui32PortAddr = g_pui32IOCPortDSignSelectReg[ui32PinNo];
																										//            break;
																																
																										//        default:
																									    // Default to port A pin 0
																										//            ui32PortAddr = IOC_PA0_SEL;
																										//            break;
																										//        }
																																
																										//
																										// Set the mux for the desired port pin to select the desired
																										// peripheral output signal
																										//
																										//        HWREG(ui32PortAddr) = ui32OutputSignal;
																									    //   }
																										//}
																										//}
                             
                             
    	GPIOPinTypeUARTOutput(GPIO_A_BASE, GPIO_PIN_1);            // command you will find in gpio.c   ////! Configures output pin(s) for use by the UART peripheral
																									    //!
																										//! \param ui32Port is the base address of the GPIO port.
																										//! \param ui8Pins is the bit-packed representation of the pin(s).
																										//!
																										//! The UART output pins must be properly configured for the UART peripheral to
																										//! function correctly.  This function provides a typical configuration for
																										//! those pin(s); other configurations might work as well depending upon the
																										//! board setup (for example, using the on-chip pull-ups).
																										//!
																										//! The pin(s) are specified using a bit-packed byte, where each bit that is
																										//! set identifies the pin to be accessed, and where bit 0 of the byte
																										//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																										//!
																										//! \note This function cannot be used to turn any pin into a UART pin; but only
																										//! configures a UART pin for proper operation.
																										//!
																										//! \return None
																										//
																										//*****************************************************************************
																										//void
																										//GPIOPinTypeUARTOutput(uint32_t ui32Port, uint8_t ui8Pins)
																										//{
																										//
																										 // Check the arguments.
																										//
																										//ASSERT(GPIOBaseValid(ui32Port));
																										//ASSERT(!((ui32Port == GPIO_C_BASE) && ((ui8Pins & 0xf) > 0)));
																																
																										//
																										// Make the pin(s) be peripheral controlled.
																										 //
																										//GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_HW);
																																
																										//
																										 // Set the pad(s) to output enable.
																										 //
																										 //IOCPadConfigSet(ui32Port, ui8Pins, IOC_OVERRIDE_OE);
																										//}
    	
        	
			
			
			
			IOCPinConfigPeriphInput(GPIO_A_BASE, GPIO_PIN_0, 
                            IOC_UARTRXD_UART0);               // command you will find in ioc.c                 //  //! Mux the desired port pin to the desired on-chip peripheral input signal
																												//!
																												//! \param ui32Port is the base address of the GPIO port.
																												//! \param ui8Pin is the bit-packed representation of the desired port pin.
																												//! \param ui32PinSelectReg is the address of the IOC mux-register for the
																												//! desired peripheral input signal to which the desired port pin shall be
																												//! routed.
																												//!
																												//! This function routes the desired port pin to the desired on-chip
																												//! peripheral input signal.  Functions are available within the GPIO device
																												//! driver that set the peripheral signal to be under hardware control and
																												//! configures the pin drive type on the desired port pin.
																												//!
																												//! The parameter \e ui32PinSelectReg is an enumerated data type that can be one
																												//! of the following values:
																												//!
																												//! - \b IOC_UARTRXD_UART0
																												//! - \b IOC_UARTCTS_UART1
																												//! - \b IOC_UARTRXD_UART1
																												//! - \b IOC_CLK_SSI_SSI0
																												//! - \b IOC_SSIRXD_SSI0
																												//! - \b IOC_SSIFSSIN_SSI0
																												//! - \b IOC_CLK_SSIIN_SSI0
																												//! - \b IOC_CLK_SSI_SSI1
																												//! - \b IOC_SSIRXD_SSI1
																												//! - \b IOC_SSIFSSIN_SSI1
																												//! - \b IOC_CLK_SSIIN_SSI1
																												//! - \b IOC_I2CMSSDA
																												//! - \b IOC_I2CMSSCL
																												//! - \b IOC_GPT0OCP1
																												//! - \b IOC_GPT0OCP2
																												//! - \b IOC_GPT1OCP1
																												//! - \b IOC_GPT1OCP2
																												//! - \b IOC_GPT2OCP1
																												//! - \b IOC_GPT2OCP2
																												//! - \b IOC_GPT3OCP1
																												//! - \b IOC_GPT3OCP2
																												//!
																												//! The pin in ui8Pin is specified using a bit-packed byte, where the bit that
																												//! is set identifies the pin to be accessed, and where bit 0 of the byte
																												//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																												//!
																												//! \return None
																												//
																												//*****************************************************************************
																												//void
																												//IOCPinConfigPeriphInput(uint32_t ui32Port, uint8_t ui8Pin,
																												//                        uint32_t ui32PinSelectReg)
																												//{
																												 //   uint32_t ui32PortPin;
																												
																												    //
																												    //Set initial values
																												    //
																												//    ui32PortPin = IOC_PAD_IN_SEL_PA0;
																												
																												    //
																												    // Check the arguments
																												    //
																												    //ASSERT((ui32Port == GPIO_A_BASE) || (ui32Port == GPIO_B_BASE) ||
																												//           (ui32Port == GPIO_C_BASE) || (ui32Port == GPIO_D_BASE));
																												    //ASSERT((ui8Pin == GPIO_PIN_0) || (ui8Pin == GPIO_PIN_1) ||
																												    //       (ui8Pin == GPIO_PIN_2) || (ui8Pin == GPIO_PIN_3) ||
																												    //       (ui8Pin == GPIO_PIN_4) || (ui8Pin == GPIO_PIN_5) ||
																												    //       (ui8Pin == GPIO_PIN_6) || (ui8Pin == GPIO_PIN_7));
																												    //ASSERT((ui32PinSelectReg == IOC_UARTRXD_UART0)  ||
																												    //       (ui32PinSelectReg == IOC_UARTCTS_UART1)  ||
																												    //       (ui32PinSelectReg == IOC_UARTRXD_UART1)  ||
																												    //       (ui32PinSelectReg == IOC_CLK_SSI_SSI0)   ||
																												    //       (ui32PinSelectReg == IOC_SSIRXD_SSI0)    ||
																												    //       (ui32PinSelectReg == IOC_SSIFSSIN_SSI0)  ||
																												    //       (ui32PinSelectReg == IOC_CLK_SSIIN_SSI0) ||
																												    //       (ui32PinSelectReg == IOC_CLK_SSI_SSI1)   ||
																												    //       (ui32PinSelectReg == IOC_SSIRXD_SSI1)    ||
																												    //       (ui32PinSelectReg == IOC_SSIFSSIN_SSI1)  ||
																												    //       (ui32PinSelectReg == IOC_CLK_SSIIN_SSI1) ||
																												    //       (ui32PinSelectReg == IOC_I2CMSSDA)       ||
																												    //       (ui32PinSelectReg == IOC_I2CMSSCL)       ||
																												    //       (ui32PinSelectReg == IOC_GPT0OCP1)       ||
																												    //       (ui32PinSelectReg == IOC_GPT0OCP2)       ||
																												    //       (ui32PinSelectReg == IOC_GPT1OCP1)       ||
																												    //       (ui32PinSelectReg == IOC_GPT1OCP2)       ||
																												    //       (ui32PinSelectReg == IOC_GPT2OCP1)       ||
																												    //       (ui32PinSelectReg == IOC_GPT2OCP2)       ||
																												    //       (ui32PinSelectReg == IOC_GPT3OCP1)       ||
																												    //      (ui32PinSelectReg == IOC_GPT3OCP2));
																												
																												    //switch(ui32Port)
																												   // {
																												    //case GPIO_A_BASE:
																												    //    if(ui8Pin == GPIO_PIN_0)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA0;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_1)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA1;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_2)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA2;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_3)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA3;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_4)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA4;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_5)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA5;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_6)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA6;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_7)
																												    //   {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PA7;
																												    //    }
																												    //     break;
																												
																												    //case GPIO_B_BASE:
																												    //    if(ui8Pin == GPIO_PIN_0)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB0;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_1)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB1;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_2)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB2;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_3)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB3;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_4)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB4;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_5)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB5;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_6)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB6;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_7)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PB7;
																												    //    }
																												    //    break;
																												
																												    //case GPIO_C_BASE:
																												    //    if(ui8Pin == GPIO_PIN_0)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC0;
																												   //     }
																												   //     if(ui8Pin == GPIO_PIN_1)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC1;
																												     //   }
																												    //   if(ui8Pin == GPIO_PIN_2)
																												     //   {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC2;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_3)
																												     //   {
																												     //       ui32PortPin = IOC_PAD_IN_SEL_PC3;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_4)
																												   //     {
																												   //         ui32PortPin = IOC_PAD_IN_SEL_PC4;
																												   ///     }
																												    //    if(ui8Pin == GPIO_PIN_5)
																												   //     {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC5;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_6)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC6;
																												     //   }
																												     //   if(ui8Pin == GPIO_PIN_7)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PC7;
																												    //    }
																												    //    break;
																												
																												    //case GPIO_D_BASE:
																												    //    if(ui8Pin == GPIO_PIN_0)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD0;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_1)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD1;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_2)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD2;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_3)
																												    //    {
																												    //       ui32PortPin = IOC_PAD_IN_SEL_PD3;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_4)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD4;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_5)
																												    //    {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD5;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_6)
																												     //   {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD6;
																												    //    }
																												    //    if(ui8Pin == GPIO_PIN_7)
																												   //     {
																												    //        ui32PortPin = IOC_PAD_IN_SEL_PD7;
																												    //    }
																												    //    break;
																												
																												    //default:
																												        // Default to port A pin 0
																												    //    ui32PortPin = IOC_PAD_IN_SEL_PA0;
																												    //    break;
																												   // }
																												
																												    //
																												    // Set the mux for the desired peripheral inputsignal to select the
																												    // the desired port pin.
																												    //
																												   // HWREG(ui32PinSelectReg) = ui32PortPin;
																											//	}                                                       
                            
                            
                            
                            
                            
    	GPIOPinTypeUARTInput(GPIO_A_BASE, GPIO_PIN_0);        //command you will find in gpio.c                     ////! Configures input pin(s) for use by the UART peripheral
																													//!
																													//! \param ui32Port is the base address of the GPIO port.
																													//! \param ui8Pins is the bit-packed representation of the pin(s).
																													//!
																													//! The UART input pins must be properly configured for the UART peripheral to
																													//! function correctly.  This function provides a typical configuration for
																													//! those pin(s); other configurations might work as well depending upon the
																													//! board setup (for example, using the on-chip pull-ups).
																													//!
																													//! \note For PC0 through PC3 the function GPIOPinTypeUARTHiDrive() should
																													//! be used to configure these high drive pins.
																													//!
																													//! The pin(s) are specified using a bit-packed byte, where each bit that is
																													//! set identifies the pin to be accessed, and where bit 0 of the byte
																													//! represents GPIO port pin 0, bit 1 represents GPIO port pin 1, and so on.
																													//!
																													//! \note This function cannot be used to turn any pin into a UART pin; but only
																													//! configures a UART pin for proper operation.
																													//!
																													//! \return None
																													//
																													//*****************************************************************************
																													//void
																													//GPIOPinTypeUARTInput(uint32_t ui32Port, uint8_t ui8Pins)
																													//{
																													    //
																													    // Check the arguments.
																													    //
																													    //ASSERT(GPIOBaseValid(ui32Port));
																													    //ASSERT(!((ui32Port == GPIO_C_BASE) && ((ui8Pins & 0xf) > 0)));
																													
																													    //
																													    // Make the pin(s) be peripheral controlled.
																													    //
																													    //GPIODirModeSet(ui32Port, ui8Pins, GPIO_DIR_MODE_HW);
																													
																													    //
																													    // Set the pad(s) to override disable.
																													    //
																													    //IOCPadConfigSet(ui32Port, ui8Pins, IOC_OVERRIDE_DIS);
																													//}
    	
    
    	// Initialize the UART (UART0) for console I/O.
    	UARTStdioInit(0);
}
