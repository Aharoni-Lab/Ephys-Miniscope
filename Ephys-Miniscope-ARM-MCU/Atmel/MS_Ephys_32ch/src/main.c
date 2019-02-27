/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
//RHD2132 Reg and Commands Definitions
#define CALIBRATE							0b0101010100000000 //Inits self cal routine
#define CLEAR								0b0110101000000000 //Clears on chip cal parameters
#define REG_ADC_CONFIG_AND_FAST_SETTLE		0
#define REG_SUPPLY_SENSOR_AND_BIAS_CURRENT	1
#define REG_MUX_BIAS_CURRENT				2
#define REG_MUX_LOAD						3
#define REG_ADC_OUTPUT_FORMAT_DSP_OFFSET	4
#define REG_IMPEDANCE_CHECK_CONTROL			5
#define REG_IMPEDANCE_CHECK_DAC				6
#define REG_IMPEDANCE_CHECK_AMP_SELECT		7
#define REG_AMP_BANDWIDTH_1					8
#define REG_AMP_BANDWIDTH_2					9
#define REG_AMP_BANDWIDTH_3					10
#define REG_AMP_BANDWIDTH_4					11
#define REG_AMP_BANDWIDTH_5					12
#define REG_AMP_BANDWIDTH_6					13
#define REG_AMP_POWER_1						14
#define REG_AMP_POWER_2						15
#define REG_AMP_POWER_3						16
#define REG_AMP_POWER_4						17

// -------- GPIO Definitions
#define LED_PIN				PIO_PA28_IDX
#define LED_PORT			ID_PIOA

#define LINEVALID_PIN		PIO_PD0_IDX
#define LINEVALID_MASK		(1u << 0) 
#define LINEVALID_PORT		ID_PIOD

// ------- SSC Definitions
#define TD_PIN				PIO_PD10_IDX
#define TD_MODE				IOPORT_MODE_MUX_C
#define TF_PIN				PIO_PB0_IDX
#define TF_MODE				IOPORT_MODE_MUX_D
#define TK_PIN				PIO_PB1_IDX
#define TK_MODE				IOPORT_MODE_MUX_D

// ------- SSC DMA
#define SSC_XDMAC_TX_CH_NUM	32 //Actual DMA channel ID for SSC
#define SSC_XDMAC_TX_CH		2 //Specific DMA channel used for this communication (0 and 1 are taken by SPI0)
static xdmac_channel_config_t ssc_xdmac_tx_cfg; //XDMAC channel configuration.

#define SSC_DMA_OFF			0
#define SSC_DMA_ON			1

// ------- TWI Definitions
#define MCU_TWI_ADDRESS		0x40
#define MCU_TWI_MEMORY_SIZE	512
#define TWCK_PIN			PIO_PB5_IDX
#define TWCK_MODE			IOPORT_MODE_MUX_A
#define TWD_PIN				PIO_PB4_IDX
#define TWD_MODE			IOPORT_MODE_MUX_A

// ------- SPI Definitions
#define SPI_MASTER_BASE     SPI0
#define SPI_IRQn			SPI0_IRQn
#define SPI_ID				ID_SPI0

#define SPI_CHIP_SEL		0
#define SPI_CHIP_PCS		spi_get_pcs(SPI_CHIP_SEL)
#define SPI_CLK_POLARITY	0
#define SPI_CLK_NPHASE		1
#define SPI_DLYBS			0x0 //Delay between NPCS activation to first SPCK (0 is half clock cycle)
#define SPI_DLYBCT			0x01 //Delay between consecutive transfers. A delay before CS is pulled up at end of transfer
#define SPI_DLYBCS			0x10 // Delay NPCS is held high before next transmit
#define SPI_CLK_RATE		16000000//24000000

#define SPI_MISO_PIN		PIO_PD20_IDX
#define SPI_MISO_MODE		IOPORT_MODE_MUX_B
#define SPI_MOSI_PIN		PIO_PD21_IDX
#define SPI_MOSI_MODE		IOPORT_MODE_MUX_B	
#define SPI_SPCK_PIN		PIO_PD22_IDX
#define SPI_SPCK_MODE		IOPORT_MODE_MUX_B
#define SPI_NPCS_PIN		PIO_PB2_IDX
#define SPI_NPCS_MODE		IOPORT_MODE_MUX_D
// SPI DMA
#define SPI0_XDMAC_TX_CH_NUM	1 //XDMAC Channel HW Interface for SPI0
#define SPI0_XDMAC_RX_CH_NUM	2 //XDMAC Channel HW Interface for SPI0
#define XDMAC_TX_CH				0 //XDMAC Channel
#define XDMAC_RX_CH				1 //XDMAC Channel
static xdmac_channel_config_t xdmac_tx_cfg,xdmac_rx_cfg; //XDMAC channel configuration.

// ------- BOARD STATE
#define STATE_STOPPED			0
#define STATE_RUNNING			1

// ------- I2C COMMANDS
#define COMMAND_START_ACQUIRING		1
#define COMMAND_STOP_ACQUIRING		2
#define COMMAND_TOGGLE_LED			3

// ------- Other
#define EPHYS_SPI_TX_SIZE				34 // In unites of 2Bytes
#define EPHYS_NUM_CHAN					32
#define EPHYS_SSC_SAMP_PER_PACKET		1		
#define EPHYS_SSC_MAX_HEADER_SIZE		16  //In 2Byte increment (START + FRAME_NUM + TIME_STAMP)

#define EPHYS_FRAME_START		0xA83C42FA
#define DEFAULT_TICK_FACTOR		20000//30000

#define HEADER_START_WORD		0xF5C7B3A1
// --------------------- Functions ------------
void spi_init();
void twi_init();
void ssc_init();
void linevalid_init();
void dma_init();
void systick_init();
void dma_start();
//void construct_spi_reg_read(uint8_t);

//Handles new I2C data
void twi_new_data_received();

//Handles the transfer of data from SPI buffers to SSC buffers
void spi_2_ssc_transfer();

// ---------------- Variables -----------------
volatile uint32_t ephys_state			= STATE_STOPPED;
volatile uint8_t spi_dma_xfer_complete	= 0;
volatile uint8_t ssc_dma_state			= SSC_DMA_OFF;
volatile uint8_t new_ephys_commands	= 0; //Bool variable if new spi ephys commands need to be transmitted

// SPI buffers
volatile uint8_t spi_buffer_size = 4;
volatile uint8_t spi_buffer_write_index = 0;
volatile uint8_t spi_buffer_read_index = 0;
volatile uint8_t spi_buffer_num_pending = 0;
volatile uint16_t spi_tx_buffer[4][EPHYS_SPI_TX_SIZE] = {0}; //DMA TX Buffer
volatile uint16_t spi_rx_buffer[4][EPHYS_SPI_TX_SIZE] = {0}; //DMA RX Buffer
volatile uint32_t spi_timing_buffer[4] = {0}; //DMA RX Buffer
	
// Circular buffer for commands received over I2C that are waiting to be sent over SPI when ephys_state == Running
volatile uint8_t	 spi_command_buffer_size = 16;
volatile uint8_t  spi_command_buffer_read_position = 0;
volatile uint8_t  spi_command_buffer_write_position = 0; 
volatile uint8_t  spi_command_buffer_num_pending = 0;
volatile uint16_t spi_command_buffer[16] = {0}; //Holds commands received over TWI and waiting to be sent over SPI

// SSC buffers
volatile uint8_t ssc_buffer_size = 4;
volatile uint8_t ssc_buffer_write_index = 0;
volatile uint8_t ssc_buffer_read_index = 0;
volatile uint8_t ssc_buffer_num_pending = 0;
volatile uint32_t ssc_buffer_array_index = 0;
volatile uint16_t ssc_buffer[4][EPHYS_SSC_MAX_HEADER_SIZE+EPHYS_NUM_CHAN*EPHYS_SSC_SAMP_PER_PACKET] = {0};

volatile uint32_t tick_factor = DEFAULT_TICK_FACTOR;
volatile uint32_t acq_ticks = 0; //Counts up each time the Sys Clock starts a new acquisition. Should be 30K a second
volatile uint32_t packet_num = 0; //Holds the number of data packets sent over SSC
volatile uint16_t words_in_header = 0;
volatile uint16_t words_in_data_packet = EPHYS_NUM_CHAN*EPHYS_SSC_SAMP_PER_PACKET;

// Testing Vars -----------------------
volatile uint16_t count1 = 0;
volatile uint16_t count2 = 0;


//TWI Stuff
typedef struct _slave_device_t {
	/** PageAddress of the slave device*/
	uint16_t us_page_address;
	/** Offset of the memory access*/
	uint16_t us_offset_memory;
	/** Read address of the request*/
	uint8_t uc_acquire_address;
	/** Memory buffer*/
	uint8_t uc_memory[MCU_TWI_MEMORY_SIZE];
} slave_device_t;
slave_device_t twi_emulate_driver;

// --------------------------------------------
void systick_init() {
	// Configures and turns on Sys Tick and Interrupts
	acq_ticks = 0;
	
	SysTick_Config(sysclk_get_cpu_hz()/tick_factor);
}
uint16_t construct_spi_reg_read(uint8_t reg) {
	return 0xFFFF&((0b11000000|reg)<<8); //Reads 'I' from reg 40
}
uint16_t construct_spi_reg_write(uint8_t reg, uint8_t val) {
	return 0xFFFF&(((0b10000000|reg)<<8)|val); //Reads 'I' from reg 40
}

void construct_spi_tx_buffer (void) {
	for (uint8_t j=0;j<spi_buffer_size;j++) { //buffer size is hard coded in most of the code. Maybe change this
		for (uint16_t i=0;i<EPHYS_SPI_TX_SIZE;i++) {
			if (i<EPHYS_NUM_CHAN) {
				spi_tx_buffer[j][i] = 0xFFFF&(i<<8);
				//spi_tx_buffer[j][i] = construct_spi_reg_read(40);
			}
			else {
				spi_tx_buffer[j][i] = construct_spi_reg_read(40); //Reads 'I' from reg 40
			}
		}
	}
}
void fill_ssc_buffer_for_testing(void) {
	for (uint8_t j=0;j<4;j++) { //buffer size is hard coded in most of the code. Maybe change this
		for (uint16_t i=0;i<words_in_header+words_in_data_packet;i++) {
			ssc_buffer[j][i] = 0b1010101010101010;
			
		}
	}
}

void config_RHD2132(void){
	
	//Setting REG_ADC_CONFIG_AND_FAST_SETTLE (Reg 0)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_ADC_CONFIG_AND_FAST_SETTLE,0b11011110),SPI_CHIP_PCS,1);
	
	//Setting BIAS based on ADC sampling rate (>=700KS/s) (Reg 1 and 2)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_SUPPLY_SENSOR_AND_BIAS_CURRENT,2),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_MUX_BIAS_CURRENT,4),SPI_CHIP_PCS,1);
	
	//Setting MUX load and temp settings (Reg 3)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_MUX_LOAD,2),SPI_CHIP_PCS,1);
	
	//Setting ADC output format and DSP offset removal (DSP cutoff at 1.2Hz @ 30KS/s) (Reg 4)
	//spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_ADC_OUTPUT_FORMAT_DSP_OFFSET,0b00010000|12),SPI_CHIP_PCS,1);
	
	//Setting ADC output format and DSP offset removal (DSP cutoff at 1.4Hz @ 20KS/s) (Reg 4)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_ADC_OUTPUT_FORMAT_DSP_OFFSET,0b10010000|11),SPI_CHIP_PCS,1);
	
	////Setting upper bandwidth (20KHz) (Reg 8-11)
	//spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_1,8),SPI_CHIP_PCS,1);
	//spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_2,0),SPI_CHIP_PCS,1);
	//spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_3,4),SPI_CHIP_PCS,1);
	//spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_4,0),SPI_CHIP_PCS,1);
	
	//Setting upper bandwidth (10KHz) (Reg 8-11)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_1,17),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_2,0),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_3,23),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_4,0),SPI_CHIP_PCS,1);
	
	//Setting lower bandwidth (0.5Hz) (Reg 12 and 13)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_5,35),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_BANDWIDTH_6,17),SPI_CHIP_PCS,1);
	
	//Setting individual amp power (Reg 14-17)
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_POWER_1,0xFF),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_POWER_2,0xFF),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_POWER_3,0xFF),SPI_CHIP_PCS,1);
	spi_write(SPI_MASTER_BASE,construct_spi_reg_write(REG_AMP_POWER_4,0xFF),SPI_CHIP_PCS,1);
	
	
	spi_write(SPI_MASTER_BASE,CALIBRATE,SPI_CHIP_PCS,1);
	for (uint8_t i=0;i<9;i++)
		spi_write(SPI_MASTER_BASE,construct_spi_reg_read(40),SPI_CHIP_PCS,1); //Dummy commands needed after CALIBRATE
	
}

void linevalid_init() {
	// NOTE: Maybe don't need interrupt here. Just need to check state before sending SSC
	pmc_enable_periph_clk(LINEVALID_PORT);
	
	// ---------- If Just Using as Input
	ioport_set_pin_dir(LINEVALID_PIN,IOPORT_DIR_INPUT);
	
	// -------------- If Using Interrupt
	//PIOD->PIO_PER		|= LINEVALID_MASK; //PIO Enable. Takes control away from peripheral (is this OK?)
	//PIOD->PIO_ODR		|= LINEVALID_MASK; //Disables output on this pin.
	//PIOD->PIO_PPDER		|= LINEVALID_MASK; //Enables pulldown resistor
	//
	//PIOD->PIO_IER		|= LINEVALID_MASK; //Enables the input change interrupt
	//PIOD->PIO_AIMER		|= LINEVALID_MASK; //Enables additional Interrupt modes
	//PIOD->PIO_ESR		|= LINEVALID_MASK; //Enables edge detect. (Edge detect is on by default)
	//PIOD->PIO_FELLSR	|= LINEVALID_MASK; //Edge detect is for falling edge (Falling edge is on by default)
//
	//NVIC_ClearPendingIRQ(PIOD_IRQn);
	//NVIC_SetPriority(PIOD_IRQn, 2);
	//NVIC_EnableIRQ( PIOD_IRQn );
	//
	
}

//Handles new I2C data
void twi_new_data_received() {
	
	switch (twi_emulate_driver.uc_memory[0]) {
		case (COMMAND_STOP_ACQUIRING):
			//Kill DMA
			//Clear Buffer and reset counters		
			ephys_state = STATE_STOPPED;
			SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk; //Disable systick counter
		
			packet_num = 0;
		
			spi_buffer_read_index = 0;
			spi_buffer_write_index = 0;
			spi_command_buffer_num_pending = 0;
		
			ssc_buffer_write_index = 0;
			ssc_buffer_read_index = 0;
			ssc_buffer_num_pending = 0;
		
			acq_ticks = 0;	
					
			break;
		case (COMMAND_START_ACQUIRING):
			//Initialize DMAs, systick, and counters
			if (ephys_state == STATE_STOPPED) {
				//Setup SPI DMA
				//Setup SSC DMA
				//Initialize variables
			
				ephys_state = STATE_RUNNING;
				systick_init(); //Sets up and turns on systick
			}
			break;
		case (COMMAND_TOGGLE_LED):
			// Switch LED output
			break;
		default:
		// Handles RHD reg commands received by slave I2C
			if (ephys_state == STATE_STOPPED) {
				spi_write(SPI_MASTER_BASE,twi_emulate_driver.uc_memory[0],SPI_CHIP_PCS,1); //Not sure about the PCS value here
			}
			else if (ephys_state == STATE_RUNNING) {
				//Add data to command buffer			
				spi_command_buffer[spi_command_buffer_write_position] = twi_emulate_driver.uc_memory[0];
				spi_command_buffer_write_position = 0b00001111&(spi_command_buffer_write_position+1); //hard coded size to 0b00001111. Maybe change this
				spi_command_buffer_num_pending += 1;
				//Maybe check for overflow
			}
	}
	
}

void spi_master_init() {
	ioport_set_pin_mode(SPI_MISO_PIN,SPI_MISO_MODE);
	ioport_disable_pin(SPI_MISO_PIN);
	ioport_set_pin_mode(SPI_MOSI_PIN,SPI_MOSI_MODE);
	ioport_disable_pin(SPI_MOSI_PIN);
	ioport_set_pin_mode(SPI_SPCK_PIN,SPI_SPCK_MODE);
	ioport_disable_pin(SPI_SPCK_PIN);
	ioport_set_pin_mode(SPI_NPCS_PIN,SPI_NPCS_MODE);
	ioport_disable_pin(SPI_NPCS_PIN);
	
	
	pmc_enable_periph_clk(SPI_ID);

	SPI_MASTER_BASE->SPI_CR = SPI_CR_SPIDIS; //Disabled SPI
	SPI_MASTER_BASE->SPI_CR = SPI_CR_SWRST; //Resets SPI
	SPI_MASTER_BASE->SPI_CR = SPI_CR_LASTXFER; //De-asserts NPCS after every TD transfer
	SPI_MASTER_BASE->SPI_MR |= SPI_MR_MSTR; //Sets to master mode
	SPI_MASTER_BASE->SPI_MR |= SPI_MR_MODFDIS; //Disables fault detection
	
	// Sets Peripheral Chip Select
	SPI_MASTER_BASE->SPI_MR &= (~SPI_MR_PCS_Msk); //Empties out the PCS bits
	SPI_MASTER_BASE->SPI_MR |= SPI_MR_PCS(SPI_CHIP_PCS); //Sets the PCS bits
	
	// Sets Polarity
	if (SPI_CLK_POLARITY) {
		SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_CPOL; //1
	}
	else {
		SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_CPOL); //0
	}
	
	// Sets Phase
	if (SPI_CLK_NPHASE) {
		SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_NCPHA; //1
	} 
	else {
		SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_NCPHA); //0
	}
	
	// Peripheral chip select line rises after each transfer for a minimum of DLYBCS
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_CSNAAT;
	
	//Bits Per Transfer
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_BITS_Msk);
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_BITS_16_BIT;

	//Set BAUD rate div CHECK ACTUAL SCLK OUTPUT!!!
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_SCBR_Msk);
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_SCBR(sysclk_get_peripheral_hz()/ SPI_CLK_RATE);

	//Set Delays
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] &= ~(SPI_CSR_DLYBS_Msk | SPI_CSR_DLYBCT_Msk);
	SPI_MASTER_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_DLYBS(SPI_DLYBS) | SPI_CSR_DLYBCT(SPI_DLYBCT);
	
	//Sets Delay of NPCS is held high between transfers	
	SPI_MASTER_BASE->SPI_MR &= (~SPI_MR_DLYBCS_Msk);
	SPI_MASTER_BASE->SPI_MR |= SPI_MR_DLYBCS(SPI_DLYBCS);
	
	
	SPI_MASTER_BASE->SPI_CR = SPI_CR_SPIEN; //Enable SPI!
}

static void ssc_xdmac_configure() {
 	//This was taken from the SPI DMA configure and modified for SSC. Might have issues
	uint32_t xdmaint;

	/* Initialize and enable DMA controller */
	pmc_enable_periph_clk(ID_XDMAC);

	xdmaint = (XDMAC_CIE_BIE);// |
	//XDMAC_CIE_DIE   |
	//XDMAC_CIE_FIE   |
	//XDMAC_CIE_RBIE  |
	//XDMAC_CIE_WBIE  |
	//XDMAC_CIE_ROIE);


	/* Initialize channel config for transmitter */
	ssc_xdmac_tx_cfg.mbr_ubc = (words_in_header+words_in_data_packet)/2; //ssc_xdmac_tx_cfg.mbr_ubc = words_in_header+words_in_data_packet;

	ssc_xdmac_tx_cfg.mbr_sa = (uint32_t)&ssc_buffer[ssc_buffer_read_index][0];
	ssc_xdmac_tx_cfg.mbr_da = (uint32_t)&(SSC->SSC_THR); // Is this the correct destination address??
	ssc_xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_DSYNC_MEM2PER |
	XDMAC_CC_CSIZE_CHK_1 |
	XDMAC_CC_DWIDTH_WORD | //XDMAC_CC_DWIDTH_HALFWORD |
	XDMAC_CC_SIF_AHB_IF0 |
	XDMAC_CC_DIF_AHB_IF1 |
	XDMAC_CC_SAM_INCREMENTED_AM |
	XDMAC_CC_DAM_FIXED_AM |
	XDMAC_CC_PERID(SSC_XDMAC_TX_CH_NUM);

	ssc_xdmac_tx_cfg.mbr_bc = 0;
	ssc_xdmac_tx_cfg.mbr_ds =  0;
	ssc_xdmac_tx_cfg.mbr_sus = 0;
	ssc_xdmac_tx_cfg.mbr_dus = 0;

	xdmac_configure_transfer(XDMAC, SSC_XDMAC_TX_CH, &ssc_xdmac_tx_cfg);
	
	ssc_buffer_read_index = 0b00000011&(ssc_buffer_read_index+1);

	
}

static void ssc_enable_xdmac(void) {
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE);// |
	//XDMAC_CIE_DIE   |
	//XDMAC_CIE_FIE   |
	//XDMAC_CIE_RBIE  |
	//XDMAC_CIE_WBIE  |
	//XDMAC_CIE_ROIE);
	
	ssc_dma_state = SSC_DMA_ON;
	
	SSC->SSC_TSHR = (uint16_t)(0xFFFF&packet_num);
	
	xdmac_channel_set_descriptor_control(XDMAC, SSC_XDMAC_TX_CH, 0);
	xdmac_channel_enable_interrupt(XDMAC, SSC_XDMAC_TX_CH, xdmaint);
	xdmac_enable_interrupt(XDMAC,SSC_XDMAC_TX_CH);
	//xdmac_channel_enable(XDMAC, SSC_XDMAC_TX_CH);
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << SSC_XDMAC_TX_CH);
	
	ssc_buffer_num_pending--;
	/*Enable XDMAC interrupt */
	//NVIC_ClearPendingIRQ(XDMAC_IRQn);
	//NVIC_SetPriority( XDMAC_IRQn ,1);
	//NVIC_EnableIRQ(XDMAC_IRQn);
}

static void ssc_disable_xdmac(void) {
	//Taken from SPI DMA disable function. Might need changing
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE);// |
	//XDMAC_CIE_DIE   |
	//XDMAC_CIE_FIE   |
	//XDMAC_CIE_RBIE  |
	//XDMAC_CIE_WBIE  |
	//XDMAC_CIE_ROIE);

	//xdmac_channel_disable_interrupt(XDMAC, SSC_XDMAC_TX_CH, xdmaint); //probably don't need to disable this every time
	xdmac_channel_disable(XDMAC, SSC_XDMAC_TX_CH);
	//xdmac_disable_interrupt(XDMAC, SSC_XDMAC_TX_CH);
	
	ssc_dma_state = SSC_DMA_OFF;
}

static void spi_xdmac_configure(Spi *const pspi)
{
	//uint32_t xdmaint;
//
	//xdmaint = (XDMAC_CIE_BIE);// |
	////XDMAC_CIE_DIE   |
	////XDMAC_CIE_FIE   |
	////XDMAC_CIE_RBIE  |
	////XDMAC_CIE_WBIE  |
	////XDMAC_CIE_ROIE);

	/* Initialize channel config for transmitter */
	xdmac_tx_cfg.mbr_ubc = EPHYS_SPI_TX_SIZE;

	xdmac_tx_cfg.mbr_sa = (uint32_t)&spi_tx_buffer[spi_buffer_write_index][0];
	xdmac_tx_cfg.mbr_da = (uint32_t)&(pspi->SPI_TDR);
	xdmac_tx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_DSYNC_MEM2PER |
	XDMAC_CC_CSIZE_CHK_1 |
	XDMAC_CC_DWIDTH_HALFWORD |
	XDMAC_CC_SIF_AHB_IF0 |
	XDMAC_CC_DIF_AHB_IF1 |
	XDMAC_CC_SAM_INCREMENTED_AM |
	XDMAC_CC_DAM_FIXED_AM |
	XDMAC_CC_PERID(SPI0_XDMAC_TX_CH_NUM);

	xdmac_tx_cfg.mbr_bc = 0;
	xdmac_tx_cfg.mbr_ds =  0;
	xdmac_tx_cfg.mbr_sus = 0;
	xdmac_tx_cfg.mbr_dus = 0;

	xdmac_configure_transfer(XDMAC, XDMAC_TX_CH, &xdmac_tx_cfg);

	//Moved to enable_DMA function
	//xdmac_channel_set_descriptor_control(XDMAC, XDMAC_TX_CH, 0);
	//xdmac_channel_enable_interrupt(XDMAC, XDMAC_TX_CH, xdmaint);
	//xdmac_channel_enable(XDMAC, XDMAC_TX_CH);
	//xdmac_enable_interrupt(XDMAC, XDMAC_TX_CH);

	/* Initialize channel config for receiver */
	xdmac_rx_cfg.mbr_ubc = EPHYS_SPI_TX_SIZE;
	xdmac_rx_cfg.mbr_da = (uint32_t)&spi_rx_buffer[spi_buffer_write_index][0];

	xdmac_rx_cfg.mbr_sa = (uint32_t)&pspi->SPI_RDR;
	xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_DSYNC_PER2MEM |
	XDMAC_CC_CSIZE_CHK_1 |
	XDMAC_CC_DWIDTH_HALFWORD|
	XDMAC_CC_SIF_AHB_IF1 |
	XDMAC_CC_DIF_AHB_IF0 |
	XDMAC_CC_SAM_FIXED_AM |
	XDMAC_CC_DAM_INCREMENTED_AM |
	XDMAC_CC_PERID(SPI0_XDMAC_RX_CH_NUM);

	xdmac_rx_cfg.mbr_bc = 0;
	xdmac_rx_cfg.mbr_ds =  0; //as _tx_ for some reason
	xdmac_rx_cfg.mbr_sus = 0;
	xdmac_rx_cfg.mbr_dus =0;

	xdmac_configure_transfer(XDMAC, XDMAC_RX_CH, &xdmac_rx_cfg);

	//Moved to enable_dma function
	//xdmac_channel_set_descriptor_control(XDMAC, XDMAC_RX_CH, 0);
	//xdmac_channel_enable_interrupt(XDMAC, XDMAC_RX_CH, xdmaint);
	//xdmac_channel_enable(XDMAC, XDMAC_RX_CH);
	//xdmac_enable_interrupt(XDMAC, XDMAC_RX_CH);

	spi_buffer_write_index = 0b00000011&(spi_buffer_write_index+1);
	
	/*Enable XDMAC interrupt */
	//Moved to enable dma function
	//NVIC_ClearPendingIRQ(XDMAC_IRQn);
	//NVIC_SetPriority( XDMAC_IRQn ,1);
	//NVIC_EnableIRQ(XDMAC_IRQn);
}
static void spi_enable_xdmac() {
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE);// |
	//XDMAC_CIE_DIE   |
	//XDMAC_CIE_FIE   |
	//XDMAC_CIE_RBIE  |
	//XDMAC_CIE_WBIE  |
	//XDMAC_CIE_ROIE);
	
	xdmac_channel_set_descriptor_control(XDMAC, XDMAC_TX_CH, 0);
	xdmac_channel_enable_interrupt(XDMAC, XDMAC_TX_CH, xdmaint);
	xdmac_enable_interrupt(XDMAC, XDMAC_TX_CH);
	//xdmac_channel_enable(XDMAC, XDMAC_TX_CH);
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << XDMAC_TX_CH);
	
	xdmac_channel_set_descriptor_control(XDMAC, XDMAC_RX_CH, 0);
	xdmac_channel_enable_interrupt(XDMAC, XDMAC_RX_CH, xdmaint);
	xdmac_enable_interrupt(XDMAC, XDMAC_RX_CH);
	//xdmac_channel_enable(XDMAC, XDMAC_RX_CH);
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << XDMAC_RX_CH);
	

}
static void spi_disable_xdmac(void)
{
	uint32_t xdmaint;

	xdmaint = (XDMAC_CIE_BIE);// |
	//XDMAC_CIE_DIE   |
	//XDMAC_CIE_FIE   |
	//XDMAC_CIE_RBIE  |
	//XDMAC_CIE_WBIE  |
	//XDMAC_CIE_ROIE);

	//xdmac_channel_disable_interrupt(XDMAC, XDMAC_RX_CH, xdmaint);
	xdmac_channel_disable(XDMAC, XDMAC_RX_CH);
	//xdmac_disable_interrupt(XDMAC, XDMAC_RX_CH);
//
	//xdmac_channel_disable_interrupt(XDMAC, XDMAC_TX_CH, xdmaint);
	xdmac_channel_disable(XDMAC, XDMAC_TX_CH);
	//xdmac_disable_interrupt(XDMAC, XDMAC_TX_CH);

	//NVIC_ClearPendingIRQ(XDMAC_IRQn);
	//NVIC_DisableIRQ(XDMAC_IRQn);
}

void twi_slave_init() {
	// NOTE: Need to update to operate as slave
	//twihs_options_t twihsOpt;
	//twihsOpt.master_clk = sysclk_get_cpu_hz()/2; //make sure this is the correct clock to be checking
	//twihsOpt.speed = 100000; //100KHz
	
	matrix_set_system_io(matrix_get_system_io() | CCFG_SYSIO_SYSIO4 | CCFG_SYSIO_SYSIO5);
	ioport_set_pin_mode(TWCK_PIN,TWCK_MODE);
	ioport_disable_pin(TWCK_PIN);
	ioport_set_pin_mode(TWD_PIN,TWD_MODE);
	ioport_disable_pin(TWD_PIN);

	pmc_enable_periph_clk(ID_TWIHS1);
	
	twihs_slave_init(TWIHS1,MCU_TWI_ADDRESS);
	
	twihs_read_byte(TWIHS1); //clears receipt buffer
	
	NVIC_ClearPendingIRQ(TWIHS1_IRQn);
	NVIC_DisableIRQ(TWIHS1_IRQn);
	NVIC_SetPriority(TWIHS1_IRQn, 3);
	NVIC_EnableIRQ(TWIHS1_IRQn);
	twihs_enable_interrupt(TWIHS1,TWIHS_SR_SVACC); //Turns on Slave Access Interrupt Enable
}


void ssc_init() {
	ioport_set_pin_mode(TD_PIN,TD_MODE);
	ioport_disable_pin(TD_PIN);
	ioport_set_pin_mode(TF_PIN,TF_MODE);
	ioport_disable_pin(TF_PIN);
	ioport_set_pin_mode(TK_PIN,TK_MODE);
	ioport_disable_pin(TK_PIN);
	
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);
	
	SSC->SSC_CR = SSC_CR_TXDIS; //Disables SSC Tx
	
	//SSC->SSC_CMR = ?; //Clock divider?
	SSC->SSC_TCMR |= (SSC_TCMR_CKS_TK		//Tx clock selection
						| SSC_TCMR_STTDLY(16)
						| SSC_TCMR_CKO_NONE);	//TK pin is an input
	//					| SSC_TCMR_CKI);	//Clock inversion. 0 = sampled on rising edge
	//SSC->SSC_TFMR |= (SSC_TFMR_DATLEN(31) //(SSC_TFMR_DATLEN(15)	//Sets data length (value + 1 bits)
	//					| SSC_TFMR_DATNB(15) //Added this when switching to 32bit words
	//					| SSC_TFMR_DATDEF	//Data default value
	//					| SSC_TFMR_MSBF		//Most sig bit first
	//					| SSC_TFMR_FSOS_HIGH//Tx frame out 
	//					);
	
	SSC->SSC_TFMR |= (SSC_TFMR_DATLEN(31) //(SSC_TFMR_DATLEN(15)	//Sets data length (value + 1 bits)
						| SSC_TFMR_DATNB(15) //Added this when switching to 32bit words
						| SSC_TFMR_DATDEF	//Data default value
						| SSC_TFMR_FSLEN(15)
						| SSC_TFMR_MSBF		//Most sig bit first
						| SSC_TFMR_FSOS_HIGH//Tx frame out
						| SSC_TFMR_FSDEN
						);
	
	SSC->SSC_CR |= SSC_CR_TXEN; //Transmit Enable		
}


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	WDT->WDT_MR = WDT_MR_WDDIS; //Disables WDT
	
	//SCB_EnableICache();
	//SCB_EnableDCache();
	
	irq_initialize_vectors();
	cpu_irq_enable();
	
	sysclk_init();
	board_init();
	
	//NVIC_ClearPendingIRQ(SPI_IRQn);
	//NVIC_DisableIRQ(SPI_IRQn);
	//NVIC_SetPriority(SPI_IRQn, 0);
	//NVIC_EnableIRQ(SPI_IRQn);
	spi_master_init();
	
	config_RHD2132();
	//
	//
	//twi_slave_init();
	//
	//
	ssc_init();
	//
	fill_ssc_buffer_for_testing(); //This can be taken out later
	
	construct_spi_tx_buffer();
	
		/*Enable XDMAC interrupt */
		pmc_enable_periph_clk(ID_XDMAC);
		
	spi_xdmac_configure(SPI_MASTER_BASE);
	//ssc_write(SSC,0b1010101010101010);
	ssc_xdmac_configure();
	
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
		NVIC_DisableIRQ(XDMAC_IRQn);
		NVIC_SetPriority( XDMAC_IRQn ,1);
		NVIC_EnableIRQ(XDMAC_IRQn);
	//----- Enable control of LED
	pmc_enable_periph_clk(LED_PORT);
	ioport_set_pin_dir(LED_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_PIN, 1); //1 is on, 0 is off
	
	
	//----- Testing stuff
	uint16_t spi_word = 0;
	uint8_t p_pcs2 = 0;
	
	
	ephys_state = STATE_RUNNING;
	systick_init();
	//-----------------------			
	while(1) {
		if (spi_buffer_num_pending > 0) {
			//New data in SPI buffer than needs to go into SSC buffer
			//spi_buffer_num_pending--;
			spi_2_ssc_transfer();
			//ssc_write(SSC,0b1010101010101010);
		}
		if ((ssc_buffer_num_pending > 0) && (ssc_dma_state == SSC_DMA_OFF)) {
			//New SSC packet is ready and SSC DMA is currently off
			//Start SSC DMA			
			ssc_enable_xdmac();
			spi_dma_xfer_complete++;
		}
		//spi_write(SPI_MASTER_BASE,construct_spi_reg_read(40),SPI_CHIP_PCS,1); //Not sure about the PCS value here
		//spi_read(SPI_MASTER_BASE, &spi_word, &p_pcs2);
		//spi_write(SPI_MASTER_BASE,construct_spi_reg_read(41),SPI_CHIP_PCS,1); //Not sure about the PCS value here
		//spi_read(SPI_MASTER_BASE, &spi_word, &p_pcs2);
		//spi_write(SPI_MASTER_BASE,construct_spi_reg_read(42),SPI_CHIP_PCS,1); //Not sure about the PCS value here
		//spi_read(SPI_MASTER_BASE, &spi_word, &p_pcs2);
		//spi_write(SPI_MASTER_BASE,construct_spi_reg_read(43),SPI_CHIP_PCS,1); //Not sure about the PCS value here
		//spi_read(SPI_MASTER_BASE, &spi_word, &p_pcs2);
		//spi_write(SPI_MASTER_BASE,construct_spi_reg_read(44),SPI_CHIP_PCS,1); //Not sure about the PCS value here		
		//spi_read(SPI_MASTER_BASE, &spi_word, &p_pcs2);
	
		

	}

}

void spi_2_ssc_transfer() {
	if (ssc_buffer_array_index == words_in_header) {
		//This is the first data being put into this SSC buffer.
		//We can add the timestamp of this SPI buffer into the SSC buffer header
		
		//ssc_buffer[ssc_buffer_write_index][0] = 0xFFFF&(HEADER_START_WORD>>16);
		//ssc_buffer[ssc_buffer_write_index][1] = 0xFFFF&(HEADER_START_WORD);
		//ssc_buffer[ssc_buffer_write_index][6] = 0xFFFF&(spi_timing_buffer[spi_buffer_read_index]>>8);
		//ssc_buffer[ssc_buffer_write_index][7] = 0xFFFF&(spi_timing_buffer[spi_buffer_read_index]);
	}
	//for (uint8_t i=2; i<34; i++) {
		//
		//// Looks like the DMA of the SPI is one behind when transferring the RX compared to TX of the SPI.
		//// So I am doing this hacky thing to get electrode data in the correct order
		//ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = spi_rx_buffer[spi_buffer_read_index][i];
		//ssc_buffer_array_index++;
	//}
	// ===================================
	//for (uint8_t i=3; i<34; i++) {
		//
		//// Looks like the DMA of the SPI is one behind when transferring the RX compared to TX of the SPI.
		//// So I am doing this hacky thing to get electrode data in the correct order
		//ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = spi_rx_buffer[spi_buffer_read_index][i];
		//ssc_buffer_array_index++;
	//}
	//ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = spi_rx_buffer[spi_buffer_read_index][0];
	//ssc_buffer_array_index++;
		ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = count1;
		count1++;
		ssc_buffer_array_index++;
		for (uint8_t i=4; i<34; i++) {
			
			// Looks like the DMA of the SPI is one behind when transferring the RX compared to TX of the SPI.
			// So I am doing this hacky thing to get electrode data in the correct order
			ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = spi_rx_buffer[spi_buffer_read_index][i];
			ssc_buffer_array_index++;
		}
		ssc_buffer[ssc_buffer_write_index][ssc_buffer_array_index] = count2;
		count2++;
		ssc_buffer_array_index++;
	// ==================================
		
	spi_buffer_read_index = 0b0000011&(spi_buffer_read_index+1);
	spi_buffer_num_pending--;
	
	if (ssc_buffer_array_index >= (words_in_header+words_in_data_packet)) {
		//SSC Packet is full
		//ssc_buffer[ssc_buffer_write_index][2] = words_in_header;
		//ssc_buffer[ssc_buffer_write_index][3] = words_in_data_packet;
		//ssc_buffer[ssc_buffer_write_index][2] = 0xFFFF&(packet_num>>16); //Upper part of packet number
		//ssc_buffer[ssc_buffer_write_index][3] = 0xFFFF&(packet_num); //Lower part of packet number
		// ******* Still need timestamp and error flags in header
		
		
		packet_num++;
		ssc_buffer_array_index = words_in_header; //Sets index in next packet after header slots		
		ssc_buffer_write_index = 0b00000011&(ssc_buffer_write_index+1);
		ssc_buffer_num_pending++;
		
	}
}
// ------------ INTERUPTS ----------------

//void PIOD_Handler(void) {
//
//}
void TWIHS1_Handler(void) {
	uint32_t status;

	status = twihs_get_interrupt_status(TWIHS1);

	if (((status & TWIHS_SR_SVACC) == TWIHS_SR_SVACC)
	&& (twi_emulate_driver.uc_acquire_address == 0)) {
		twihs_disable_interrupt(TWIHS1, TWIHS_IDR_SVACC);
		twihs_enable_interrupt(TWIHS1,
		TWIHS_IER_RXRDY | TWIHS_IER_GACC |
		TWIHS_IER_NACK | TWIHS_IER_EOSACC | TWIHS_IER_SCL_WS);
		twi_emulate_driver.uc_acquire_address++;
		twi_emulate_driver.us_page_address = 0;
		twi_emulate_driver.us_offset_memory = 0;
	}

	if ((status & TWIHS_SR_GACC) == TWIHS_SR_GACC) {
		//puts("General Call Treatment\n\r");
		//puts("not treated");
	}

	if (((status & TWIHS_SR_SVACC) == TWIHS_SR_SVACC) &&
	((status & TWIHS_SR_GACC) == 0) &&
	((status & TWIHS_SR_RXRDY) == TWIHS_SR_RXRDY)) {

		if (twi_emulate_driver.uc_acquire_address == 1) {
			/* Acquire MSB address */
			twi_emulate_driver.us_page_address =
			(twihs_read_byte(TWIHS1) & 0xFF) << 8;
			twi_emulate_driver.uc_acquire_address++;
		} else {
			if (twi_emulate_driver.uc_acquire_address == 2) {
				/* Acquire LSB address */
				twi_emulate_driver.us_page_address |=
				(twihs_read_byte(TWIHS1) & 0xFF);
				twi_emulate_driver.uc_acquire_address++;
			} else {
				/* Read one byte of data from master to slave device */
				twi_emulate_driver.uc_memory[twi_emulate_driver.us_page_address +
				twi_emulate_driver.us_offset_memory] =
				(twihs_read_byte(TWIHS1) & 0xFF);
				twi_emulate_driver.us_offset_memory++;
			}
		}
	} else {
		if (((status & TWIHS_SR_TXRDY) == TWIHS_SR_TXRDY)
		&& ((status & TWIHS_SR_TXCOMP) == TWIHS_SR_TXCOMP)
		&& ((status & TWIHS_SR_EOSACC) == TWIHS_SR_EOSACC)) {
			/* End of transfer, end of slave access */
			twi_emulate_driver.us_offset_memory = 0;
			twi_emulate_driver.uc_acquire_address = 0;
			twi_emulate_driver.us_page_address = 0;
			twihs_enable_interrupt(TWIHS1, TWIHS_SR_SVACC);
			twihs_disable_interrupt(TWIHS1,
			TWIHS_IDR_RXRDY | TWIHS_IDR_GACC |
			TWIHS_IDR_NACK | TWIHS_IDR_EOSACC | TWIHS_IDR_SCL_WS);
			
			// Call function to handle new I2C data
			twi_new_data_received();
		} else {
			if (((status & TWIHS_SR_SVACC) == TWIHS_SR_SVACC)
			&& ((status & TWIHS_SR_GACC) == 0)
			&& (twi_emulate_driver.uc_acquire_address == 3)
			&& ((status & TWIHS_SR_SVREAD) == TWIHS_SR_SVREAD)
			&& ((status & TWIHS_SR_NACK) == 0)) {
				/* Write one byte of data from slave to master device */
				twihs_write_byte(TWIHS1,
				twi_emulate_driver.uc_memory[twi_emulate_driver.us_page_address
				+ twi_emulate_driver.us_offset_memory]);
				twi_emulate_driver.us_offset_memory++;
			}
		}
	}
}


void XDMAC_Handler(void)
{
	//NOTE: Need to update for EPHYS
	uint32_t dma_status = 0;
	uint32_t ssc_dma_status = 0;
	uint32_t trash = 0;
	uint32_t DMA_Init = XDMAC->XDMAC_GIS;
	dma_status = XDMAC->XDMAC_CHID[XDMAC_RX_CH].XDMAC_CIS; //This checks the dma_status of the SPI DMA
	
	ssc_dma_status = XDMAC->XDMAC_CHID[SSC_XDMAC_TX_CH].XDMAC_CIS;
	trash = XDMAC->XDMAC_CHID[XDMAC_TX_CH].XDMAC_CIS;
	// ** Need to check SSC DMA status too for tx finish **
	
	if (dma_status & XDMAC_CIS_BIS) { //If SPI DMA is done
		spi_dma_xfer_complete = 1;
		if (ephys_state == STATE_RUNNING) { 
			spi_buffer_num_pending++;
			
			spi_disable_xdmac(); //Disable SPI DMA			
			spi_xdmac_configure(SPI_MASTER_BASE); //Set up next SPI DMA transfer **** Need to set it up but not turn it on ******
			
		}
	}
	if (ssc_dma_status & XDMAC_CIS_BIS) {
		//if (ssc_buffer_num_pending > 0) //Why do I need this?!?!?!?!?!?!
			//ssc_buffer_num_pending--;
		ssc_disable_xdmac();
		ssc_xdmac_configure();
		ssc_dma_state = SSC_DMA_OFF;
		
	}

		
		
		
	//NVIC_ClearPendingIRQ(XDMAC_IRQn);
	//NVIC_DisableIRQ(XDMAC_IRQn);
}

void SysTick_Handler(void) {
	//Should only be triggering when ephys is running.
	ioport_set_pin_level(LED_PIN,0b1&(acq_ticks>>10));
	//ioport_toggle_pin_level(LED_PIN);
	if (ephys_state == STATE_RUNNING) {
		acq_ticks++;
		spi_timing_buffer[spi_buffer_write_index] = acq_ticks; //This holds the systick at start of SPI DMA
		//spi_disable_xdmac();
		//spi_xdmac_configure(SPI_MASTER_BASE);
		spi_enable_xdmac();
		//Start SPI DMA. SPI DMA turns off in the DMA_SPI_Tx interrupt
		
		
	}
}

void SPI0_Handler (void) {
	while(1) {
		
	}
}

void PMC_Handler(void) {
	while(1) {
		
	}
}
void ICM_Handler (void) {
	while(1) {
		
	}
}