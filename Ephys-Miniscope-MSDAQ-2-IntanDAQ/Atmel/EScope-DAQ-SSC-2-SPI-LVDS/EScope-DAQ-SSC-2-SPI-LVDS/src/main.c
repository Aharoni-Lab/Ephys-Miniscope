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

//------------- Definitions -----------
//RHD2132 Reg Definitions
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

#define RHD_REG_BIT							0x8000
#define RHD_READ_BIT						0x4000

// ------- LEDs on Board
#define LED1_PIN			PIO_PC0_IDX
#define LED2_PIN			PIO_PC1_IDX
#define LED3_PIN			PIO_PC2_IDX
// ------- SSC Definitions
#define RD_PIN				PIO_PA10_IDX
#define RD_MODE				IOPORT_MODE_MUX_C
#define RF_PIN				PIO_PD24_IDX
#define RF_MODE				IOPORT_MODE_MUX_B
#define RK_PIN				PIO_PA22_IDX
#define RK_MODE				IOPORT_MODE_MUX_A

// ------- SSC DMA
#define SSC_XDMAC_RX_CH_NUM	33 //Actual DMA channel ID for SSC
#define SSC_XDMAC_RX_CH		0 //Specific DMA channel used for this communication (0 and 1 are taken by SPI0)

#define SSC_DMA_OFF			0
#define SSC_DMA_ON			1

// ------- SPI Definitions
#define SPI_SLAVE_BASE		SPI0
#define SPI_IRQn			SPI0_IRQn
#define SPI_ID				ID_SPI0

#define SPI_CHIP_SEL		0
#define SPI_CHIP_PCS		spi_get_pcs(SPI_CHIP_SEL)
#define SPI_CLK_POLARITY	0
#define SPI_CLK_NPHASE		1
#define SPI_DLYBS			0x0 //Delay between NPCS activation to first SPCK (0 is half clock cycle)
#define SPI_DLYBCT			0x01 //Delay between consecutive transfers. A delay before CS is pulled up at end of transfer
#define SPI_DLYBCS			0x10 // Delay NPCS is held high before next transmit
#define SPI_CLK_RATE		10000000//24000000

#define SPI_MISO_PIN		PIO_PD20_IDX
#define SPI_MISO_MODE		IOPORT_MODE_MUX_B
#define SPI_MOSI_PIN		PIO_PD21_IDX
#define SPI_MOSI_MODE		IOPORT_MODE_MUX_B
#define SPI_SPCK_PIN		PIO_PD22_IDX
#define SPI_SPCK_MODE		IOPORT_MODE_MUX_B
#define SPI_NPCS_PIN		PIO_PB2_IDX
#define SPI_NPCS_MODE		IOPORT_MODE_MUX_D

//Other Definitions
#define EPHYS_FRAME_START				0xA83C42FA

#define EPHYS_NUM_CHAN					32
#define EPHYS_SSC_SAMP_PER_PACKET		1
#define MAX_WORDS_IN_HEADER				4
#define MAX_WORDS_IN_DATA_PACKET		EPHYS_NUM_CHAN*EPHYS_SSC_SAMP_PER_PACKET

//Functions

// ------------ Global Vars -----------
static xdmac_channel_config_t ssc_xdmac_rx_cfg; //XDMAC channel configuration.
volatile uint32_t words_in_header		= 0; //4; //????
volatile uint32_t words_in_data_packet	= 32;
volatile uint16_t RHD_reg_value[256] = {0}; //Holds a copy of all RHD2132 reg values
volatile uint16_t RHD_ADC_value[MAX_WORDS_IN_HEADER+MAX_WORDS_IN_DATA_PACKET] = {0}; //Holds last sampled ADC value

volatile uint16_t packet_num = 0;
volatile uint16_t num_packets_received = 0;
volatile uint16_t spi_miso_next_data = 0; //Holds upcoming MISO output words

// Testing vars
volatile uint16_t testCh = 0;

volatile uint16_t spi_tx[35] = {0};
volatile uint16_t spi_tx_idx = 0;

volatile uint16_t spi_rx[35] = {0};
volatile uint16_t spi_rx_idx = 0;

// ---------- Functions ----------
void fill_RHD_reg_value_array(void);
uint8_t RHD_get_reg_from_command(uint16_t);
uint8_t RHD_get_channel_from_command(uint16_t);
uint8_t RHD_get_value_from_command(uint16_t);
uint16_t spi_handle_command(uint16_t );
void spi_slave_init(void);
void ssc_init(void);
void ssc_xdmac_configure(void);
void ssc_disable_xdmac(void);
void ssc_enable_xdmac(void);
// ------------------------------------

void fill_RHD_reg_value_array(void) {
	//Don't currently have default values for reg. Hopefully I don't need them
	RHD_reg_value[0b11000000|40] = 'I';
	RHD_reg_value[0b11000000|41] = 'N';
	RHD_reg_value[0b11000000|42] = 'T';
	RHD_reg_value[0b11000000|43] = 'A';
	RHD_reg_value[0b11000000|44] = 'N';
	
	RHD_reg_value[0b11000000|48] = 'R'; // From Intan source code
	RHD_reg_value[0b11000000|49] = 'H'; // From Intan source code
	RHD_reg_value[0b11000000|50] = 'D'; // From Intan source code
	
	//RHD_reg_value[40] = ; //Die revision
	RHD_reg_value[0b11000000|59] = 0; // For RHD2132 found in Intan source code
	RHD_reg_value[0b11000000|61] = 1; //Unipolar amps
	RHD_reg_value[0b11000000|62] = 32; //Number of amps on chip
	RHD_reg_value[0b11000000|63] = 1; //RHD2132 chip ID
	
}

uint8_t RHD_get_reg_from_command(uint16_t command) {
	return (0x3F&(command>>8));
}

uint8_t RHD_get_channel_from_command(uint16_t command) {
	return (0x3F&(command>>8));
}

uint8_t RHD_get_value_from_command(uint16_t command) {
	return (0xFF&command);
}

uint16_t spi_handle_command(uint16_t command) {
	//This will return what the Intan DAQ is expecting to get from the MISO line
	volatile uint8_t testComm;
	volatile uint16_t outValue = 0;
	
	// Testing data in and out of spi
	//spi_tx[spi_tx_idx] = spi_miso_next_data;
	//spi_tx_idx++;
	//if (spi_tx_idx >= 35) {
	//spi_tx_idx = 0;
	//}
	//spi_rx[spi_rx_idx] = command;
	//spi_rx_idx++;
	//if (spi_rx_idx >= 35) {
	//spi_rx_idx = 0;
	//}
	// ===============================
	
	if ((command&(RHD_READ_BIT|RHD_REG_BIT)) == 0) {
		//Convert command
		//RHD_ADC_value[RHD_get_channel_from_command(command)+words_in_header]++;
		testComm = RHD_get_channel_from_command(command);
		if ((testComm == 1) || (testComm == 9)) {
			testCh++;
			outValue = testCh;
		}
		else {
			outValue = (RHD_ADC_value[testComm+words_in_header]);
		}
	}
	else if (command&RHD_REG_BIT){
		//Will be reading or writing to a reg in RHD
		if (command&RHD_READ_BIT) {
			//Will be reading an RHD reg
			outValue = (RHD_reg_value[RHD_get_reg_from_command(command)]);
		}
		else {
			//Will be writing to an RHD reg
			//We are going to fake a write
			RHD_reg_value[RHD_get_reg_from_command(command)] = RHD_get_value_from_command(command);
			outValue = (0xFF00 | RHD_get_value_from_command(command));
		}
	}
	else {
		//Handles all non-read/write commands
		switch (command)
		{
			case (CALIBRATE):
			//Should actually return this on this and following 9 commands
			outValue = (0x0000); //Will be 0x8000 is expecting 2's complement
			break;
			case (CLEAR):
			outValue = (0x0000); //Will be 0x8000 is expecting 2's complement
			break;
		}
	}
	return outValue; // Should never get here
}

void spi_slave_init(void) {
	//Need to shift from Master to Slave !!!!
	ioport_set_pin_mode(SPI_MISO_PIN,SPI_MISO_MODE);
	ioport_disable_pin(SPI_MISO_PIN);
	ioport_set_pin_mode(SPI_MOSI_PIN,SPI_MOSI_MODE);
	ioport_disable_pin(SPI_MOSI_PIN);
	ioport_set_pin_mode(SPI_SPCK_PIN,SPI_SPCK_MODE);
	ioport_disable_pin(SPI_SPCK_PIN);
	ioport_set_pin_mode(SPI_NPCS_PIN,SPI_NPCS_MODE);
	ioport_disable_pin(SPI_NPCS_PIN);
	
	pmc_enable_periph_clk(SPI_ID);

	SPI_SLAVE_BASE->SPI_CR = SPI_CR_SPIDIS; //Disabled SPI
	SPI_SLAVE_BASE->SPI_CR = SPI_CR_SWRST; //Resets SPI
	//SPI_SLAVE_BASE->SPI_CR = SPI_CR_LASTXFER; //De-asserts NPCS after every TD transfer
	//SPI_SLAVE_BASE->SPI_MR |= SPI_MR_MSTR; //Sets to master mode
	SPI_SLAVE_BASE->SPI_MR |= SPI_MR_MODFDIS; //Disables fault detection
	
	// Sets Peripheral Chip Select
	SPI_SLAVE_BASE->SPI_MR &= (~SPI_MR_PCS_Msk); //Empties out the PCS bits
	SPI_SLAVE_BASE->SPI_MR |= SPI_MR_PCS(SPI_CHIP_PCS); //Sets the PCS bits
	
	// Sets Polarity
	if (SPI_CLK_POLARITY) {
		SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_CPOL; //1
	}
	else {
		SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_CPOL); //0
	}
	
	// Sets Phase
	if (SPI_CLK_NPHASE) {
		SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_NCPHA; //1
	}
	else {
		SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_NCPHA); //0
	}
	
	// Peripheral chip select line rises after each transfer for a minimum of DLYBCS
	//SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_CSNAAT;
	
	//Bits Per Transfer
	SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_BITS_Msk);
	SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_BITS_16_BIT;

	//Set BAUD rate div CHECK ACTUAL SCLK OUTPUT!!!
	//SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] &= (~SPI_CSR_SCBR_Msk);
	//SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_SCBR(sysclk_get_peripheral_hz()/ SPI_CLK_RATE);

	//Set Delays
	//SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] &= ~(SPI_CSR_DLYBS_Msk | SPI_CSR_DLYBCT_Msk);
	//SPI_SLAVE_BASE->SPI_CSR[SPI_CHIP_SEL] |= SPI_CSR_DLYBS(SPI_DLYBS) | SPI_CSR_DLYBCT(SPI_DLYBCT);
	
	//Sets Delay of NPCS is held high between transfers
	//SPI_SLAVE_BASE->SPI_MR &= (~SPI_MR_DLYBCS_Msk);
	//SPI_SLAVE_BASE->SPI_MR |= SPI_MR_DLYBCS(SPI_DLYBCS);
	
	//Set up interrupts
	SPI_SLAVE_BASE->SPI_IER = SPI_IER_RDRF | SPI_IER_TDRE; //SPI receive reg full interrupt
	SPI_SLAVE_BASE->SPI_CR = SPI_CR_SPIEN; //Enable SPI!
	
	
}

void ssc_init(void) {
	ioport_set_pin_mode(RD_PIN,RD_MODE);
	ioport_disable_pin(RD_PIN);
	ioport_set_pin_mode(RF_PIN,RF_MODE);
	ioport_disable_pin(RF_PIN);
	ioport_set_pin_mode(RK_PIN,RK_MODE);
	ioport_disable_pin(RK_PIN);
	
	pmc_enable_periph_clk(ID_SSC);
	ssc_reset(SSC);
	
	SSC->SSC_CR = SSC_CR_RXDIS; //Disables SSC Rx
	
	//SSC->SSC_CMR = ?; //Clock divider?
	//Receiver clock settings
	SSC->SSC_RCMR = (SSC_RCMR_CKS_RK | //Use RK pin as input clock
	SSC_RCMR_CKI | //Sampled on rising edge
	SSC_RCMR_STTDLY(16) |
	SSC_RCMR_START_RF_RISING); //Sets start trigger
	//SSC->SSC_RFMR = (SSC_RFMR_DATLEN(31) | //(SSC_RFMR_DATLEN(15) | //Data bit length
	//				SSC_RFMR_DATNB(15) |
	//				SSC_RFMR_MSBF);
	SSC->SSC_RFMR = (SSC_RFMR_DATLEN(31) | //(SSC_RFMR_DATLEN(15) | //Data bit length
	SSC_RFMR_FSLEN(15) | //Putting 16bit sync to hold packet number
	SSC_RFMR_DATNB(15) |
	SSC_RFMR_MSBF);
	
	SSC->SSC_CR |= SSC_CR_RXEN; //Transmit Enable
}

void ssc_xdmac_configure(void) {


	/* Initialize channel config for receiver */
	ssc_xdmac_rx_cfg.mbr_ubc = (words_in_header+words_in_data_packet)/2; //ssc_xdmac_rx_cfg.mbr_ubc = words_in_header+words_in_data_packet;

	ssc_xdmac_rx_cfg.mbr_sa = (uint32_t)&(SSC->SSC_RHR);
	ssc_xdmac_rx_cfg.mbr_da = (uint32_t)&RHD_reg_value[0];
	ssc_xdmac_rx_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN |
	XDMAC_CC_MBSIZE_SINGLE |
	XDMAC_CC_DSYNC_PER2MEM |
	XDMAC_CC_CSIZE_CHK_1 |
	XDMAC_CC_DWIDTH_WORD | //XDMAC_CC_DWIDTH_HALFWORD |
	XDMAC_CC_SIF_AHB_IF1 |
	XDMAC_CC_DIF_AHB_IF0 |
	XDMAC_CC_SAM_FIXED_AM |
	XDMAC_CC_DAM_INCREMENTED_AM |
	XDMAC_CC_PERID(SSC_XDMAC_RX_CH_NUM);

	ssc_xdmac_rx_cfg.mbr_bc = 0;
	ssc_xdmac_rx_cfg.mbr_ds =  0;
	ssc_xdmac_rx_cfg.mbr_sus = 0;
	ssc_xdmac_rx_cfg.mbr_dus = 0;

	xdmac_configure_transfer(XDMAC, SSC_XDMAC_RX_CH, &ssc_xdmac_rx_cfg);
	
	xdmac_channel_set_descriptor_control(XDMAC, SSC_XDMAC_RX_CH, 0);
	xdmac_channel_enable_interrupt(XDMAC, SSC_XDMAC_RX_CH, XDMAC_CIE_BIE);
	xdmac_enable_interrupt(XDMAC,SSC_XDMAC_RX_CH);
	//xdmac_channel_enable(XDMAC, SSC_XDMAC_RX_CH);
	
	
}

void ssc_enable_xdmac(void) {
	
	SCB_CleanInvalidateDCache();
	XDMAC->XDMAC_GE = (XDMAC_GE_EN0 << SSC_XDMAC_RX_CH);
	
	
}

void ssc_disable_xdmac(void) {
	//Taken from SPI DMA disable function. Might need changing
	xdmac_channel_disable(XDMAC, SSC_XDMAC_RX_CH);
}


int main (void)
{
	//NVIC_DisableIRQ(XDMAC_IRQn);
	//NVIC_DisableIRQ(SPI_IRQn);
	
	WDT->WDT_MR = WDT_MR_WDDIS; //Disables WDT
	
	sysclk_init();
	board_init();
	ioport_set_pin_dir(LED1_PIN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED1_PIN, 1);
	
	irq_initialize_vectors();
	cpu_irq_enable();
	

	
	pmc_enable_periph_clk(ID_XDMAC);
	pmc_enable_periph_clk(ID_SPI0);
	pmc_enable_periph_clk(ID_SSC);
	
	/*Enable XDMAC interrupt */
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_SetPriority( XDMAC_IRQn ,1);
	NVIC_EnableIRQ(XDMAC_IRQn);
	
	/*Enable SPI interrupt */
	NVIC_ClearPendingIRQ(SPI_IRQn);
	NVIC_DisableIRQ(SPI_IRQn);
	NVIC_SetPriority( SPI_IRQn ,0);
	NVIC_EnableIRQ(SPI_IRQn);
	
	//Build copy of RHD2132 reg and values to echo to DAQ
	fill_RHD_reg_value_array();
	
	for (uint8_t i=0;i<32;i++) {
		RHD_reg_value[i] = i*2000;
	}
	
	ssc_init();
	ssc_disable_xdmac();
	ssc_xdmac_configure();
	
	spi_slave_init();
	

	
	
	
	
	ssc_enable_xdmac();
	//ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
	//SPI_SLAVE_BASE->SPI_TDR = 0;
	while (1) {
		
	}
}
//SPI0 Interrupt for detecting events
void SPI0_Handler (void) {
	
	uint32_t spi_int_status = SPI_SLAVE_BASE->SPI_SR; //Gets which interrupts were triggered
	if (spi_int_status & SPI_SR_RDRF) { //New data has been received over SPI
		//ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
		spi_miso_next_data = RHD_reg_value[(SPI_SLAVE_BASE->SPI_RDR)>>8];
		//spi_miso_next_data = spi_handle_command(SPI_SLAVE_BASE->SPI_RDR); //This will figure out what to do and load data into spi_miso_next_data
	}
	if (spi_int_status & SPI_SR_TDRE) {
		//Transmit reg can accept new data. Generally triggered at the start of a new SPI transfer. Data for current transfer is held in shift reg
		//We can load up the data for the next spi cycle. It will stay in TDR until start of next SPI NSS fall
		//ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
		SPI_SLAVE_BASE->SPI_TDR = spi_miso_next_data;
	}
}

//XDMAC Interrupt for ssc rx
void XDMAC_Handler(void)
{
	//NOTE: Need to update for EPHYS
	uint32_t ssc_dma_status = 0;
	ssc_dma_status = XDMAC->XDMAC_CHID[SSC_XDMAC_RX_CH].XDMAC_CIS;
	
	if (ssc_dma_status & XDMAC_CIS_BIS) {
		//Send of SSC packet
		//ioport_set_pin_level(LED0_GPIO, LED0_INACTIVE_LEVEL);
		packet_num = SSC->SSC_RSHR;
		num_packets_received++;
		ssc_disable_xdmac();
		//Might be able to get rid of configure here
		ssc_xdmac_configure(); //Setup DMA for next packet
		ssc_enable_xdmac();
	}
}
