/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for CM55 CPU
*
* Related Document : See README.md
*
********************************************************************************
* (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header File
*******************************************************************************/

#include "cybsp.h"
#include "retarget_io_init.h"
#include "xensiv_bgt60trxx_mtb.h"
#include "xensiv_bgt60trxx_platform.h"
#include "presence_radar_setting.h"
#include "cy_scb_spi.h"
#include "cy_systick.h"
#include "cy_sysclk.h"

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <inttypes.h>
#include "mtb_hal.h"

#include "xensiv_radar_presence.h"

#define RADAR_SPI_CONTROLLER_HW     CYBSP_SPI_CONTROLLER_HW
#define RADAR_SPI_CONTROLLER_IRQ    CYBSP_SPI_CONTROLLER_IRQ
#define RADAR_SPI_CONTROLLER_config CYBSP_SPI_CONTROLLER_config
#define RADAR_SPI_SLAVE_SELECT      CY_SCB_SPI_SLAVE_SELECT1
#define SYSTICK_INTERVAL_1MS        (CY_SYSCLK_IMO_FREQ / 1000U)

#define XENSIV_BGT60TRXX_IRQ_PRIORITY                   (1U) 
#define SPI_INTR_NUM                                    ((IRQn_Type) RADAR_SPI_CONTROLLER_IRQ)
#define SPI_INTR_PRIORITY                               (2U)


/* These sizes and masks must be aligned. */
#define DEFAULT_FIFO_SETTING                            6144

#define RING_BUFFER_MASK                                0x0000FFFF
#define RING_BUFFER_MASK32                              0x00007FFF /* Mask for 32-bits */
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD             (0xFF000000UL)  /* Write addr 7f<<1 | 0x01 */
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS        (17U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS         (16U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS         (9U)
#define RING_BUFFER_SIZE                                0x00010000 /* 64k samples */

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS)
/*******************************************************************************
* Ringbuffer array
*******************************************************************************/
uint16_t bgt60_tensor_ring[ RING_BUFFER_SIZE ];  /* This is the size of the internal fifo. */
uint32_t* ring32;
int bgt60_ring_next_to_write = 0;
int bgt60_ring_next_to_read = 0;
int bgt60_ring_level=0;


cy_stc_scb_spi_context_t SPI_context;
mtb_hal_spi_t CYBSP_SPI_CONTROLLER_hal_obj;

cy_stc_sysint_t irq_cfg;

static volatile bool data_available = false;


/*******************************************************************************
* Preset configuration variables and structs.
*******************************************************************************/
struct xensiv_bgt60trxx_type
{
    uint32_t fifo_addr;
    uint16_t fifo_size;
    xensiv_bgt60trxx_device_t device;
};

struct radar_config radar_configs;

enum spi_state {
    NONE = 0,
    IDLE,
    BURST_PENDING,
    FIFO_READ_PENDING,
    FIFO_READ_DONE
};

enum spi_state sp_state = IDLE;

static void load_presets(void);
static void spi_set_data_width(CySCB_Type* base, uint32_t data_width);
void presence_detection_cb(xensiv_radar_presence_handle_t handle,
                           const xensiv_radar_presence_event_t* event,
                           void *data);

#define RING_BUFFER_SIZE 0x00010000 /* 64k samples */


typedef struct {
    xensiv_radar_presence_event_t last_reported_event;
    bool verbose;
    XENSIV_RADAR_PRESENCE_TIMESTAMP bookmark_timestamp;
}ce_state_s;

ce_state_s ce_app_state;

static const xensiv_radar_presence_config_t default_config =
{
    .bandwidth                         = 460E6,
    .num_samples_per_chirp             = XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP,
    .micro_fft_decimation_enabled      = false,
    .micro_fft_size                    = 128,
    .macro_threshold                   = 0.5f,
    .micro_threshold                   = 12.5f,
    .min_range_bin                     = 1,
    .max_range_bin                     = 5,
    .macro_compare_interval_ms         = 250,
    .macro_movement_validity_ms        = 1000,
    .micro_movement_validity_ms        = 4000,
    .macro_movement_confirmations      = 0,
    .macro_trigger_range               = 1,
    .mode                              = XENSIV_RADAR_PRESENCE_MODE_MICRO_IF_MACRO,
    .macro_fft_bandpass_filter_enabled = false,
    .micro_movement_compare_idx       = 5
};

/*******************************************************************************
* Types
*******************************************************************************/
typedef struct {
    xensiv_bgt60trxx_mtb_t bgt60_obj;
    uint16_t bgt60_buffer0[16384];
    uint16_t bgt60_buffer1[16384];
    bool have_data;
    int skipped_frames;
} dev_bgt60trxx_t;

__attribute__ ((section(".cy_socmem_data"), used)) dev_bgt60trxx_t radar = {0}; 
static float32_t frame[NUM_SAMPLES_PER_FRAME * 2];
static float32_t avg_chirp[XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP];

uint64_t tick_count = 0;
void systick_cb(void)
{
	tick_count++;
}
/*******************************************************************************
* Function Name: mSPI_Interrupt
********************************************************************************
* Summary:
*   SPI_Interrupt
* Parameters:
*   
* Return:
*   
*******************************************************************************/
void mSPI_Interrupt(void)
{
    Cy_SCB_SPI_Interrupt(RADAR_SPI_CONTROLLER_HW, &SPI_context);
}



/*******************************************************************************
* Function Name: xensiv_bgt60trxx_interrupt_handler
********************************************************************************
* Summary:
* This is the interrupt handler to react on sensor indicating the availability
* of new data
*    1. Triggers the radar data manager for buffering radar data into software buffer.
*
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
void xensiv_bgt60trxx_interrupt_handler(void)
{
    data_available = true;
    Cy_GPIO_ClearInterrupt(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_NUM);
    NVIC_ClearPendingIRQ(irq_cfg.intrSrc);
}


/*******************************************************************************
* Function Name: read_radar_data
********************************************************************************
* Summary:
* This is the function for reading radar data using buffering
*
* Parameters:
*  * data: pointer to radar data
*  *num_samples: pointer to number of samples per frame
*  samples_ub: maximum number of samples to be copied at a time from owner task/caller
*
* Return:
*  int32_t: 0 if success
*
*******************************************************************************/
int32_t read_radar_data(uint16_t* data, uint32_t *num_samples, uint32_t samples_ub)
{
    if (xensiv_bgt60trxx_get_fifo_data(&radar.bgt60_obj.dev,
            data,
            NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
    {
        *num_samples = NUM_SAMPLES_PER_FRAME *2; // in bytes

        if (samples_ub < NUM_SAMPLES_PER_FRAME *2)
        {
            xensiv_bgt60trxx_soft_reset(&radar.bgt60_obj.dev,XENSIV_BGT60TRXX_RESET_FIFO );
        }
    }

    return 0;
}

/*******************************************************************************
* Function Name: radar_data_process
********************************************************************************
* Summary:
* This function processes the radar sensor data and feeds the data to DEEPCRAFT
* pre-processor model and returns the results.
*
* Parameters:
*  void
*
* Return:
*  CY_RSLT_SUCCESS if successful, otherwise an error code indicating failure.
*
*******************************************************************************/
bool radar_data_process(void)
{
    int a;
    int words_a_24;
    int fifo_size_32;
    xensiv_bgt60trxx_t* p_device = &radar.bgt60_obj.dev;

    void* iface = p_device->iface;
    xensiv_bgt60trxx_mtb_iface_t* mtb_iface = iface;

    /* Check if SPI transfer is active. If it is active then skip everything. */
    if (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &
                Cy_SCB_SPI_GetTransferStatus(mtb_iface->scb_inst, mtb_iface->spi)))
    {
        return false;
    }

    switch (sp_state)
    {
        case IDLE:
            a = Cy_GPIO_Read(CYBSP_RADAR_INT_PORT,CYBSP_RADAR_INT_NUM);
            if ( a )
            {
                /* Make Chip select */
                xensiv_bgt60trxx_platform_spi_cs_set(iface, false);

                /* Read current fifo state */
                uint32_t* head = (uint32_t*)radar.bgt60_buffer0;
                const xensiv_bgt60trxx_mtb_iface_t* mtb_iface = iface;

                *head = XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD |
                        (p_device->type->fifo_addr << XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS) |   /* Addr 0x60.. */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS) |    /* Read mode */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS);     /* Read until termination. */

                /* Ensure correct byte order for sending the command */
                *head = xensiv_bgt60trxx_platform_word_reverse(*head);

                spi_set_data_width(mtb_iface->scb_inst, 8U);
                Cy_SCB_SetByteMode(mtb_iface->scb_inst, true);

                Cy_SCB_SPI_Transfer(mtb_iface->scb_inst, (uint8_t*)radar.bgt60_buffer0,
                                                                    (uint8_t*)radar.bgt60_buffer1, /* Can be set to NULL, Recieved data is discarded. */
                                                                    4,
                                                                    mtb_iface->spi);

                sp_state = BURST_PENDING;
            }
            break;

        case BURST_PENDING:

            words_a_24 = radar_configs.fifo_int_level & 0xFFFFFFC0;

            spi_set_data_width(mtb_iface->scb_inst, 12U);

            Cy_SCB_SetByteMode(mtb_iface->scb_inst, false);


            Cy_SCB_SPI_Transfer(mtb_iface->scb_inst,
                                            (uint8_t*)radar.bgt60_buffer0, /* Can be set to NULL, Recieved data is discarded. */
                                            (uint8_t*)radar.bgt60_buffer1,
                                            words_a_24,
                                            mtb_iface->spi);

        /* At this point it might be a good idea to check if there is another interrupt from the radar pending. */
        /* If there is it should be possible to swap buffers and issue a second read of data. */
        /* In this state the radar burst mode is still active. */
            sp_state = FIFO_READ_PENDING;
            break;

        case FIFO_READ_PENDING:
            /* Clear the Chip Select. Done when the next polling allows for it. */
            xensiv_bgt60trxx_platform_spi_cs_set(iface, true);
            sp_state = FIFO_READ_DONE;
            break;

        case FIFO_READ_DONE:
            sp_state = IDLE;
            break;

        default:
            break;
    }

    
    if(sp_state == FIFO_READ_DONE)
    {
        radar.have_data = false;

/*********************************************************************************
**  Data copying is now made as a 32-bit data copying to reduce the number of
**  clock cycles needed.
*/
        ring32 = (uint32_t*)bgt60_tensor_ring; /* This will point to a 32k buffer. */
        fifo_size_32 = radar_configs.fifo_int_level >> 1;

        for ( int x = 0; x<fifo_size_32; x++ )
        {
            ring32[bgt60_ring_next_to_write] = ((uint32_t*)(radar.bgt60_buffer1))[x]; /* radar->full_buffer[x]; */
            bgt60_ring_level+=2;        /* Copying 2 samples at a time. */
            bgt60_ring_next_to_write++;
            bgt60_ring_next_to_write&=RING_BUFFER_MASK32;
        }

        const uint32_t samples_per_frame = radar_configs.num_samples_per_frame * 2U;
        if (bgt60_ring_level >= (int)samples_per_frame)
        {
            for (uint32_t i = 0; i < samples_per_frame; i++)
            {
                frame[i] = (float32_t)((int16_t*)bgt60_tensor_ring)[bgt60_ring_next_to_read] / 4096.0f;
                bgt60_ring_level-=1;
                bgt60_ring_next_to_read++;
                bgt60_ring_next_to_read&=RING_BUFFER_MASK;
            }

            return true;
        }
    }
    return false;
}

/*******************************************************************************
* Function Name: radar_init
********************************************************************************
* Summary:
* This function configures the SPI interface, initializes radar and interrupt
* service routine to indicate the availability of radar data.
*
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
cy_rslt_t radar_init(void)
{
    cy_rslt_t result;

    result = Cy_SCB_SPI_Init(RADAR_SPI_CONTROLLER_HW, &RADAR_SPI_CONTROLLER_config, &SPI_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    
    if (result == CY_RSLT_SUCCESS)
    {
        cy_stc_sysint_t spiIntrConfig =
        {
            .intrSrc      = SPI_INTR_NUM,
            .intrPriority = SPI_INTR_PRIORITY,
        };

        Cy_SysInt_Init(&spiIntrConfig, &mSPI_Interrupt);
        NVIC_EnableIRQ(SPI_INTR_NUM);

        /* Set active target select to line 0 */
        Cy_SCB_SPI_SetActiveSlaveSelect(RADAR_SPI_CONTROLLER_HW, RADAR_SPI_SLAVE_SELECT);
        /* Enable SPI Controller block. */
        Cy_SCB_SPI_Enable(RADAR_SPI_CONTROLLER_HW);
    }


    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    load_presets();


    radar.have_data = false;
    radar.skipped_frames = 0;
    
    /* Enable the RADAR. */
    radar.bgt60_obj.iface.scb_inst = RADAR_SPI_CONTROLLER_HW;
    radar.bgt60_obj.iface.spi = &SPI_context;
    radar.bgt60_obj.iface.sel_port = CYBSP_RSPI_CS_PORT;
    radar.bgt60_obj.iface.sel_pin = CYBSP_RSPI_CS_PIN;
    radar.bgt60_obj.iface.rst_port = CYBSP_RADAR_RESET_PORT;
    radar.bgt60_obj.iface.rst_pin = CYBSP_RADAR_RESET_PIN;
    radar.bgt60_obj.iface.irq_port = CYBSP_RADAR_INT_PORT;
    radar.bgt60_obj.iface.irq_pin = CYBSP_RADAR_INT_PIN;
    radar.bgt60_obj.iface.irq_num = CYBSP_RADAR_INT_IRQ;


    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYBSP_RSPI_MOSI_PORT, CYBSP_RSPI_MOSI_PIN, CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYBSP_RSPI_MOSI_PORT, CYBSP_RSPI_MOSI_PIN, CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYBSP_RSPI_CLK_PORT, CYBSP_RSPI_CLK_PIN, CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYBSP_RSPI_CLK_PORT, CYBSP_RSPI_CLK_PIN, CY_GPIO_DRIVE_1_8);

    irq_cfg.intrSrc = (IRQn_Type)radar.bgt60_obj.iface.irq_num;
    irq_cfg.intrPriority = XENSIV_BGT60TRXX_IRQ_PRIORITY;

    result = xensiv_bgt60trxx_mtb_init(&radar.bgt60_obj, radar_configs.register_list, radar_configs.number_of_regs);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /*Initialization uses preset 0*/
    result = xensiv_bgt60trxx_mtb_interrupt_init(&radar.bgt60_obj, radar_configs.num_samples_per_frame);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    Cy_SysInt_Init(&irq_cfg, xensiv_bgt60trxx_interrupt_handler);

    NVIC_ClearPendingIRQ(irq_cfg.intrSrc);
    NVIC_EnableIRQ(irq_cfg.intrSrc);

    Cy_GPIO_ClearInterrupt(CYBSP_RADAR_INT_PORT, CYBSP_RADAR_INT_NUM);
    NVIC_ClearPendingIRQ(irq_cfg.intrSrc);

    Cy_SysLib_Delay(1000);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return false;
    }
  
    if (result == CY_RSLT_SUCCESS)
    {
        xensiv_bgt60trxx_set_fifo_limit(&radar.bgt60_obj.dev, radar_configs.fifo_int_level);
    }
    printf("Radar: Initialized device.\r\n");

    if (xensiv_bgt60trxx_soft_reset(&radar.bgt60_obj.dev, XENSIV_BGT60TRXX_RESET_FIFO) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        printf("Fifo reset error error.\r\n");
    }

    /* Reset the local fifo. */
    bgt60_ring_next_to_write = 0;
    bgt60_ring_next_to_read = 0;
    bgt60_ring_level=0;

    radar.have_data = false;
    if (xensiv_bgt60trxx_start_frame(&radar.bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        printf("Start frame error.\r\n");
    }

    return CY_RSLT_SUCCESS;

}

/*******************************************************************************
 * Platform functions implementation
 ******************************************************************************/
static void spi_set_data_width(CySCB_Type* base, uint32_t data_width)
{
    CY_ASSERT(CY_SCB_SPI_IS_DATA_WIDTH_VALID(data_width));

    CY_REG32_CLR_SET(SCB_TX_CTRL(base),
                     SCB_TX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
    CY_REG32_CLR_SET(SCB_RX_CTRL(base),
                     SCB_RX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
}

/*******************************************************************************
* Summary:
*   This just loads preset from header file constants to enable selection of
*   various radar presets.
* See radar_settings.h for information of the sources of radar settings.
*
*******************************************************************************/
static void load_presets()
{
    radar_configs.start_freq =            XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs.end_freq =              XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs.samples_per_chirp =     XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs.chirps_per_frame =      XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs.rx_antennas =           XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs.tx_antennas =           XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs.sample_rate =           XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs.chirp_repetition_time = XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs.frame_repetition_time = XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs.frame_rate = (int)(0.5 + 1.0/radar_configs.frame_repetition_time);
    radar_configs.num_samples_per_frame =
            radar_configs.samples_per_chirp *
            radar_configs.chirps_per_frame *
            radar_configs.rx_antennas;

    radar_configs.fifo_int_level = DEFAULT_FIFO_SETTING; /* This will equal 75% of the fifo. 12288 samples */

    radar_configs.number_of_regs = XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs.register_list = register_list;

}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM55 application. 
* 
* CM33 application enables the CM55 CPU and then the CM55 CPU enters 
* deep sleep.
* 
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
	 xensiv_radar_presence_handle_t handle;

    /* Initialize the device and board peripherals. */
    result = cybsp_init();

    /* Board init failed. Stop program execution. */
    if (CY_RSLT_SUCCESS != result)
    {
        /* Disable all interrupts. */
        __disable_irq();

        CY_ASSERT(0);

        /* Infinite loop */
        while(true);
    }

    /* Enable global interrupts. */
    __enable_irq();

	 /* Initialize retarget-io middleware */
    init_retarget_io();

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen. */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "PSOC Edge MCU: radar presence "
           "****************** \r\n\n");

    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_IMO, SYSTICK_INTERVAL_1MS);
	Cy_SysTick_SetCallback(0,systick_cb);
	Cy_SysTick_Enable();

	result = radar_init();

    /* Initialization failed */
    if(CY_RSLT_SUCCESS != result)
    {
        /* Failed to initialize properly */
        printf("Radar initialization fail\r\n");
        while(1);
    }
	printf("Radar Start for reading\r\n");
  	
	xensiv_radar_presence_set_malloc_free(malloc,
                                          free);

    if (xensiv_radar_presence_alloc(&handle, &default_config) != 0)
    {
        CY_ASSERT(0);
    }

    xensiv_radar_presence_set_callback(handle, presence_detection_cb, NULL);
    
    for (;;)
    {
        if (!radar_data_process())
        {
            continue;
        }

	    /* calculate the average of the chirps first */
        arm_fill_f32(0, avg_chirp, XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP);

        for (int chirp = 0; chirp < XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME * 2; chirp++)
        {
            arm_add_f32(avg_chirp, &frame[XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP * chirp], avg_chirp, XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP);
        }

        arm_scale_f32(avg_chirp, 1.0f / XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME, avg_chirp, XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP);

        xensiv_radar_presence_process_frame(handle, avg_chirp, tick_count);

    }
}


/*******************************************************************************
* Function Name: presence_detection_cb
********************************************************************************
* Summary:
* This is the callback function o indicate presence/absence events on terminal 
* and LEDs.
* Parameters:
*  void
*
* Return:
*  None
*
*******************************************************************************/
void presence_detection_cb(xensiv_radar_presence_handle_t handle,
                           const xensiv_radar_presence_event_t* event,
                           void *data)
{
    (void)handle;
    (void)data;

    if (!ce_app_state.verbose)
    {
        switch (event->state)
        {
            case XENSIV_RADAR_PRESENCE_STATE_MACRO_PRESENCE:
                printf("[INFO] macro presence %" PRIi32 " %" PRIi32 "\n",
                        event->range_bin,
                        event->timestamp);
                break;

            case XENSIV_RADAR_PRESENCE_STATE_MICRO_PRESENCE:
                printf("[INFO] micro presence %" PRIi32 " %" PRIi32 "\n",
                        event->range_bin,
                        event->timestamp);
                break;

            case XENSIV_RADAR_PRESENCE_STATE_ABSENCE:
                printf("[INFO] absence %" PRIu32 "\n", event->timestamp);
                break;

            default:
                printf("[MSG] ERROR: Unknown reported state in event handling\n");
                break;
        }

    }

    /* save the last reported event state */
    ce_app_state.last_reported_event = *event;
}

/* [] END OF FILE */
