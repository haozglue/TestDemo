/**
 ****************************************************************************************
 *
 * @file i2c_core.c
 *
 * @brief device driver over i2c interface.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.    All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include "global_io.h"
#include "gpio.h"
//#include "periph_setup.h"
#include "i2c_core.h"

// macros
#define SEND_I2C_COMMAND(X) SetWord16(I2C_DATA_CMD_REG, (X))
#define WAIT_WHILE_I2C_FIFO_IS_FULL() while( (GetWord16(I2C_STATUS_REG) & TFNF) == 0 )
#define WAIT_UNTIL_I2C_FIFO_IS_EMPTY() while( (GetWord16(I2C_STATUS_REG) & TFE) == 0 )
#define WAIT_UNTIL_NO_MASTER_ACTIVITY() while( (GetWord16(I2C_STATUS_REG) & MST_ACTIVITY) !=0 )
#define WAIT_FOR_RECEIVED_BYTE() while(GetWord16(I2C_RXFLR_REG) == 0)

//#define ENABLE_I2C_DATA_BREAK_TRANSMISSION_MODE
#define I2C_DATA_BREAK_THRESHOLD 32

/**
 ****************************************************************************************
 * @brief Initialize I2C controller as a master for Device handling.
 ****************************************************************************************
 */
void i2c_init(uint16_t dev_address, uint8_t speed, uint8_t address_mode)
{
    SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                                                              // enable   clock for I2C
    SetWord16(I2C_ENABLE_REG, 0x0);                                                                                             // Disable the I2C controller

    SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE | I2C_RESTART_EN);   // Slave is disabled
    SetBits16(I2C_CON_REG, I2C_SPEED, I2C_FAST /*speed*/);                                              // Set speed
    SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, address_mode);                                     // Set addressing mode

    SetWord16(I2C_TAR_REG, dev_address & 0xFF);                                                                     // Set Slave device address
    SetWord16(I2C_ENABLE_REG, 0x1);                                                                                             // Enable the I2C controller
    while ( (GetWord16(I2C_STATUS_REG) & 0x20) != 0 );                                                      // Wait for I2C master FSM to be IDLE
}

/**
 ****************************************************************************************
 * @brief Disable I2C controller and clock
 ****************************************************************************************
 */
void i2c_release(void)
{
    SetWord16(I2C_ENABLE_REG, 0x0);                                                         // Disable the I2C controller
    SetBits16(CLK_PER_REG, I2C_ENABLE, 0);                                          // Disable clock for I2C
}

/**
****************************************************************************************
* @brief Read single byte from I2C.
*
* @param[in] Memory address to read the byte from.
*
* @return Read byte.
****************************************************************************************
*/
uint8_t i2c_read_byte(uint32_t address)
{
    SEND_I2C_COMMAND(address & 0xFF );  // Set address MSB, write access

    WAIT_WHILE_I2C_FIFO_IS_FULL();                  // Wait if Tx FIFO is full
    SEND_I2C_COMMAND(0x0100);                               // Set R/W bit to 1 (read access)
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                 // Wait until I2C Tx FIFO empty
    WAIT_FOR_RECEIVED_BYTE();                               // Wait for received data
    return(0xFF & GetWord16(I2C_DATA_CMD_REG));      // Get received byte
}

/**
 ****************************************************************************************
 * @brief Read single series of bytes from I2C EEPROM (for driver's internal use)
 *
 * @param[in] p     Memory address to read the series of bytes from (all in the same page)
 * @param[in] size count of bytes to read (must not cross page)
 ****************************************************************************************
 */
static void read_data_single(uint8_t **p, uint32_t address, uint32_t size)
{

#ifdef ENABLE_I2C_DATA_BREAK_TRANSMISSION_MODE
        int j;
        uint32_t tmp_size = size;
        
        SEND_I2C_COMMAND(address & 0xFF );  // Set address MSB, write access
    
        while(1)
        {
            if(tmp_size > I2C_DATA_BREAK_THRESHOLD)
            {
                for (j = 0; j < I2C_DATA_BREAK_THRESHOLD; j++)
                {    
                    WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
                    SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
                }
                
                // Critical section
                GLOBAL_INT_DISABLE();
                
                // Get the received data
                for (j = 0; j < I2C_DATA_BREAK_THRESHOLD; j++)                                         
                {
                    WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
                    *(*p) =(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
                    (*p)++;
                }
                // End of critical section
                GLOBAL_INT_RESTORE();
    
                tmp_size -= I2C_DATA_BREAK_THRESHOLD;
                
            }
            else
            {
                for (j = 0; j < tmp_size; j++)
                {    
                    WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
                    SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
                }
                
                // Critical section
                GLOBAL_INT_DISABLE();
                
                // Get the received data
                for (j = 0; j < tmp_size; j++)                                         
                {
                    WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
                    **p =(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
                    (*p)++;
                }
                // End of critical section
                GLOBAL_INT_RESTORE();
                break;
            }
        }
#else
        int j;
        
        SEND_I2C_COMMAND(address & 0xFF );  // Set address MSB, write access
                    
        for (j = 0; j < size; j++)
        {    
            WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
            SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
        }
        
        // Critical section
        GLOBAL_INT_DISABLE();
        
        // Get the received data
        for (j = 0; j < size; j++)                                         
        {
            WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
            **p =(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
            (*p)++;
        }
        // End of critical section
        GLOBAL_INT_RESTORE();
#endif    

}

/**
 ****************************************************************************************
 * @brief Reads data from I2C EEPROM to memory position of given pointer.
 *
 * @param[in] rd_data_ptr               Read data pointer.
 * @param[in] address                   Starting memory address.
 * @param[in] size                      Size of the data to be read.
 *
 * @return Bytes that were actually read (due to memory size limitation).
 ****************************************************************************************
 */
uint32_t i2c_read_data(uint8_t *rd_data_ptr, uint32_t address, uint32_t size)
{
    uint32_t tmp_size;
    uint32_t bytes_read;

    if (size == 0)
        return(0);

    tmp_size = size;
    bytes_read = size;

    if (tmp_size) {
        read_data_single(&rd_data_ptr, address, tmp_size);
    }

    return(bytes_read);
}

/**
 ****************************************************************************************
 * @brief Write single byte to I2C.
 *
 * @param[in] address        Memory position to write the byte to.
 * @param[in] wr_data        Byte to be written.
 ****************************************************************************************
 */
void i2c_write_byte(uint32_t address, uint8_t wr_data)
{
    SEND_I2C_COMMAND(address & 0xFF);                   // Set address LSB, write access

    WAIT_WHILE_I2C_FIFO_IS_FULL();                      // Wait if I2C Tx FIFO is full
    SEND_I2C_COMMAND(wr_data & 0xFF);                   // Send write data
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                     // Wait until Tx FIFO is empty

    WAIT_UNTIL_NO_MASTER_ACTIVITY();                    // wait until no master activity
}

/**
 ****************************************************************************************
 * @brief Writes page to I2C EEPROM.
 *
 * @param[in] address                Starting address of memory page.
 * @param[in] wr_data_ptr        Pointer to the first of bytes to be written.
 * @param[in] size                      Size of the data to be written (MUST BE LESS OR EQUAL TO I2C_EEPROM_PAGE).
 *
 * @return                                      Count of bytes that were actually written
 ****************************************************************************************
 */
uint16_t i2c_write_data(uint8_t *wr_data_ptr, uint32_t address, uint16_t size)
{
    uint16_t feasible_size;
    uint16_t bytes_written = 0;

    feasible_size = size;                                           // adjust limit accordingly

    // Critical section
    GLOBAL_INT_DISABLE();

    SEND_I2C_COMMAND(address & 0xFF);                   // Set address LSB, write access

    do {
        WAIT_WHILE_I2C_FIFO_IS_FULL();                  // Wait if I2c Tx FIFO is full
        SEND_I2C_COMMAND(*wr_data_ptr & 0xFF);  // Send write data
        wr_data_ptr++;
        feasible_size--;
        bytes_written++;
    } while (feasible_size != 0);

    // End of critical section
    GLOBAL_INT_RESTORE();

    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                     // Wait until Tx FIFO is empty
    WAIT_UNTIL_NO_MASTER_ACTIVITY();                    // Wait until no master activity

    return(bytes_written);
}

