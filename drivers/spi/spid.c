/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>

#include "callback.h"
#include "dma/dma.h"
#include "errno.h"
#include "irq/irq.h"
#include "mm/cache.h"
#include "peripherals/bus.h"
#ifdef CONFIG_HAVE_FLEXCOM
#include "peripherals/flexcom.h"
#endif
#include "peripherals/pmc.h"
#include "spi/spi.h"
#include "spi/spid.h"
#include "trace.h"

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define SPID_POLLING_THRESHOLD      16

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

CACHE_ALIGNED
static uint32_t _garbage = UINT32_MAX;

/*----------------------------------------------------------------------------
 *        Local functions
 *----------------------------------------------------------------------------*/

#ifdef CONFIG_HAVE_SPI_FIFO

static void _spid_fifo_configure(struct _spi_desc *desc)
{
	desc->fifo.rx.size = get_peripheral_fifo_depth(desc->addr);
	desc->fifo.rx.threshold = desc->fifo.rx.size / 2;
	desc->fifo.tx.size = get_peripheral_fifo_depth(desc->addr);
	desc->fifo.tx.threshold = desc->fifo.tx.size / 2;
	spi_fifo_configure(desc->addr, desc->fifo.tx.threshold, desc->fifo.rx.threshold);
}

static void _spid_wait_tx_fifo_not_full(struct _spi_desc* desc)
{
	if (desc->use_fifo)
		while (desc->addr->SPI_SR & SPI_SR_TXFFF);
}

#endif /* CONFIG_HAVE_SPI_FIFO */

/* forward declaration */
static void _spid_transfer_next_buffer(struct _spi_desc* desc);

static int _spid_dma_rx_callback(void* arg, void* arg2)
{
	struct _spi_desc* desc = (struct _spi_desc*)arg;

	if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)
		cache_invalidate_region(desc->xfer.current->data, desc->xfer.current->size);

	dma_reset_channel(desc->xfer.dma.rx_channel);

	/* process next buffer */
	_spid_transfer_next_buffer(desc);

	return 0;
}

static int _spid_dma_tx_callback(void* arg, void* arg2)
{
	struct _spi_desc* desc = (struct _spi_desc*)arg;

	dma_reset_channel(desc->xfer.dma.tx_channel);

	return 0;
}

static void _spid_transfer_current_buffer_dma(struct _spi_desc* desc)
{
	uint32_t id = get_spi_id_from_addr(desc->addr);
	struct _callback _cb;
	struct _dma_transfer_cfg rx_cfg = {
		.saddr = (void*)&desc->addr->SPI_RDR,
		.daddr = &_garbage,
		.len = desc->xfer.current->size,
	};
	struct _dma_transfer_cfg tx_cfg = {
		.saddr = &_garbage,
		.daddr = (void*)&desc->addr->SPI_TDR,
		.len = desc->xfer.current->size,
	};
	struct _dma_cfg rx_cfg_dma = {
		.incr_saddr = false,
		.incr_daddr = false,
		.loop = false,
		.data_width = DMA_DATA_WIDTH_BYTE,
		.chunk_size = DMA_CHUNK_SIZE_1,
	};
	struct _dma_cfg tx_cfg_dma = {
		.incr_saddr = false,
		.incr_daddr = false,
		.loop = false,
		.data_width = DMA_DATA_WIDTH_BYTE,
		.chunk_size = DMA_CHUNK_SIZE_1,
	};

	if (desc->xfer.current->attr & BUS_BUF_ATTR_TX) {
		if ( desc->stride == 4 )
		{
			// Variable select mode, we use 32bits at a time for sending made up of lastxfer + chip_select + 16bit of data
			// Cache clean region is done in samples.c, to save a few cycles
			// cache_clean_region(desc->variable_peripheral_tx, desc->xfer.current->size * 2);
			tx_cfg.saddr = desc->variable_peripheral_tx;
			tx_cfg_dma.incr_saddr = true;
		}
		else
		{
			cache_clean_region(desc->xfer.current->data, desc->xfer.current->size);
			tx_cfg.saddr = desc->xfer.current->data;
			tx_cfg_dma.incr_saddr = true;
		}
	}

	if (desc->xfer.current->attr & BUS_BUF_ATTR_RX) {
		rx_cfg.daddr = desc->xfer.current->data;
		rx_cfg_dma.incr_daddr = true;
	}

	if ( desc->stride ==  2 || desc->stride == 4)
	{
		// 16 bit (stride 2) mode requires a dma width of 16bits
		rx_cfg_dma.data_width = DMA_DATA_WIDTH_HALF_WORD;	// 16 bits at a time
		rx_cfg.len /= 2;					// Length is the number of groups of 16 bits
	}
	
	if ( desc->stride == 2 )
	{
		// 16 bit (stride 2) mode requires a dma width of 16bits
		tx_cfg_dma.data_width = DMA_DATA_WIDTH_HALF_WORD;	// 16 bits at a time
		tx_cfg.len /= 2;					// Length is the number of groups of 16 bits
	}
	else if ( desc->stride == 4 )
	{
		// Variable Peripheral Select - 32 bit (stride 4) mode requires a dma width of 32bits
		tx_cfg_dma.data_width = DMA_DATA_WIDTH_WORD;		// 32 bits at a time
		tx_cfg.len /= 2;					// Length is the number of groups of 32 bits
	}

	//trace_debug("tx_cfg.len: %lu\r\n", tx_cfg.len);
	//trace_debug("rx_cfg.len: %lu\r\n", rx_cfg.len);

	if (!desc->xfer.dma.tx_channel)
		desc->xfer.dma.tx_channel = dma_allocate_channel(DMA_PERIPH_MEMORY, id);
	if (!desc->xfer.dma.rx_channel)
		desc->xfer.dma.rx_channel = dma_allocate_channel(id, DMA_PERIPH_MEMORY);

	dma_reset_channel(desc->xfer.dma.tx_channel);
	dma_configure_transfer(desc->xfer.dma.tx_channel, &tx_cfg_dma, &tx_cfg, 1);
	callback_set(&_cb, _spid_dma_tx_callback, (void*)desc);
	dma_set_callback(desc->xfer.dma.tx_channel, &_cb);

	dma_reset_channel(desc->xfer.dma.rx_channel);
	dma_configure_transfer(desc->xfer.dma.rx_channel, &rx_cfg_dma, &rx_cfg, 1);
	callback_set(&_cb, _spid_dma_rx_callback, (void*)desc);
	dma_set_callback(desc->xfer.dma.rx_channel, &_cb);

	dma_start_transfer(desc->xfer.dma.rx_channel);
	dma_start_transfer(desc->xfer.dma.tx_channel);
}

static void _spid_handler(uint32_t source, void* user_arg)
{
	uint16_t data;
	uint32_t status = 0;
	Spi* addr = get_spi_addr_from_id(source);
	struct _spi_desc *desc = (struct _spi_desc*)user_arg;

	if (!desc->xfer.current || addr != desc->addr) {
		/* async descriptor not found, disable interrupt */
		spi_disable_it(addr, SPI_IDR_RDRF | SPI_IDR_TDRE | SPI_IDR_TXEMPTY);
		return;
	}

	status = spi_get_masked_status(addr);

	if (SPI_STATUS_TDRE(status)) {
#ifdef CONFIG_HAVE_SPI_FIFO
		_spid_wait_tx_fifo_not_full(desc);
#endif /* CONFIG_HAVE_SPI_FIFO */

		if (desc->xfer.current->attr & BUS_BUF_ATTR_TX)
		{
			if ( desc->stride == 2 )	data = *((uint16_t *)&desc->xfer.current->data[desc->xfer.async.tx]);
			else				data = desc->xfer.current->data[desc->xfer.async.tx];
		}
		else
			data = 0xffff;

		spi_write(desc->addr, data);

		desc->xfer.async.tx += desc->stride;

		if (desc->xfer.async.tx >= desc->xfer.current->size) {
			/* current buffer TX complete */
			spi_disable_it(addr, SPI_IDR_TDRE);
		}
	}

	if (SPI_STATUS_RDRF(status)) {
		data = spi_read(desc->addr);

		if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)
		{
			if ( desc->stride == 2 )	*((uint16_t *)&desc->xfer.current->data[desc->xfer.async.rx]) = data;
			else				desc->xfer.current->data[desc->xfer.async.rx] = data;
		}

		desc->xfer.async.rx+=desc->stride;

		if (desc->xfer.async.rx >= desc->xfer.current->size) {
			/* current buffer RX complete */
			spi_disable_it(addr, SPI_IDR_RDRF);
			spi_enable_it(addr, SPI_IER_TXEMPTY);
		}
	}

	if (SPI_STATUS_TXEMPTY(status)) {
		spi_disable_it(addr, SPI_IDR_TXEMPTY);
		_spid_transfer_next_buffer(desc);
	}
}

static void _spid_transfer_current_buffer_polling(struct _spi_desc* desc)
{
	int i;

	uint16_t data;

	if ( desc->stride == 4 )
	{
		uint32_t data2;

		for (i = 0; i < desc->xfer.current->size; i += 2) {
			#ifdef CONFIG_HAVE_SPI_FIFO
				_spid_wait_tx_fifo_not_full(desc);
			#endif /* CONFIG_HAVE_SPI_FIFO */

			data2 = *((uint32_t *)&desc->variable_peripheral_tx[i*2]);
		
			//trace_debug("%d: 0x%08lX\r\n", i, data2);

			data = spi_transfer_variable_slave(desc->addr, data2);

			if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)	*((uint16_t *)&desc->xfer.current->data[i]) = data;
		}
	}
	else if ( desc->stride <= 1 )
	{
		for (i = 0; i < desc->xfer.current->size; ++i) {
			#ifdef CONFIG_HAVE_SPI_FIFO
				_spid_wait_tx_fifo_not_full(desc);
			#endif /* CONFIG_HAVE_SPI_FIFO */

			if (desc->xfer.current->attr & BUS_BUF_ATTR_TX) data = desc->xfer.current->data[i];
			else						data = 0xff;

			data = spi_transfer(desc->addr, data);

			if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)	desc->xfer.current->data[i] = data;
		}
	}
	else
	{
		for (i = 0; i < desc->xfer.current->size; i+=desc->stride) {
			#ifdef CONFIG_HAVE_SPI_FIFO
				_spid_wait_tx_fifo_not_full(desc);
			#endif /* CONFIG_HAVE_SPI_FIFO */

			if (desc->xfer.current->attr & BUS_BUF_ATTR_TX) data = *((uint16_t *)&desc->xfer.current->data[i]);
			else						data = 0xffff;

			data = spi_transfer(desc->addr, data);

			if (desc->xfer.current->attr & BUS_BUF_ATTR_RX)	*((uint16_t *)&desc->xfer.current->data[i]) = data;
		}
	}

	_spid_transfer_next_buffer(desc);
}

static void _spid_transfer_current_buffer_async(struct _spi_desc* desc)
{
	uint32_t id = get_spi_id_from_addr(desc->addr);

	desc->xfer.async.rx = 0;
	desc->xfer.async.tx = 0;

	spi_disable_it(desc->addr, ~0u);
	irq_add_handler(id, _spid_handler, desc);
	irq_enable(id);

	spi_enable_it(desc->addr, SPI_IER_RDRF | SPI_IER_TDRE);
}

static void _spid_transfer_current_buffer(struct _spi_desc* desc)
{
	enum _bus_transfer_mode tmode = (enum _bus_transfer_mode)desc->transfer_mode;

	if (desc->xfer.current->size < SPID_POLLING_THRESHOLD)
		tmode = BUS_TRANSFER_MODE_POLLING;

	switch (tmode) {
	case BUS_TRANSFER_MODE_POLLING:
		_spid_transfer_current_buffer_polling(desc);
		break;

	case BUS_TRANSFER_MODE_ASYNC:
		_spid_transfer_current_buffer_async(desc);
		break;

	case BUS_TRANSFER_MODE_DMA:
		_spid_transfer_current_buffer_dma(desc);
		break;

	default:
		trace_fatal("Unknown SPI transfer mode\r\n");
	}
}

static void _spid_transfer_next_buffer(struct _spi_desc* desc)
{
	if (desc->xfer.current < desc->xfer.last) {
		desc->xfer.current++;

		_spid_transfer_current_buffer(desc);
	} else {
		if (desc->xfer.current->attr & BUS_SPI_BUF_ATTR_RELEASE_CS)
			spi_release_cs(desc->addr);

		desc->xfer.current = NULL;
		mutex_unlock(&desc->mutex);
		callback_call(&desc->xfer.callback, NULL);
	}
}

/*----------------------------------------------------------------------------
 *        Public functions
 *----------------------------------------------------------------------------*/


// Runs through an array of buffers in sequence
// Each buffer has an attribute which is a bitmask of BUS_SPI_BUF_ATTR_RELEASE_CS | BUS_BUF_ATTR_RX | BUS_BUF_ATTR_TX
// It's possible to mix and match, i.e. BUS_BUF_ATTR_TX for buffers[0] followed by BUS_BUF_ATTR_RX for buffers[1] by specifying the bitmask
// It's also possible to specify BUS_BUF_ATTR_TX and BUS_BUF_ATTR_RX in the same buffer, which will implement TX followed by read into the same buffer

int spid_transfer(struct _spi_desc* desc,
		struct _buffer* buffers, int buffer_count,
		struct _callback* cb)
{
	int i;

	if (buffers == NULL)
		return -EINVAL;

	for (i = 0 ; i < buffer_count ; i++) {
		if ((buffers[i].attr & (BUS_BUF_ATTR_TX | BUS_BUF_ATTR_RX)) == 0)
			return -EINVAL;
	}

	if (!mutex_try_lock(&desc->mutex)) {
		trace_error("SPID mutex already locked!\r\n");
		return -EBUSY;
	}

	if ( desc->stride != 4 )	spi_select_cs(desc->addr, desc->chip_select);

	desc->xfer.current = buffers;
	desc->xfer.last = &buffers[buffer_count - 1];
	callback_copy(&desc->xfer.callback, cb);

	_spid_transfer_current_buffer(desc);

	return 0;
}

bool spid_is_busy(struct _spi_desc* desc)
{
	return mutex_is_locked(&desc->mutex);
}

void spid_wait_transfer(struct _spi_desc* desc)
{
	while (spid_is_busy(desc)) {
		if (desc->transfer_mode == BUS_TRANSFER_MODE_DMA)
			dma_poll();
	}
}

int spid_configure(struct _spi_desc* desc)
{
	uint32_t id = get_spi_id_from_addr(desc->addr);

#ifdef CONFIG_HAVE_FLEXCOM
	Flexcom* flexcom = get_flexcom_addr_from_id(id);
	if (flexcom)
		flexcom_select(flexcom, FLEX_MR_OPMODE_SPI);
#endif
	pmc_configure_peripheral(id, NULL, true);
	spi_configure(desc->addr);
	spi_mode_master_enable(desc->addr, true);
#ifdef CONFIG_HAVE_SPI_FIFO
	_spid_fifo_configure(desc);
	if (desc->use_fifo)
		spi_fifo_enable(desc->addr);
#endif

	spi_disable_it(desc->addr, ~0u);
	desc->xfer.dma.tx_channel = 0;
	desc->xfer.dma.rx_channel = 0;

	spi_enable(desc->addr);

	return 0;
}

void spid_configure_cs(struct _spi_desc* desc, uint8_t cs,
		uint32_t bitrate, float delay_dlybs, float delay_dlybct,
		enum _spid_mode mode, uint8_t bits)
{
	if ( bits > 16 || bits < 8 )	trace_error("spid_configure_cs: bits must be 8..16\r\n");

	desc->stride = (bits == 8) ? 1 : 2;

	uint32_t csr = SPI_CSR_BITS(bits - 8) | SPI_CSR_CSAAT;

	switch (mode) {
	case SPID_MODE_0:
		csr |= SPI_CSR_NCPHA;
		break;
	case SPID_MODE_1:
		csr |= 0;
		break;
	case SPID_MODE_2:
		csr |= SPI_CSR_CPOL | SPI_CSR_NCPHA;
		break;
	case SPID_MODE_3:
		csr |= SPI_CSR_CPOL;
		break;
	}

	spi_configure_cs(desc->addr, cs, bitrate, delay_dlybs, delay_dlybct, csr);
}

void spid_change_bits_cs(struct _spi_desc* desc, uint8_t cs, uint8_t bits)
{
	if ( bits > 16 || bits < 8 )	trace_error("spid_change_bits_cs: bits must be 8..16\r\n");

	desc->stride = (bits == 8) ? 1 : 2;

	uint32_t csr = desc->addr->SPI_CSR[cs];
	csr &= ~SPI_CSR_BITS_Msk;
	csr |= SPI_CSR_BITS(bits-8);
	desc->addr->SPI_CSR[cs] = csr;
}

void spid_set_cs_bitrate(struct _spi_desc* desc, uint8_t cs, uint32_t bitrate)
{
	spi_set_cs_bitrate(desc->addr, cs, bitrate);
}

int spid_configure_master(struct _spi_desc* desc, bool master)
{
	spi_disable(desc->addr);
	spi_mode_master_enable(desc->addr, master);
	spi_enable(desc->addr);

	return 0;
}

void spid_fixed_peripheral_select(struct _spi_desc* desc)
{
	uint32_t mr = desc->addr->SPI_MR;
	mr &= ~SPI_MR_PS;
	desc->addr->SPI_MR = mr;
}

void spid_variable_peripheral_select(struct _spi_desc* desc)
{
	desc->addr->SPI_MR |= SPI_MR_PS;
	desc->stride = 4;
}
