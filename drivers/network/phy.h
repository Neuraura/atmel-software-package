/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2012, Atmel Corporation
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

/** \file */

/** \addtogroup phy_module Ethernet PHY Driver
 *@{
 *  Implement PHY driver, that initializes the PHY to prepare for
 *  ethernet transfer.
 *
 *  \section Usage
 *  -# GMAC related pins and Driver should be initialized at first.
 *  -# Configure PHY via phy_configure(), PHY address is
 *     automatically adjusted by attempt to read.
 *  -# Perform PHY auto negotiate through phy_auto_negotiate(), so
 *     connection established.
 *
 *
 *  Related files:\n
 *  \ref phy.h\n
 *  \ref phy.c\n
 *  \ref gmii.h.\n
 *
 */
/**@}*/

#ifndef _PHY_H
#define _PHY_H

/*---------------------------------------------------------------------------
 *         Headers
 *---------------------------------------------------------------------------*/

#include <stdint.h>

/*---------------------------------------------------------------------------
 *         Definitions
 *---------------------------------------------------------------------------*/

/** Default max retry count */
#define PHY_DEFAULT_RETRIES 300000

/*---------------------------------------------------------------------------
 *         Types
 *---------------------------------------------------------------------------*/

struct _phy_desc {
	Gmac* addr;       /**< GMAC instance */
	uint32_t retries; /**< max retries / timeout */
	uint8_t phy_addr; /**< PHY address (configured) */
};

struct _phy {
	const struct _phy_desc* desc;
	uint8_t phy_addr; /**< PHY address (actual) */
};

/*---------------------------------------------------------------------------
 *         Exported functions
 *---------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

extern bool phy_configure(struct _phy* phy);

extern bool phy_get_id(const struct _phy* phy, uint16_t* id1, uint16_t* id2);

extern bool phy_reset(const struct _phy* phy);

extern bool phy_auto_negotiate(const struct _phy* phy);

extern void phy_dump_registers(const struct _phy* phy);


#ifdef __cplusplus
}
#endif

#endif /* _PHY_H_ */