/*
 * CAN module object for generic microcontroller.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @copyright   2004 - 2020 Janez Paternoster
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include "301/CO_driver.h"

#include "mbed.h"

extern "C" {

#define CANID_MASK	0x07FF
#define FLAG_RTR	0x8000

void on_can_tx(CO_CANmodule_t *CANmodule);
void on_can_rx(CO_CANmodule_t *CANmodule);

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr){
    /* Put CAN module in configuration mode */
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */

    CANmodule->CANnormal = true;
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        void                   *CANptr,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

	printf("rxSize: %u\n", rxSize);

    /* Configure object variables */
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = rxSize <= CO_DRIVER_MBED_NUM_HARDWARE_FILTERS;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    for (uint16_t i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }


    /* Configure CAN module registers */


    /* Configure CAN timing */


    /* Configure CAN module hardware filters */
    if (CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else{
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    }


    /* configure CAN interrupt registers */

	static_cast<CAN *>(CANmodule->CANptr)->attach(callback(on_can_tx, CANmodule), CAN::TxIrq);
	static_cast<CAN *>(CANmodule->CANptr)->attach(callback(on_can_rx, CANmodule), CAN::RxIrq);


    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule) {
    if (CANmodule != NULL) {
        /* turn off the module */
		static_cast<CAN *>(CANmodule->CANptr)->attach(nullptr, CAN::TxIrq);
		static_cast<CAN *>(CANmodule->CANptr)->attach(nullptr, CAN::RxIrq);
    }
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if ((CANmodule != NULL) && (object != NULL) && (CANrx_callback != NULL) && (index < CANmodule->rxSize)) {
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /* CAN identifier and CAN mask, bit aligned with CAN module. Different on different microcontrollers. */
		buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters){
			static_cast<CAN *>(CANmodule->CANptr)->filter(ident, mask, CANStandard, index);
        }
    }
    else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if ((CANmodule != NULL) && (index < CANmodule->txSize)) {
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer.
         * Microcontroller specific. */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
		buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}

inline bool_t send_can_message(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {
	CANMessage msg(
			buffer->ident & CANID_MASK,
			buffer->data,
			buffer->DLC,
			(buffer->ident & FLAG_RTR) ? CANRemote : CANData
		);

	return static_cast<CAN *>(CANmodule->CANptr)->write(msg);
}

inline void recv_can_message(CO_CANmodule_t *CANmodule) {
	CANMessage rcvMsg;
	if (!static_cast<CAN *>(CANmodule->CANptr)->read(rcvMsg)) {
		return;
	}

	rcvMsg.id |= rcvMsg.type == CANRemote ? FLAG_RTR : 0x00;

	bool_t rcvMsgMatched = false;
	CO_CANrx_t *buffer = NULL;

	/* Search rxArray form CANmodule for the same CAN-ID. */
	buffer = &CANmodule->rxArray[0];
	for (uint16_t index = CANmodule->rxSize; index > 0U; index--) {
		if(((rcvMsg.id ^ buffer->ident) & buffer->mask) == 0U) {
			rcvMsgMatched = true;
			break;
		}
		buffer++;
	}

	/* Call specific function, which will process the message */
	if (rcvMsgMatched && (buffer != NULL) && (buffer->CANrx_callback != NULL)) {
		buffer->CANrx_callback(buffer->object, static_cast<CO_CANrxMsg_t *>(&rcvMsg));
	}
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer) {

    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    CO_LOCK_CAN_SEND(CANmodule);
    if (send_can_message(CANmodule, buffer)) {
		CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
    /* if no buffer is free, message will be sent by interrupt */
    else {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */

		// There is no mbed API to cancel transmission, so leave it
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0U){
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for (uint16_t i = CANmodule->txSize; i > 0U; i--){
            if (buffer->bufferFull) {
                if (buffer->syncFlag) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);


    if (tpdoDeleted != 0U) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}


/******************************************************************************/

void CO_CANmodule_process(CO_CANmodule_t *CANmodule) {

	uint8_t txErrors = static_cast<CAN *>(CANmodule->CANptr)->tderror();
	uint8_t rxErrors = static_cast<CAN *>(CANmodule->CANptr)->rderror();

    uint32_t err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8);

    if (CANmodule->errOld != err) {
        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (txErrors == 255U) {
            /* bus off */
            status |= CO_CAN_ERRTX_BUS_OFF;
        }
        else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF |
                                CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            /* rx bus warning or passive */
            if (rxErrors >= 128) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRRX_WARNING;
            }

            /* tx bus warning or passive */
            if (txErrors >= 128) {
                status |= CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE;
            } else if (rxErrors >= 96) {
                status |= CO_CAN_ERRTX_WARNING;
            }

            /* if not tx passive clear also overflow */
            if ((status & CO_CAN_ERRTX_PASSIVE) == 0) {
                status &= 0xFFFF ^ CO_CAN_ERRTX_OVERFLOW;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
}


/******************************************************************************/

void on_can_rx(CO_CANmodule_t *CANmodule){

	/* receive interrupt */

	recv_can_message(CANmodule);
}


void on_can_tx(CO_CANmodule_t *CANmodule){
	/* transmit interrupt */

	/* First CAN message (bootup) was sent successfully */
	CANmodule->firstCANtxMessage = false;
	/* clear flag from previous message */
	CANmodule->bufferInhibitFlag = false;
	/* Are there any new messages waiting to be send */
	if (CANmodule->CANtxCount > 0U){

		/* first buffer */
		CO_CANtx_t *buffer = &CANmodule->txArray[0];
		/* search through whole array of pointers to transmit message buffers. */
		uint16_t i;    /* index of transmitting message */
		for (i = CANmodule->txSize; i > 0U; i--){
			/* if message buffer is full, send it. */
			if (buffer->bufferFull) {
				if (send_can_message(CANmodule, buffer)) {
					buffer->bufferFull = false;
					CANmodule->CANtxCount--;
					CANmodule->bufferInhibitFlag = buffer->syncFlag;
					break;                      /* exit for loop */
				}
			}
			buffer++;
		}/* end of for loop */

		/* Clear counter if no more messages */
		if (i == 0U) {
			CANmodule->CANtxCount = 0U;
		}
	}
}

}
