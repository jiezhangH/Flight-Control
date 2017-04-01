/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tap_esc_uploader.cpp
 * firmware uploader for TAP ESC
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <sys/stat.h>
#include <nuttx/arch.h>
#include <crc32.h>
#include <systemlib/px4_macros.h>
#include <drivers/drv_hrt.h>
#include <sys/ioctl.h>

#include "drv_tap_esc.h"
#include "tap_esc_uploader.h"

// define for comms logging
//#define UDEBUG

const uint8_t TAP_ESC_UPLOADER::_crc_table[256] = TAP_ESC_CRC;
const uint8_t TAP_ESC_UPLOADER::_device_mux_map[TAP_ESC_MAX_MOTOR_NUM] = ESC_POS;

TAP_ESC_UPLOADER::TAP_ESC_UPLOADER(uint8_t esc_counter) :
	_esc_fd(-1),
	_fw_fd(-1),
	_esc_counter(esc_counter),
	_bl_rev(0),
	_uploader_packet{}
{
	_uartbuf.head = 0;
	_uartbuf.tail = 0;
	_uartbuf.dat_cnt = 0;
	memset(_uartbuf.esc_feedback_buf, 0, sizeof(_uartbuf.esc_feedback_buf));
}

TAP_ESC_UPLOADER::~TAP_ESC_UPLOADER()
{
	close(_fw_fd);
	deinitialize_uart();
}

size_t
TAP_ESC_UPLOADER::initialise_firmware_file(const char *filenames[])
{
	const char *filename = NULL;
	size_t firmware_size;

	/* allow an early abort and look for file first */
	for (unsigned i = 0; filenames[i] != nullptr; i++) {
		_fw_fd = open(filenames[i], O_RDONLY);

		if (_fw_fd < 0) {
			log("failed to open %s", filenames[i]);
			continue;
		}

		log("using firmware from %s", filenames[i]);
		filename = filenames[i];
		break;
	}

	if (filename == NULL) {
		log("no firmware found");
		return -ENOENT;
	}

	struct stat st;

	if (stat(filename, &st) != 0) {
		log("Failed to stat %s - %d\n", filename, (int)errno);
		return -errno;
	}

	firmware_size = st.st_size;

	if (_fw_fd == -1) {
		return -ENOENT;
	}

	return firmware_size;
}

int
TAP_ESC_UPLOADER::upload(const char *filenames[])
{
	int	ret = -1;
	size_t fw_size;

	fw_size = initialise_firmware_file(filenames);
	initialise_uart();

	/* uploader esc_id(0,1,2,3,4,5) */
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {

/******************************************
 * first:send sync
 ******************************************/
		log("uploader esc_id %d...",esc_id);
		/* look for the bootloader, blocking 120 ms,uploader begin esc id0*/
		for (int i = 0; i < SYNC_RETRY_TIMES; i++) {
			ret = sync(esc_id);

			if (ret == OK) {
				break;

			} else {
				usleep(50000);
			}
		}
		if (ret != OK) {
			/* this is immediately fatal */
			log("esc_id %d bootloader not responding",esc_id);
			return -EIO;
		}

		/* do the usual program thing - allow for failure */
		for (unsigned retries = 0; retries < UPLOADER_RETRY_TIMES; retries++) {
			if (retries > 0) {
				log("esc_id %d retrying update...",esc_id);
				ret = sync(esc_id);
				if (ret != OK) {
					/* this is immediately fatal */
					log("esc_id %d bootloader not responding",esc_id);
					return -EIO;
				}
			}

/******************************************
* second: get device bootloader revision
 ******************************************/
			ret = get_device_info(esc_id, PROTO_DEVICE_BL_REV, _bl_rev);

			if (ret == OK) {
				if (_bl_rev <= PROTO_SUPPORT_BL_REV) {
					log("esc_id %d found bootloader revision: %d", esc_id, _bl_rev);

				} else {
					log("esc_id %d found unsupported bootloader revision %d, exiting", esc_id, _bl_rev);
					return EPERM;
				}
			}

/******************************************
* third: erase program
 ******************************************/
			ret = erase(esc_id);
			if (ret != OK) {
				log("esc_id %d %d erase failed",esc_id,ret);
				continue;
			}

/******************************************
* fourth: program
 ******************************************/
			ret = program(esc_id, fw_size);
			if (ret != OK) {
				log("esc_id %d program failed",esc_id);
				continue;
			}

/******************************************
* fifth: verify flash crc
 *****************************************/
			ret = verify_crc(esc_id, fw_size);
			if (ret != OK) {
				log("verify failed");
				continue;
			}

/******************************************
* sixth: reboot tap esc
 *****************************************/
			ret = reboot(esc_id);
			if (ret != OK) {
				log("reboot failed");
				return ret;
			}

			log("esc_id %d uploader complete",esc_id);

			ret = OK;
			break;
		}
	}

	// sleep for enough time for the TAP ESC chip to boot. This makes
	// forceupdate more reliably startup TAP ESC again after update
	up_udelay(100 * 1000);

	return ret;
}

int
TAP_ESC_UPLOADER::read_data_from_uart(unsigned timeout)
{
	uint8_t tmp_serial_buf[UART_BUFFER_SIZE];
	int err = 0, bytesAvailable = 0;
	struct pollfd fds[1];

	fds[0].fd = _esc_fd;
	fds[0].events = POLLIN;

	/* wait <timout> ms for a character */
	int ret = ::poll(&fds[0], 1, timeout);

	if (ret < 1) {
#ifdef UDEBUG
		log("poll timeout %d", ret);
#endif
		return -ETIMEDOUT;
	}

	err = ioctl(_esc_fd, FIONREAD, (unsigned)&bytesAvailable);
	if ((err != 0) || (bytesAvailable < 11)) {
		usleep(ESC_WAIT_BEFORE_READ * 1000);
	}

	int len = read(_esc_fd, tmp_serial_buf, arraySize(tmp_serial_buf));

	if (len > 0 && (_uartbuf.dat_cnt + len < UART_BUFFER_SIZE)) {
		for (int i = 0; i < len; i++) {
			_uartbuf.esc_feedback_buf[_uartbuf.tail++] = tmp_serial_buf[i];
			_uartbuf.dat_cnt++;
			if (_uartbuf.tail >= UART_BUFFER_SIZE) {
				_uartbuf.tail = 0;
			}
		}
	}

	return OK;
}

int
TAP_ESC_UPLOADER::parse_tap_esc_feedback(ESC_UART_BUF *serial_buf, EscUploaderMessage *packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

	if (serial_buf->dat_cnt > 0) {
		int count = serial_buf->dat_cnt;

		for (int i = 0; i < count; i++) {
#ifdef UDEBUG
			log("decode data[%d] 0x%02x",i,serial_buf->esc_feedback_buf[serial_buf->head]);
#endif
			switch (state) {
			case HEAD:
				if (serial_buf->esc_feedback_buf[serial_buf->head] == 0xFE) {
					packetdata->head = 0xFE; //just_keep the format
					state = LEN;
				}

				break;

			case LEN:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < sizeof(packetdata->d)) {
					packetdata->len = serial_buf->esc_feedback_buf[serial_buf->head];
					state = ID;

				} else {
					state = HEAD;
				}

				break;

			case ID:
				if (serial_buf->esc_feedback_buf[serial_buf->head] < ESCBUS_MSG_ID_MAX_NUM) {
					packetdata->msg_id = serial_buf->esc_feedback_buf[serial_buf->head];
					data_index = 0;
					state = DATA;

				} else {
					state = HEAD;
				}

				break;

			case DATA:
				packetdata->d.data[data_index++] = serial_buf->esc_feedback_buf[serial_buf->head];

				if (data_index >= packetdata->len) {

					crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
					state = CRC;
				}

				break;

			case CRC:
				if (crc_data_cal == serial_buf->esc_feedback_buf[serial_buf->head]) {
					packetdata->crc_data = serial_buf->esc_feedback_buf[serial_buf->head];

					if (++serial_buf->head >= UART_BUFFER_SIZE) {
						serial_buf->head = 0;
					}
					serial_buf->dat_cnt--;
					state = HEAD;
					return OK;
				}

				state = HEAD;
				break;

			default:
				state = HEAD;
				break;

			}

			if (++serial_buf->head >= UART_BUFFER_SIZE) {
				serial_buf->head = 0;
			}

			serial_buf->dat_cnt--;

		}

	}

	return EPROTO;
}

uint8_t
TAP_ESC_UPLOADER::crc8_esc(uint8_t *p, uint8_t len)
{
	uint8_t crc = 0;

	for (uint8_t i = 0; i < len; i++) {
		crc = _crc_table[crc^*p++];
	}

	return crc;
}

uint8_t
TAP_ESC_UPLOADER::crc_packet(EscUploaderMessage &p)
{
	/* Calculate the crc include Len,ID,data */
	p.d.data[p.len] = crc8_esc(&p.len, p.len + 2);
	return p.len + offsetof(EscUploaderMessage, d) + 1;
}

void
TAP_ESC_UPLOADER::select_responder(uint8_t sel)
{
#if defined(GPIO_S0)
	/* _device_mux_map[sel]:Asign the id's to the ESCs to match the mux */
	px4_arch_gpiowrite(GPIO_S0, _device_mux_map[sel] & 1);
	px4_arch_gpiowrite(GPIO_S1, _device_mux_map[sel] & 2);
	px4_arch_gpiowrite(GPIO_S2, _device_mux_map[sel] & 4);
#endif
}

int
TAP_ESC_UPLOADER::send_packet(EscUploaderMessage &packet, int responder)
{
	if (responder >= 0) {

		if (responder > _esc_counter) {
			return -EINVAL;
		}

		select_responder(responder);
	}

	int packet_len = crc_packet(packet);
	int ret = ::write(_esc_fd, (uint8_t *)&packet.head, packet_len);

#ifdef UDEBUG
	uint8_t *buf = (uint8_t *)&packet;

	for (int i=0;i<packet_len;i++) {
		log("buf[%d] 0x%02x %d",i,buf[i],ret);
	}
#endif

	if (ret != packet_len) {
#ifdef UDEBUG
		log("TX ERROR: ret: %d, errno: %d", ret, errno);
#endif
		return errno;
	}

	return OK;
}

int
TAP_ESC_UPLOADER::sync(uint8_t esc_id)
{
	int ret;

	/* send sync packet */
	EscUploaderMessage sync_packet = {0xfe, sizeof(EscbusBootSyncPacket), PROTO_GET_SYNC};
	sync_packet.d.sync_packet.myID = esc_id;
	send_packet(sync_packet, esc_id);

	/* read sync feedback packet, blocking 60ms between esc app jump boot */
	ret = read_data_from_uart(60);

	/* decode feedback packet from esc*/
	ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);
	if (ret != OK) {
		return ret;
	}

	/* check sync feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			log("sync don't match myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}
		if (_uploader_packet.d.feedback_packet.command != PROTO_GET_SYNC) {
			log("bad sync myID: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if(_uploader_packet.msg_id == PROTO_INVALID) {
		log("sync invalid: don't receive sync invalid: myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
		return -EIO;

	} else if(_uploader_packet.msg_id == PROTO_FAILED) {
		log("sync failed: don't receive sync failed: myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
		return -EIO;

	}

	return OK;
}

int
TAP_ESC_UPLOADER::get_device_info(uint8_t esc_id, int param, uint32_t &val)
{
	int ret;

	/* send device information packet */
	EscUploaderMessage device_info_packet = {0xfe, sizeof(EscbusGetDevicePacket), PROTO_GET_DEVICE};
	device_info_packet.d.device_info_packet.myID = esc_id;
	device_info_packet.d.device_info_packet.deviceInfo = param;
	send_packet(device_info_packet, esc_id);

	/* read device information feedback packet, blocking 50ms */
	ret = read_data_from_uart();
	if (ret != OK) {
		return ret;
	}

	/* decode feedback packet from esc*/
	ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);

	if (ret != OK) {
		return ret;
	}

	/* check device information feedback is ok or fail */
	switch (param) {

		case PROTO_DEVICE_BL_REV:
			if (_uploader_packet.msg_id == PROTO_OK) {
				if (_uploader_packet.d.bootloader_revis_packet.myID != esc_id){
					log("get device bootloader revision id don't match, myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.bootloader_revis_packet.myID, esc_id);
					return -EIO;
				}
				val = _uploader_packet.d.bootloader_revis_packet.version;

			} else if (_uploader_packet.msg_id == PROTO_FAILED) {
				log("get device bootloader revision failed, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x", _uploader_packet.d.bootloader_revis_packet.myID, esc_id,
						_uploader_packet.d.bootloader_revis_packet.version);
				return -EIO;

			} else if (_uploader_packet.msg_id == PROTO_INVALID) {
				log("get device bootloader revision invalid, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x", _uploader_packet.d.bootloader_revis_packet.myID, esc_id,
						_uploader_packet.d.bootloader_revis_packet.version);
				return -EIO;

			}
			break;

		case PROTO_DEVICE_BOARD_ID:
			if (_uploader_packet.msg_id == PROTO_OK) {
				if (_uploader_packet.d.hardware_id_packet.myID != esc_id){
					log("get device hardware_id id don't match, myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.hardware_id_packet.myID, esc_id);
					return -EIO;
				}
				val = _uploader_packet.d.hardware_id_packet.targetSystemId;

			} else if (_uploader_packet.msg_id == PROTO_FAILED) {
				log("get device hardware id failed, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x", _uploader_packet.d.hardware_id_packet.myID, esc_id,
						_uploader_packet.d.hardware_id_packet.targetSystemId);
				return -EIO;

			} else if (_uploader_packet.msg_id == PROTO_INVALID) {
				log("get device hardware id invalid, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x", _uploader_packet.d.hardware_id_packet.myID, esc_id,
						_uploader_packet.d.hardware_id_packet.targetSystemId);
				return -EIO;

			}
			break;

		case PROTO_DEVICE_BOARD_REV:
			if (_uploader_packet.msg_id == PROTO_OK) {
				if (_uploader_packet.d.hardware_revis_packet.myID != esc_id){
					log("get device hardware revision id don't match, myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.hardware_revis_packet.myID, esc_id);
					return -EIO;
				}
				val = _uploader_packet.d.hardware_revis_packet.boardRev;

			} else if (_uploader_packet.msg_id == PROTO_FAILED) {
				log("get device hardware revision failed, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x", _uploader_packet.d.hardware_revis_packet.myID, esc_id,
						_uploader_packet.d.hardware_revis_packet.boardRev);
				return -EIO;

			} else if (_uploader_packet.msg_id == PROTO_INVALID) {
				log("get device hardware revision invalid, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x", _uploader_packet.d.hardware_revis_packet.myID, esc_id,
						_uploader_packet.d.hardware_revis_packet.boardRev);
				return -EIO;

			}
			break;

		case PROTO_DEVICE_FW_SIZE:
			if (_uploader_packet.msg_id == PROTO_OK) {
				if (_uploader_packet.d.firmware_size_packet.myID != esc_id){
					log("get device firmware size id don't match, myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.firmware_size_packet.myID, esc_id);
					return -EIO;
				}
				val = _uploader_packet.d.firmware_size_packet.FwSize;

			} else if (_uploader_packet.msg_id == PROTO_FAILED) {
				log("get device firmware size failed, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x", _uploader_packet.d.firmware_size_packet.myID, esc_id,
						_uploader_packet.d.firmware_size_packet.FwSize);
				return -EIO;

			} else if (_uploader_packet.msg_id == PROTO_INVALID) {
				log("get device firmware size invalid, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x", _uploader_packet.d.firmware_size_packet.myID, esc_id,
						_uploader_packet.d.firmware_size_packet.FwSize);
				return -EIO;

			}
			break;

		case PROTO_DEVICE_VEC_AREA:
			;
			break;

		case PROTO_DEVICE_FW_REV:
			if (_uploader_packet.msg_id == PROTO_OK) {
				if (_uploader_packet.d.firmware_revis_packet.myID != esc_id){
					log("get device firmware revision id don't match, myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.firmware_revis_packet.myID, esc_id);
					return -EIO;
				}
				val = _uploader_packet.d.firmware_revis_packet.FwRev;
			} else if (_uploader_packet.msg_id == PROTO_FAILED) {
				log("get device firmware revision failed, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x", _uploader_packet.d.firmware_revis_packet.myID, esc_id,
						_uploader_packet.d.firmware_revis_packet.FwRev);
				return -EIO;

			} else if (_uploader_packet.msg_id == PROTO_INVALID) {
				log("get device firmware revision invalid, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x", _uploader_packet.d.firmware_revis_packet.myID, esc_id,
						_uploader_packet.d.firmware_revis_packet.FwRev);
				return -EIO;

			}
			break;

		default:
			break;
	}

	return OK;
}

int
TAP_ESC_UPLOADER::erase(uint8_t esc_id)
{
	int ret;
	log("erase...");

	/* send erase packet */
	EscUploaderMessage erase_packet = {0xfe, sizeof(EscbusBootErasePacket), PROTO_CHIP_ERASE};
	erase_packet.d.erase_packet.myID = esc_id;
	send_packet(erase_packet, esc_id);

	/* read erase feedback packet, blocking 50ms */
	ret = read_data_from_uart(500);
	if (ret != OK) {
		return ret;
	}
	/* decode feedback packet from esc*/
	ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);
	if (ret != OK) {

		return ret;
	}

	/* check erase feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			log("erase id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}
		if (_uploader_packet.d.feedback_packet.command != PROTO_CHIP_ERASE) {
			log("erase bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		log("erase failed, myID: 0x%02x,esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		log("erase invalid, myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	}

	return OK;
}


static int read_with_retry(int fd, void *buf, size_t n)
{
	int ret;
	uint8_t retries = 0;

	do {
		ret = read(fd, buf, n);
	} while (ret == -1 && retries++ < 100);

	if (retries != 0) {
		printf("read of %u bytes needed %u retries\n",
		       (unsigned)n,
		       (unsigned)retries);
	}

	return ret;
}

int
TAP_ESC_UPLOADER::program(uint8_t esc_id, size_t fw_size)
{
	uint8_t	*file_buf;
	ssize_t count;
	int ret;
	size_t sent = 0;

	file_buf = new uint8_t[PROG_MULTI_MAX];

	if (!file_buf) {
		log("Can't allocate program buffer");
		return -ENOMEM;
	}

	//ASSERT((fw_size & 3) == 0);
	//ASSERT((PROG_MULTI_MAX & 3) == 0);

	log("programming %u bytes...", (unsigned)fw_size);

	ret = lseek(_fw_fd, 0, SEEK_SET);

	while (sent < fw_size) {
		/* get more bytes to program */
		size_t n = fw_size - sent;

		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		/* fill the rest with 0xff */
		if (n < PROG_MULTI_MAX) {
			for( uint8_t i = count; i < PROG_MULTI_MAX; i++) {
				file_buf[i] = 0xff;
			}
			count = PROG_MULTI_MAX;
			n 	  = PROG_MULTI_MAX;
		}

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d",
			    (unsigned)n,
			    (unsigned)sent,
			    (int)count,
			    (int)errno);
			ret = -errno;
			break;
		}

		sent += count;

		/* send program packet */
		EscUploaderMessage program_packet = {0xfe, sizeof(EscbusBootProgPacket), PROTO_PROG_MULTI};
		program_packet.d.program_packet.myID = esc_id;
		for(uint8_t i = 0; i < PROG_MULTI_MAX; i++ ) {
			program_packet.d.program_packet.data[i] = file_buf[i];
		}
		send_packet(program_packet, esc_id);

		/* read program feedback packet, blocking 50ms */
		ret = read_data_from_uart();
		if (ret != OK) {
			break;
		}

		/* decode feedback packet from esc*/
		ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);

		if (ret != OK) {
			break;
		}

		/* check program is ok or fail */
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.feedback_packet.myID != esc_id) {
				log("program id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
				return -EIO;
			}
			if (_uploader_packet.d.feedback_packet.command != PROTO_PROG_MULTI) {
				log("program bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
				return -EIO;
			}

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			log("program failed: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			log("program invalid: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
			return -EIO;

		}

	}

	delete [] file_buf;
	return ret;
}

int
TAP_ESC_UPLOADER::verify_crc(uint8_t esc_id, size_t fw_size_local)
{
	int ret;
	uint8_t	*file_buf;
	ssize_t count;
	uint32_t sum = 0;
	uint32_t bytes_read = 0;
	uint32_t crc = 0;
	uint32_t fw_size_remote;
	uint8_t fill_blank = 0xff;

	file_buf = new uint8_t[PROG_MULTI_MAX];

	log("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	ret = get_device_info(esc_id, PROTO_DEVICE_FW_SIZE, fw_size_remote);

	if (ret != OK) {
		log("could not read firmware size");
		return ret;
	}

	/* read through the firmware file again and calculate the checksum*/
	while (bytes_read < fw_size_local) {
		size_t n = fw_size_local - bytes_read;

		if (n > PROG_MULTI_MAX) {
			n = PROG_MULTI_MAX;
		}

		count = read_with_retry(_fw_fd, file_buf, n);

		/* fill the rest with 0xff */
		if (n < PROG_MULTI_MAX) {
			for( uint8_t i = count; i < PROG_MULTI_MAX; i++) {
				file_buf[i] = fill_blank;
			}
			count = PROG_MULTI_MAX;
			n 	  = PROG_MULTI_MAX;
		}

		if (count != (ssize_t)n) {
			log("firmware read of %u bytes at %u failed -> %d errno %d",
			    (unsigned)n,
			    (unsigned)bytes_read,
			    (int)count,
			    (int)errno);
		}

		/* set the rest to ff */
		if (count == 0) {
			break;
		}

		/* stop if the file cannot be read */
		if (count < 0) {
			return -errno;
		}

		/* calculate crc32 sum */
		sum = crc32part(file_buf, PROG_MULTI_MAX, sum);

		bytes_read += count;
	}

	/* fill the rest with 0xff */
	while (bytes_read < fw_size_remote) {
		sum = crc32part(&fill_blank, sizeof(fill_blank), sum);
		bytes_read += sizeof(fill_blank);
	}

	/* send flash crc packet */
	EscUploaderMessage flash_crc_packet = {0xfe, sizeof(EscbusFlashCRCPacket), PROTO_GET_CRC};
	flash_crc_packet.d.flash_crc_packet.myID = esc_id;
	send_packet(flash_crc_packet, esc_id);

	/* feedback CRC from tap esc,blocking 50ms */
	ret = read_data_from_uart();

	if (ret != OK) {
		return ret;;
	}

	/* decode feedback packet from esc*/
	ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);

	if (ret != OK) {
		return ret;
	}

	/* check flash crc feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_crc_packet.myID != esc_id) {
			log("flash crc check id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}
		if (_uploader_packet.d.feedback_crc_packet.command != PROTO_GET_CRC) {
			log("flash crc check bad command, myID: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}
		crc = _uploader_packet.d.feedback_crc_packet.crc32;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		log("flash crc check failed: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		log("flash crc check invalid: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		log("CRC wrong: received: %d, expected: %d", crc, sum);
		return -EINVAL;
	}

	delete [] file_buf;
	return OK;
}

int
TAP_ESC_UPLOADER::reboot(uint8_t esc_id)
{
	int ret;

	/* send reboot packet */
	EscUploaderMessage reboot_packet = {0xfe, sizeof(EscbusRebootPacket), PROTO_REBOOT};
	reboot_packet.d.reboot_packet.myID = esc_id;
	send_packet(reboot_packet, esc_id);

	/* read reboot feedback packet, blocking 50ms */
	ret = read_data_from_uart();
	if (ret != OK) {
		return ret;
	}

	/* decode feedback packet from esc*/
	ret = parse_tap_esc_feedback(&_uartbuf, &_uploader_packet);

	if (ret != OK) {
		return ret;
	}

	/* check reboot feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			log("reboot id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}
		if (_uploader_packet.d.feedback_packet.command != PROTO_REBOOT) {
			log("reboot receive bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		log("reboot fail, myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		log("reboot invalid,myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	}

	return OK;
}

int
TAP_ESC_UPLOADER::initialise_uart()
{
#ifndef TAP_ESC_SERIAL_DEVICE
#error Must define TAP_ESC_SERIAL_DEVICE in board configuration to support firmware upload
#endif
	/* open uart */
	_esc_fd = open(TAP_ESC_SERIAL_DEVICE, O_RDWR);
	int termios_state = -1;

	if (_esc_fd < 0) {
		log("failed to open uart device!");
		return -1;
	}

	/* set baud rate */
	int speed = 250000;
	struct termios uart_config;
	tcgetattr(_esc_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		log("failed to set baudrate for %s: %d\n", TAP_ESC_SERIAL_DEVICE, termios_state);
		close(_esc_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_esc_fd, TCSANOW, &uart_config)) < 0) {
		log("tcsetattr failed for %s\n", TAP_ESC_SERIAL_DEVICE);
		close(_esc_fd);
		return -1;
	}

	return _esc_fd;
}

void
TAP_ESC_UPLOADER::deinitialize_uart()
{
	close(_esc_fd);
	_esc_fd = -1;
}

void
TAP_ESC_UPLOADER::log(const char *fmt, ...)
{
	va_list	ap;

	printf("[TAP_ESC] ");
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
	printf("\n");
	fflush(stdout);
}
