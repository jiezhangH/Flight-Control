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
#include <px4_log.h>

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

}

TAP_ESC_UPLOADER::~TAP_ESC_UPLOADER()
{
	close(_fw_fd);
	deinitialize_uart();
}

int32_t
TAP_ESC_UPLOADER::initialise_firmware_file(const char *filenames[])
{
	const char *filename = NULL;
	int32_t firmware_size;

	/* allow an early abort and look for file first */
	for (unsigned i = 0; filenames[i] != nullptr; i++) {
		_fw_fd = open(filenames[i], O_RDONLY);

		if (_fw_fd < 0) {
			PX4_LOG("failed to open %s", filenames[i]);
			continue;
		}

		PX4_LOG("using firmware from %s", filenames[i]);
		filename = filenames[i];
		break;
	}

	if (filename == NULL) {
		PX4_LOG("no firmware found");
		return -ENOENT;
	}

	struct stat st;

	if (stat(filename, &st) != 0) {
		PX4_LOG("Failed to stat %s - %d\n", filename, (int)errno);
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
	int32_t fw_size;

	fw_size = initialise_firmware_file(filenames);

	if (fw_size < 0) {
		PX4_LOG("initialise firmware file failed");
		return fw_size;
	}

	ret = initialise_uart();

	if (ret < 0) {
		PX4_LOG("initialise uart failed %s");
		return ret;
	}

	/* uploader esc_id(0,1,2,3,4,5) */
	for (unsigned esc_id = 0; esc_id < _esc_counter; esc_id++) {

		/******************************************
		 * first:send sync
		 ******************************************/
		PX4_LOG("uploader esc_id %d...", esc_id);

		/* look for the bootloader, blocking 60 ms,uploader begin esc id0*/
		for (int i = 0; i < SYNC_RETRY_TIMES; i++) {
			ret = sync(esc_id);

			if (ret == OK) {
				break;

			} else {
				usleep(100000);
			}
		}

		if (ret != OK) {
			/* this is immediately fatal */
			PX4_LOG("esc_id %d bootloader not responding", esc_id);
			return -EIO;
		}

		/* do the usual program thing - allow for failure */
		for (unsigned retries = 0; retries < UPLOADER_RETRY_TIMES; retries++) {
			if (retries > 0) {
				PX4_LOG("esc_id %d retrying update...", esc_id);
				ret = sync(esc_id);

				if (ret != OK) {
					/* this is immediately fatal */
					PX4_LOG("esc_id %d bootloader not responding", esc_id);
					return -EIO;
				}
			}

			/******************************************
			* second: get device bootloader revision
			 ******************************************/
			ret = get_device_info(esc_id, PROTO_DEVICE_BL_REV, _bl_rev);

			if (ret == OK) {
				if (_bl_rev <= PROTO_SUPPORT_BL_REV) {
					PX4_LOG("esc_id %d found bootloader revision: %d", esc_id, _bl_rev);

				} else {
					PX4_LOG("esc_id %d found unsupported bootloader revision %d, exiting", esc_id, _bl_rev);
					return EPERM;
				}
			}

			/******************************************
			* third: erase program
			 ******************************************/
			ret = erase(esc_id);

			if (ret != OK) {
				PX4_LOG("esc_id %d %d erase failed", esc_id, ret);
				continue;
			}

			/******************************************
			* fourth: program
			 ******************************************/
			ret = program(esc_id, fw_size);

			if (ret != OK) {
				PX4_LOG("esc_id %d program failed", esc_id);
				continue;
			}

			/******************************************
			* fifth: verify flash crc
			 *****************************************/
			ret = verify_crc(esc_id, fw_size);

			if (ret != OK) {
				PX4_LOG("verify failed");
				continue;
			}

			/******************************************
			* sixth: reboot tap esc
			 *****************************************/
			ret = reboot(esc_id);

			if (ret != OK) {
				PX4_LOG("reboot failed");
				return ret;
			}

			PX4_LOG("esc_id %d uploader complete", esc_id);

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
TAP_ESC_UPLOADER::checkcrc(const char *filenames[])
{
	/*
	  check tap_esc flash CRC against CRC of a file
	 */

	int32_t fw_size;
	int ret = -1;
	fw_size = initialise_firmware_file(filenames);

	if (fw_size < 0) {
		PX4_LOG("initialise firmware file failed");
		return fw_size;
	}

	ret = initialise_uart();

	if (ret < 0) {
		PX4_LOG("initialise uart failed %s");
		return ret;
	}

	/* look for the bootloader, blocking 60 ms,uploader begin esc id0*/
	for (int i = 0; i < SYNC_RETRY_TIMES; i++) {
		ret = sync(0);

		if (ret == OK) {
			break;

		} else {
			usleep(100000);
		}
	}

	if (ret != OK) {
		/* this is immediately fatal */
		PX4_LOG("esc_id0 bootloader not responding");
		return -EIO;
	}

	/* only check esc_id0: compare esc flash crc with .bin file crc */
	ret = verify_crc(0, fw_size);

	if (ret == -EINVAL) {
		PX4_LOG("check CRC is different,will upload tap esc firmware ");
		upload(filenames);

	} else {
		/* reboot tap esc_id0 */
		reboot(0);

		// sleep for enough time for the TAP ESC chip to boot. This makes
		// forceupdate more reliably startup TAP ESC again after update
		up_udelay(20 * 1000);
	}

	exit(0);
}

int
TAP_ESC_UPLOADER::recv_byte_with_timeout(uint8_t *c, unsigned timeout)
{
	struct pollfd fds[1];

	fds[0].fd = _esc_fd;
	fds[0].events = POLLIN;

	/* wait <timout> ms for a character */
	int ret = ::poll(&fds[0], 1, timeout);

	if (ret < 1) {
#ifdef UDEBUG
		PX4_LOG("poll timeout %d", ret);
#endif
		return -ETIMEDOUT;
	}

	read(_esc_fd, c, 1);
#ifdef UDEBUG
	PX4_LOG("recv_bytes 0x%02x", c);
#endif
	return OK;
}

int
TAP_ESC_UPLOADER::read_and_parse_data(unsigned timeout)
{
	uint8_t c = 0;
	int ret = -1;

	/* set a timeout Ms for a first byte */
	ret = recv_byte_with_timeout(&c, timeout);

	if (ret == OK) {
		do {
			/* try to parse the message from esc */
			ret = parse_tap_esc_feedback(c, &_uploader_packet);

			/* parse success all data */
			if (ret == OK) {
				return OK;
			}

			/* read a byte and buffer it with poll and a timeout 2Ms */
			ret = recv_byte_with_timeout(&c, 2);

			/* timeout in 1 is exceeded */
			if (ret == -ETIMEDOUT) {
				PX4_LOG("parse timeout %d data %d", ret, c);
				return ret;
			}

		} while (true);

	}

	return ret;
}

int
TAP_ESC_UPLOADER::parse_tap_esc_feedback(uint8_t decode_data, EscUploaderMessage *packetdata)
{
	static PARSR_ESC_STATE state = HEAD;
	static uint8_t data_index = 0;
	static uint8_t crc_data_cal;

#ifdef UDEBUG
	PX4_LOG("decode data 0x%02x", decode_data);
#endif

	switch (state) {
	case HEAD:
		if (decode_data == 0xFE) {
			packetdata->head = 0xFE; //just_keep the format
			state = LEN;

		}

		break;

	case LEN:
		if (decode_data < sizeof(packetdata->d)) {
			packetdata->len = decode_data;
			state = ID;

		} else {
			state = HEAD;
		}

		break;

	case ID:
		if (decode_data < PROTO_MSG_ID_MAX_NUM) {
			packetdata->msg_id = decode_data;
			data_index = 0;
			state = DATA;

		} else {
			state = HEAD;
		}

		break;

	case DATA:
		packetdata->d.data[data_index++] = decode_data;

		if (data_index >= packetdata->len) {
			crc_data_cal = crc8_esc((uint8_t *)(&packetdata->len), packetdata->len + 2);
			state = CRC;

		}

		break;

	case CRC:
		if (crc_data_cal == decode_data) {
			packetdata->crc_data = decode_data;
			state = HEAD;
			return OK;

		}

		state = HEAD;
		break;

	default:
		state = HEAD;
		break;

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

int
TAP_ESC_UPLOADER::send_packet(EscUploaderMessage &packet, int responder)
{
	if (responder >= 0) {

		if (responder > _esc_counter) {
			return -EINVAL;
		}

		select_responder(_device_mux_map[responder]);
	}

	int packet_len = crc_packet(packet);
	int ret = ::write(_esc_fd, (uint8_t *)&packet.head, packet_len);

#ifdef UDEBUG
	uint8_t *buf = (uint8_t *)&packet;

	for (int i = 0; i < packet_len; i++) {
		PX4_LOG("buf[%d] 0x%02x %d", i, buf[i], ret);
	}

#endif

	if (ret != packet_len) {
#ifdef UDEBUG
		PX4_LOG("TX ERROR: ret: %d, errno: %d", ret, errno);
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

	/* read and parse sync feedback packet, blocking 60ms between esc app jump boot */
	ret = read_and_parse_data(60);

	if (ret != OK) {
		return ret;
	}

	/* check sync feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_LOG("sync don't match myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_GET_SYNC) {
			PX4_LOG("bad sync myID: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				_uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_LOG("sync invalid: don't receive sync invalid: myID: 0x%02x, esc_id: 0x%02x",
			_uploader_packet.d.feedback_packet.myID,
			esc_id);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_LOG("sync failed: don't receive sync failed: myID: 0x%02x, esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID,
			esc_id);
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

	/* read and parse device information feedback packet, blocking 50ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check device information feedback is ok or fail */
	switch (param) {

	case PROTO_DEVICE_BL_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.bootloader_revis_packet.myID != esc_id) {
				PX4_LOG("get device bootloader revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					_uploader_packet.d.bootloader_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.bootloader_revis_packet.version;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("get device bootloader revision failed, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x",
				_uploader_packet.d.bootloader_revis_packet.myID, esc_id,
				_uploader_packet.d.bootloader_revis_packet.version);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("get device bootloader revision invalid, myID: 0x%02x, esc_id: 0x%02x, ver: 0x%02x",
				_uploader_packet.d.bootloader_revis_packet.myID, esc_id,
				_uploader_packet.d.bootloader_revis_packet.version);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_BOARD_ID:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.hardware_id_packet.myID != esc_id) {
				PX4_LOG("get device hardware_id id don't match, myID: 0x%02x, esc_id: 0x%02x",
					_uploader_packet.d.hardware_id_packet.myID,
					esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.hardware_id_packet.targetSystemId;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("get device hardware id failed, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x",
				_uploader_packet.d.hardware_id_packet.myID, esc_id,
				_uploader_packet.d.hardware_id_packet.targetSystemId);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("get device hardware id invalid, myID: 0x%02x, esc_id: 0x%02x, tar: 0x%02x, tar: 0x%02x",
				_uploader_packet.d.hardware_id_packet.myID, esc_id,
				_uploader_packet.d.hardware_id_packet.targetSystemId);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_BOARD_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.hardware_revis_packet.myID != esc_id) {
				PX4_LOG("get device hardware revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					_uploader_packet.d.hardware_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.hardware_revis_packet.boardRev;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("get device hardware revision failed, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x",
				_uploader_packet.d.hardware_revis_packet.myID, esc_id,
				_uploader_packet.d.hardware_revis_packet.boardRev);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("get device hardware revision invalid, myID: 0x%02x, esc_id: 0x%02x, boardRev: 0x%02x",
				_uploader_packet.d.hardware_revis_packet.myID, esc_id,
				_uploader_packet.d.hardware_revis_packet.boardRev);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_FW_SIZE:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.firmware_size_packet.myID != esc_id) {
				PX4_LOG("get device firmware size id don't match, myID: 0x%02x, esc_id: 0x%02x",
					_uploader_packet.d.firmware_size_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.firmware_size_packet.FwSize;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("get device firmware size failed, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x",
				_uploader_packet.d.firmware_size_packet.myID, esc_id,
				_uploader_packet.d.firmware_size_packet.FwSize);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("get device firmware size invalid, myID: 0x%02x, esc_id: 0x%02x, FwSize:  0x%02x",
				_uploader_packet.d.firmware_size_packet.myID, esc_id,
				_uploader_packet.d.firmware_size_packet.FwSize);
			return -EIO;

		}

		break;

	case PROTO_DEVICE_VEC_AREA:
		;
		break;

	case PROTO_DEVICE_FW_REV:
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.firmware_revis_packet.myID != esc_id) {
				PX4_LOG("get device firmware revision id don't match, myID: 0x%02x, esc_id: 0x%02x",
					_uploader_packet.d.firmware_revis_packet.myID, esc_id);
				return -EIO;
			}

			val = _uploader_packet.d.firmware_revis_packet.FwRev;

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("get device firmware revision failed, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x",
				_uploader_packet.d.firmware_revis_packet.myID, esc_id,
				_uploader_packet.d.firmware_revis_packet.FwRev);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("get device firmware revision invalid, myID: 0x%02x,esc_id: 0x%02x, FwRev: 0x%02x",
				_uploader_packet.d.firmware_revis_packet.myID, esc_id,
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
	PX4_LOG("erase...");

	/* send erase packet */
	EscUploaderMessage erase_packet = {0xfe, sizeof(EscbusBootErasePacket), PROTO_CHIP_ERASE};
	erase_packet.d.erase_packet.myID = esc_id;
	send_packet(erase_packet, esc_id);

	/* read and parse erase feedback packet, blocking 500ms */
	ret = read_and_parse_data(500);

	if (ret != OK) {
		return ret;
	}

	/* check erase feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_LOG("erase id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_CHIP_ERASE) {
			PX4_LOG("erase bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				_uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_LOG("erase failed, myID: 0x%02x,esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			_uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_LOG("erase invalid, myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			_uploader_packet.d.feedback_packet.command);
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
		PX4_LOG("Can't allocate program buffer");
		return -ENOMEM;
	}

	//ASSERT((fw_size & 3) == 0);
	//ASSERT((PROG_MULTI_MAX & 3) == 0);

	PX4_LOG("programming %u bytes...", (unsigned)fw_size);

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
			for (uint8_t i = count; i < PROG_MULTI_MAX; i++) {
				file_buf[i] = 0xff;
			}

			count = PROG_MULTI_MAX;
			n 	  = PROG_MULTI_MAX;
		}

		if (count != (ssize_t)n) {
			PX4_LOG("firmware read of %u bytes at %u failed -> %d errno %d",
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

		for (uint8_t i = 0; i < PROG_MULTI_MAX; i++) {
			program_packet.d.program_packet.data[i] = file_buf[i];
		}

		send_packet(program_packet, esc_id);

		/* read and parse program feedback packet, blocking 100ms */
		ret = read_and_parse_data(100);

		if (ret != OK) {
			break;
		}

		/* check program is ok or fail */
		if (_uploader_packet.msg_id == PROTO_OK) {
			if (_uploader_packet.d.feedback_packet.myID != esc_id) {
				PX4_LOG("program id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
				return -EIO;
			}

			if (_uploader_packet.d.feedback_packet.command != PROTO_PROG_MULTI) {
				PX4_LOG("program bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
					_uploader_packet.d.feedback_packet.command);
				return -EIO;
			}

		} else if (_uploader_packet.msg_id == PROTO_FAILED) {
			PX4_LOG("program failed: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
				_uploader_packet.d.feedback_packet.command);
			return -EIO;

		} else if (_uploader_packet.msg_id == PROTO_INVALID) {
			PX4_LOG("program invalid: myID: 0x%02x, esc_id: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				esc_id,
				_uploader_packet.d.feedback_packet.command);
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

	PX4_LOG("verify...");
	lseek(_fw_fd, 0, SEEK_SET);

	ret = get_device_info(esc_id, PROTO_DEVICE_FW_SIZE, fw_size_remote);

	if (ret != OK) {
		PX4_LOG("could not read firmware size");
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
			for (uint8_t i = count; i < PROG_MULTI_MAX; i++) {
				file_buf[i] = fill_blank;
			}

			count = PROG_MULTI_MAX;
			n 	  = PROG_MULTI_MAX;
		}

		if (count != (ssize_t)n) {
			PX4_LOG("firmware read of %u bytes at %u failed -> %d errno %d",
				(unsigned)n,
				(unsigned)bytes_read,
				(int)count,
				(int)errno);
		}

		/* set the rest to 0xff */
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

	/* read and parse feedback CRC from tap esc,blocking 50ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check flash crc feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_crc_packet.myID != esc_id) {
			PX4_LOG("flash crc check id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_crc_packet.command != PROTO_GET_CRC) {
			PX4_LOG("flash crc check bad command, myID: 0x%02x command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				_uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

		crc = _uploader_packet.d.feedback_crc_packet.crc32;

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_LOG("flash crc check failed: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x",
			_uploader_packet.d.feedback_packet.myID,
			esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_LOG("flash crc check invalid: myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x",
			_uploader_packet.d.feedback_packet.myID,
			esc_id, _uploader_packet.d.feedback_packet.command);
		return -EIO;
	}

	/* compare the CRC sum from the IO with the one calculated */
	if (sum != crc) {
		PX4_LOG("CRC wrong: received: %d, expected: %d", crc, sum);
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

	/* read and parse reboot feedback packet, blocking 100ms */
	ret = read_and_parse_data();

	if (ret != OK) {
		return ret;
	}

	/* check reboot feedback is ok or fail */
	if (_uploader_packet.msg_id == PROTO_OK) {
		if (_uploader_packet.d.feedback_packet.myID != esc_id) {
			PX4_LOG("reboot id don't match myID: 0x%02x,esc_id: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id);
			return -EIO;
		}

		if (_uploader_packet.d.feedback_packet.command != PROTO_REBOOT) {
			PX4_LOG("reboot receive bad command, myID: 0x%02x,command: 0x%02x", _uploader_packet.d.feedback_packet.myID,
				_uploader_packet.d.feedback_packet.command);
			return -EIO;
		}

	} else if (_uploader_packet.msg_id == PROTO_FAILED) {
		PX4_LOG("reboot fail, myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			_uploader_packet.d.feedback_packet.command);
		return -EIO;

	} else if (_uploader_packet.msg_id == PROTO_INVALID) {
		PX4_LOG("reboot invalid,myID: 0x%02x, esc_id: 0x%02x, command: 0x%02x", _uploader_packet.d.feedback_packet.myID, esc_id,
			_uploader_packet.d.feedback_packet.command);
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
		PX4_LOG("failed to open uart device!");
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
		PX4_LOG("failed to set baudrate for %s: %d\n", TAP_ESC_SERIAL_DEVICE, termios_state);
		close(_esc_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_esc_fd, TCSANOW, &uart_config)) < 0) {
		PX4_LOG("tcsetattr failed for %s\n", TAP_ESC_SERIAL_DEVICE);
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
