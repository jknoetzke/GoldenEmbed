// Copyright 2009 `date +paul@ant%m%y.sbrk.co.uk`
// Released under GPLv3

#define ANT_PRINT 1
#define ANT_EXIT 1

#define hexval(c) ((c >= '0' && c <= '9') ? (c-'0') : ((c&0xdf)-'A'+10))

#define ANTDATALEN 9

#define MAXMSG	16
#define MAXCHAN 8

#define EVENT_RX_ACKNOWLEDGED		0x9b
#define EVENT_RX_BROADCAST		0x9a
#define EVENT_RX_BURST_PACKET		0x9c
#define EVENT_RX_EXT_ACKNOWLEDGED	0x9e
#define EVENT_RX_EXT_BROADCAST		0x9d
#define EVENT_RX_EXT_BURST_PACKET	0x9f
#define EVENT_RX_FAKE_BURST		0xdd
#define EVENT_TRANSFER_TX_COMPLETED	0x05
#define INVALID_MESSAGE			0x28
#define MESG_ACKNOWLEDGED_DATA_ID	0x4f
#define MESG_ASSIGN_CHANNEL_ID		0x42
#define MESG_BROADCAST_DATA_ID		0x4e
#define MESG_BURST_DATA_ID		0x50
#define MESG_CAPABILITIES_ID		0x54
#define MESG_CHANNEL_ID_ID		0x51
#define MESG_CHANNEL_MESG_PERIOD_ID	0x43
#define MESG_CHANNEL_RADIO_FREQ_ID	0x45
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID	0x44
#define MESG_CHANNEL_STATUS_ID		0x52
#define MESG_CLOSE_CHANNEL_ID		0x4c
#define MESG_EXT_ACKNOWLEDGED_DATA_ID	0x5e
#define MESG_EXT_BROADCAST_DATA_ID	0x5d
#define MESG_EXT_BURST_DATA_ID		0x5f
#define MESG_NETWORK_KEY_ID		0x46
#define MESG_RADIO_TX_POWER_ID		0x47
#define MESG_OPEN_CHANNEL_ID		0x4b
#define MESG_OPEN_RX_SCAN_ID		0x5b
#define MESG_REQUEST_ID			0x4d
#define MESG_RESPONSE_EVENT_ID		0x40
#define MESG_RESPONSE_EVENT_SIZE	3
#define MESG_SEARCH_WAVEFORM_ID		0x49
#define MESG_SYSTEM_RESET_ID		0x4a
#define MESG_TX_SYNC			0xa4
#define MESG_UNASSIGN_CHANNEL_ID	0x41
#define RESPONSE_NO_ERROR		0x00
#define EVENT_RX_FAIL			0x02
#define EVENT_TX			0x03
#define MESG_RADIO_CW_MODE_ID		0x48
#define MESG_RADIO_CW_INIT_ID		0x53
#define MESG_ID_LIST_ADD_ID		0x59
#define MESG_ID_LIST_CONFIG_ID		0x5a
#define MESG_SET_LP_SEARCH_TIMEOUT_ID	0x63
#define MESG_SERIAL_NUM_SET_CHANNEL_ID_ID 0x65
#define MESG_RX_EXT_MESGS_ENABLE_ID	0x66
#define MESG_ENABLE_LED_FLASH_ID	0x68


#define AM_NOREPLY	1 << 0
#define AM_REPLIED	1 << 1
#define AM_PREDELAY	1 << 2
#define AM_POSTDELAY	1 << 3
#define AM_POSTRTS	1 << 4
#define AM_XMIT		1 << 5

/*
#pragma pack(1)
struct antmsg {
	u8 flags;
	u8 len;
	u8 msg_id;
	union {
		u8 d[13];
		struct {
			u8 chan;
		} UNASSIGN_CHANNEL;
		struct {
			u8 chan;
			u8 chtype;
			u8 net;
			u8 extass;
		} ASSIGN_CHANNEL;
		struct {
			u8 chan;
			u16 period;
		} CHANNEL_MESG_PERIOD;
		struct {
			u8 chan;
			u8 timeout;
		} CHANNEL_SEARCH_TIMEOUT;
		struct {
			u8 chan;
			u8 freq;
		} CHANNEL_RADIO_FREQ;
		struct {
			u8 chan;
			u8 data[8];
		} DATA;
		struct {
			u8 net;
			u8 key[8];
		} NETWORK_KEY;
		struct {
			u8 zero;
			u8 power;
		} RADIO_TX_POWER;
		struct {
			u8 zero;
			u8 power;
			u8 freq;
		} RADIO_CW_MODE;
		struct {
			u8 zero;
		} SYSTEM_RESET;
		struct {
			u8 chan;
		} OPEN_CHANNEL;
		struct {
			u8 chan;
		} CLOSE_CHANNEL;
		struct {
			u8 chan;
			u8 message;
		} REQUEST;
		struct {
			u8 chan;
			u16 devno;
			u8 devtype;
			u8 trans;
		} CHANNEL_ID;
		struct {
			u8 zero;
		} RADIO_CW_INIT;
		struct {
			u8 chan;
			u16 devno;
			u8 devtype;
			u8 trans;
			u8 listidx;
		} ID_LIST_ADD;
		struct {
			u8 chan;
			u8 size;
			u8 exclude;
		} ID_LIST_CONFIG;
		struct {
			u8 zero;
		} OPEN_RX_SCAN;
		struct {
			u8 chan;
			u8 timeout;
		} SET_LP_SEARCH_TIMEOUT;
		struct {
			u8 chan;
			u8 devtype;
			u8 trans;
		} SERIAL_NUM_SET_CHANNEL_ID;
		struct {
			u8 zero;
			u8 enable;
		} RX_EXT_MESGS_ENABLE;
		struct {
			u8 zero;
			u8 enable;
		} ENABLE_LED_FLASH;
	};
	u8 cookie;
	struct timeval reqtime;
	struct timeval sendtime;
	struct timeval replytime;
	u16 predelay; // delay after sending in ms
	u16 postdelay; // delay before sending in ms
	u8 code;
};
#pragma pack(1)

struct rcvmsg {
	u8 rtype;
	u8 chan;
	u8 data[8];
	struct timeval rcvtime;
};

void ant_debug(unsigned dbg);
void ant_errors(unsigned err_level);
void ant_messages(unsigned msg_level);
void ant_sync(unsigned sync);

struct anth *ant_open(char *devfile, unsigned speed, int rtscts);
void ant_reset(struct anth *);
void ant_req_cap(struct anth *);
void msg_send(struct anth *h, const int msg_id, ...);
struct rcvmsg *ant_getdata(struct anth *h);
void ant_bcdata(struct anth *h, const u8 len, const u8 *);

#ifdef WIN32
#define FDTYPE HANDLE
#define FDCAST SOCKET
#else
#define FDTYPE int
#define FDCAST int
#endif
*/
// vim:se sw=8 ts=8:
