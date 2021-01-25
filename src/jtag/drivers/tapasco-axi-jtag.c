#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <sys/socket.h>

#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/bits.h>
#include <helper/replacements.h>
#include <sys/mman.h>
#include <sys/un.h>

// Ignore cast alignment warnings in capnp_c
#pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wcast-align"
# include "capnp_c.h"
#pragma GCC diagnostic pop

#include "tapasco-riscv.capnp.h"

#define SOCKET_LOC "/tmp/riscv-debug.sock"

#define IDCODE  0x1
#define DTMCS   0x10
#define DMI     0x11
#define BYPASS  0x0

#define JTAG_REG_WIDTH  32
#define DMI_REG_WIDTH   41

// TaPaSCo register offset
#define TAP_CTRL        0
#define TAP_IDCODE      4
#define TAP_DTMCS       8
#define TAP_DMI_DATA    12
#define TAP_DMI_ADDR    16

#define AXI_JTAG_MAP_SIZE 4096

#define SOCKET_BUF 1024

#define READ_RSP_SIZE 24
#define WRITE_RSP_SIZE 24

struct axi_jtag {
	int fd;
	void *io_base;
	void *io_vaddr;
	off_t offset;

	char *device_node;
};

static struct axi_jtag axi_jtag_state;
static struct axi_jtag *axi_jtag = &axi_jtag_state;


struct tapasco_intf {
    int socket;
};

static struct tapasco_intf tapasco_intf_state;
static struct tapasco_intf *tapasco_intf = &tapasco_intf_state;

static uint32_t current_ir_reg = IDCODE;

/**
 * Read DTM reg
 */
static uint32_t axi_jtag_read_reg_file(const off_t offset)
{
	return *((volatile uint32_t*)(axi_jtag->io_vaddr + offset));
}

static uint32_t axi_jtag_read_reg(const uint32_t offset, uint64_t* tdo)
{
    // Cap'n Proto stuff
    uint8_t buf[4096];
    ssize_t sz = 0;
    int size;
    uint8_t *buffer = malloc (SOCKET_BUF);

    struct capn c;
    capn_init_malloc(&c);
    capn_ptr cr = capn_root(&c);
    struct capn_segment *cs = cr.seg;

    //LOG_INFO("Sending read for address: %lx!\n", offset);

    if (tapasco_intf->socket == 0)
    {
        LOG_ERROR("Socket is not initialized!\n");
        return ERROR_JTAG_INIT_FAILED;
    }

    struct Request request = {
        .type = Request_RequestType_dtm,
        .isRead = true,
        .addr = offset,
    };

    Request_ptr req_ptr = new_Request(cs);
    write_Request(&request, req_ptr);

    int setp_ret = capn_setp(capn_root(&c), 0, req_ptr.p);
    if (setp_ret != 0){
        LOG_ERROR("Could not create cap'n proto message!\n");
        return ERROR_JTAG_INIT_FAILED;
    }
    sz = capn_write_mem(&c, buf, sizeof(buf), 0 /* packed */);
    capn_free(&c);

    int success = send(tapasco_intf->socket, buf, sz, 0);

    if (success < 1) {
        LOG_ERROR("Error while sending package %d!\n", success);
        return ERROR_JTAG_INIT_FAILED;
    }

    // Read response from socket
    // Size is fixed since recv would read less then full message occasionally
    size = recv(tapasco_intf->socket, buffer, READ_RSP_SIZE, MSG_WAITALL);
    if( size == 0) {
        LOG_ERROR("Received message of size 0!\n");
        return ERROR_JTAG_INIT_FAILED;
    }    

    struct capn rc;
    int init_mem_ret = capn_init_mem(&rc, buffer, size, 0 /* packed */);
    if(init_mem_ret != 0) {
        LOG_ERROR("Could not initialize memory for read response!\n"
                "Size of buffer: %d\n", size);
        return ERROR_JTAG_INIT_FAILED;
    }
    Response_ptr response_ptr;
    struct Response response_struct;
    response_ptr.p = capn_getp(capn_root(&rc), 0 /* offset */, 1 /* resolve */);
    read_Response(&response_struct, response_ptr);

    if (!response_struct.isRead) {
        LOG_ERROR("Reponse to read is not a read response!\n");
        return ERROR_JTAG_INIT_FAILED;
    }
    //LOG_INFO("Received message: %x\n", response_struct.data);

    capn_free(&rc);
    free(buffer);


    *tdo = (uint32_t) response_struct.data;
    return ERROR_OK;
}

/**
 * Write DTM reg
 */
static void axi_jtag_write_reg_file(const off_t offset, const uint32_t val)
{
	*((volatile uint32_t*)(axi_jtag->io_vaddr + offset)) = val;
}

static void axi_jtag_write_reg (const uint32_t offset, const uint32_t val)
{
    // Cap'n Proto stuff
    uint8_t buf[4096];
    ssize_t sz = 0;
    int size;
    uint8_t *buffer = malloc (SOCKET_BUF);

    struct capn c;
    capn_init_malloc(&c);
    capn_ptr cr = capn_root(&c);
    struct capn_segment *cs = cr.seg;

    //LOG_INFO("Sending write request at: 0x%lx to 0x%x\n", offset, val);

    if (tapasco_intf->socket == 0)
    {
        LOG_ERROR("Socket is not initialized!\n");
        return;
    }

    struct Request write_req = {
        .type = Request_RequestType_dtm,
        .isRead = false,
        .addr = offset,
        .data = val,
    };

    Request_ptr req_ptr = new_Request(cs);
    write_Request(&write_req, req_ptr);

    int setp_ret = capn_setp(capn_root(&c), 0, req_ptr.p);
    if (setp_ret != 0){
        LOG_ERROR("Could not create cap'n proto message!\n");
        return;
    }
    sz = capn_write_mem(&c, buf, sizeof(buf), 0 /* packed */);
    capn_free(&c);

    int success = send(tapasco_intf->socket, buf, sz, 0);
    if (success < 1) {
        LOG_ERROR("Error while sending package %d!\n", success);
        return;
    }

    // Read response from socket, response is neccessary to keep in sync
    // Size is fixed since recv would read less then full message occasionally
    size = recv(tapasco_intf->socket, buffer, WRITE_RSP_SIZE, MSG_WAITALL);
    if(size == 0) {
        LOG_ERROR("Received message of size 0!\n");
        return;
    }

    struct capn rc;
    int init_mem_ret = capn_init_mem(&rc, buffer, size, 0 /* packed */);
    if(init_mem_ret != 0) {
        LOG_ERROR("Could not initialize memory for write response!\n"
                "Size of buffer: %d\n", size);
        return;
    }
    Response_ptr response_ptr;
    struct Response response_struct;
    response_ptr.p = capn_getp(capn_root(&rc), 0 /* offset */, 1 /* resolve */);
    read_Response(&response_struct, response_ptr);

    if (response_struct.isRead) {
        LOG_ERROR("Reponse to wite is not a write response!\n");
        return;
    }

    capn_free(&rc);
    free(buffer);
}

static int axi_jtag_execute_scan(struct jtag_command *cmd)
{
	enum scan_type type = jtag_scan_type(cmd->cmd.scan);
	bool ir_scan = cmd->cmd.scan->ir_scan;
    // TODO tdo might need to be larger because dmi is more than 32 bit
	uint64_t tdo = 0;
	uint8_t *buf, *rd_ptr;
	int err, scan_size;
    uint32_t next_register, addr = 0, data = 0, op = 0;
    int error_code = ERROR_OK;

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buf);
	rd_ptr = buf;
	LOG_DEBUG_IO("%s scan type %d %d bits; starts in %s end in %s",
		     (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		     tap_state_name(tap_get_state()),
		     tap_state_name(cmd->cmd.scan->end_state));

    if (type == SCAN_OUT){
        // Scan out means register change is requested
        next_register = cmd->cmd.scan->fields->out_value[0];
        if (next_register == IDCODE || next_register == DTMCS
                || next_register == DMI) {
            current_ir_reg = next_register;
        } else {
            LOG_ERROR("SCAN_OUT to register %x is not supported",
                    next_register);
		    return ERROR_JTAG_QUEUE_FAILED;
        }
    } else if(cmd->cmd.scan->fields->num_bits == JTAG_REG_WIDTH
            || cmd->cmd.scan->fields->num_bits == DMI_REG_WIDTH) {
        switch (current_ir_reg) {
            case IDCODE:
                axi_jtag_read_reg(TAP_IDCODE, &tdo);
                break;
            case DTMCS: 
                axi_jtag_read_reg(TAP_DTMCS, &tdo);
                break;
            case DMI: 
                // Retrieve what OpenOCD tries to do
                data = (cmd->cmd.scan->fields->out_value[0] >> 2) +
                    (cmd->cmd.scan->fields->out_value[1] << 6) +
                    (cmd->cmd.scan->fields->out_value[2] << 14) +
                    (cmd->cmd.scan->fields->out_value[3] << 22) +
                    (cmd->cmd.scan->fields->out_value[4] << 30);
                addr = (cmd->cmd.scan->fields->out_value[4] >> 2);
                op = cmd->cmd.scan->fields->out_value[0] & 0b11;

                if (addr == 0x38) {
                    // Do not show that system bus access is possible
                    // Current SweRV does not support it
                    //tdo = 0x0;
                    //break;
                }
                if (op == 0b01) {
                    axi_jtag_write_reg(TAP_DMI_ADDR, addr);
                    error_code = axi_jtag_read_reg(TAP_DMI_DATA, &tdo);
                    // Insert valid op since it is removed by HW module
                    tdo = tdo << 2;
                } else if (op == 0b10) {
                    // Write DM register
                    axi_jtag_write_reg(TAP_DMI_ADDR, addr);
                    axi_jtag_write_reg(TAP_DMI_DATA, data);
                } else {
                    // With ensure_success flag the dmi reg is read twice once with NOP
                    error_code = axi_jtag_read_reg(TAP_DMI_DATA, &tdo);
                    tdo = tdo << 2;
                }
                break;
            default: tdo = 0xDEADBEEF;
        }
    } else if (ir_scan) {
        // Fake the IR scan for OpenOCD, is not supported
        tdo = 0x181;
    }

    if (error_code != ERROR_OK) {
        LOG_ERROR("Got error during DMI action!\n");
        return error_code;
    }

    // Status is constantly probed
    if (addr != 0x11 && addr != 0x0) {
        LOG_INFO("TAPASCO: DMI: op: %x, data: %x addr: %x tdo: %" PRIx64,
            op, data, addr, tdo);
    }

	buf_set_u64(rd_ptr, 0, 64, tdo);
	rd_ptr += sizeof(uint64_t);


	err = jtag_read_buffer(buf, cmd->cmd.scan);
	if (buf)
		free(buf);

	return err;
}

static int axi_jtag_execute_command(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s: cmd->type: %u", __func__, cmd->type);
	switch (cmd->type) {
		case JTAG_SCAN: {
            return axi_jtag_execute_scan(cmd);
        }
	    case JTAG_RESET:
            // TODO maybe try to do reset here
		    break;
	    case JTAG_TLR_RESET:
            // TODO maybe try to do reset here
            break;
	    default:
		    LOG_ERROR("BUG: Unknown JTAG command type encountered: %u", cmd->type);
		    return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int axi_jtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int ret;

	while (cmd) {
		ret = axi_jtag_execute_command(cmd);

		if (ret != ERROR_OK) {
            LOG_ERROR("Error executing queue\n");
			return ret;
        }

		cmd = cmd->next;
	}

	return ERROR_OK;
}

/**
 * Map the tapasco device file into memory
 */
static int axi_jtag_init(void)
{
    // Connect to Socket
    int tapasco_socket;
    struct sockaddr_un address;

    if((tapasco_socket = socket(PF_UNIX, SOCK_STREAM, 0)) > 0) {
        LOG_INFO("Socket was created\n");
    } else {
        LOG_ERROR("Could not create Socket!\n");
        return ERROR_JTAG_INIT_FAILED;
    }
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, SOCKET_LOC);

    if (connect(tapasco_socket,
                (struct sockaddr *) &address, 
                sizeof (address)) != 0) {
        LOG_ERROR("Could not connect to socket!\n");
        return ERROR_JTAG_INIT_FAILED;
    }

    tapasco_intf->socket = tapasco_socket;


    return ERROR_OK;
}


static int axi_jtag_file(void)
{
    // TODO not longer properly supported, but could still be used

    // File type config
	axi_jtag->fd = open(axi_jtag->device_node, O_RDWR | O_SYNC);
	if (axi_jtag->fd < 0) {
		LOG_ERROR("Failed to open device: %s", axi_jtag->device_node);
		return ERROR_JTAG_INIT_FAILED;
	}
	axi_jtag->io_base = mmap(NULL, AXI_JTAG_MAP_SIZE,
				 PROT_READ | PROT_WRITE, MAP_SHARED,
				 axi_jtag->fd,
				 axi_jtag->offset);
	if (axi_jtag->io_base == MAP_FAILED) {
		LOG_ERROR("Failed to mmap device: %s, offset %lx",
			  axi_jtag->device_node, axi_jtag->offset);
		goto err_mmap;
	}

	axi_jtag->io_vaddr = axi_jtag->io_base + (axi_jtag->offset);

	return ERROR_OK;

err_mmap:
	close(axi_jtag->fd);
	return ERROR_JTAG_INIT_FAILED;
}

static int axi_jtag_quit(void)
{
	int err;

    err = close (tapasco_intf->socket);
	//err = munmap(axi_jtag->io_base, getpagesize());
	if (err)
		goto err_unmap;

	err = close(axi_jtag->fd);
	if (err)
		return err;
	return ERROR_OK;

err_unmap:
	return ERROR_JTAG_QUEUE_FAILED;
}

COMMAND_HANDLER(tapasco_handle_config_command)
{
    if (CMD_ARGC == 2) {
        // Set offset and tlkm device file
		axi_jtag->offset = strtol(CMD_ARGV[1], NULL, 0);
		axi_jtag->device_node = strdup(CMD_ARGV[0]);
	    return ERROR_OK;
    } else {
		return ERROR_COMMAND_SYNTAX_ERROR;
    }
}

COMMAND_HANDLER(tapasco_handle_config_file)
{
    axi_jtag_file();
    axi_jtag_write_reg_file(0, 0);
    axi_jtag_read_reg_file(0);
    return ERROR_OK;
}

static const struct command_registration axi_jtag_command_handlers[] = {
	{
		.name = "tapasco_config",
		.handler = tapasco_handle_config_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure TaPaSCo debug device and offset",
		.usage = "device offset",
	},
    {
		.name = "tapasco_config_file",
		.handler = tapasco_handle_config_file,
		.mode = COMMAND_CONFIG,
		.help = "Do not use not longer supported",
		.usage = "device offset",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const axi_jtag_transports[] = { "jtag", NULL };

static struct jtag_interface wrapper = {
    .execute_queue = &axi_jtag_execute_queue
};

struct adapter_driver tapasco_jtag_interface = {
	.name = "tapascojtag",
	.commands = axi_jtag_command_handlers,
	.transports = axi_jtag_transports,
	.jtag_ops = &wrapper,
	.init = &axi_jtag_init,
	.quit = &axi_jtag_quit,
};
