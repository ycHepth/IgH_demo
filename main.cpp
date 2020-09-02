#include <iostream>
#include <errno.h>
#include "signal.h"
#include "stdio.h"
#include "sys/resource.h"
#include "sys/time.h"
#include "sys/types.h"
#include "unistd.h"
#include "sys/mman.h"

#include "ecrt.h"

#define TASK_FREQUENCY 30 // Hz
#define TARGET_VELOCITY 110000 // cnt per second
#define PROFILE_VELOCITY 3 // operation mode for 0x6060:0

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

// in profile mode, master does not send new calculated new value at each cycle, instead, the slave computes the
// intermediate position/velocity/torque itself directly at trajectory generator.


// etherCAT

static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc;
static ec_slave_config_state_t sc_state = {};


// PDO
static uint8_t *domain1_pd = NULL;

#define SynapticonSlave 0,0;
#define Synapticon  0x000022D2，0x00000201// vendor id + product id

// Offsets for PDO entries
static struct offset
{
    unsigned int operation_mode;
    unsigned int ctrl_word;
    unsigned int target_velocity;
    unsigned int status_word;
    unsigned int current_position;
    unsigned int current_velocity;
};

const static ec_pdo_entry_reg_t domain1_reg[] = {
    {SynapticonSlave, Synapticon, 0x6040, 0, &offset.ctrl_word},
    {SynapticonSlave, Synapticon, 0x6060, 0, &offset.operation_mode},
    {SynapticonSlave, Synapticon, 0x60FF, 0, &offset.target_velocity},
    {SynapticonSlave, Synapticon, 0x6041, 0, &offset.status_word},
    {SynapticonSlave, Synapticon, 0x6064, 0, &offset.current_position},
    {SynapticonSlave, Synapticon, 0x606C, 0, &offset.current_velocity},
    {}
}

// config PDOs
static ec_pdo_entry_info_t device_pdo_entries[] = {
    /* RxPdo 0x1600 */
    {0x6040, 0x00, 16}, // control word
    {0x6060, 0x00,  8}, // Modes of operation
    {0x60FF, 0x00, 32}, // Target velocity

    /* TxPdo 0x1A00 */
    {0x6041, 0x00, 16}, // Status word
    {0x606C, 0x00, 32}, // actual velocity
    {0x6064, 0x00, 32}  // actual position
};

static ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_OUTPUT, 1, device_pdos + 1, EC_WD_DISABLE},
    {0xFF}
};

// =================== Function =======================


void check_domain1_state(void){
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);

}


int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
