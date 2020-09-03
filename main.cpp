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

#define TASK_FREQUENCY 50 // Hz
#define TARGET_VELOCITY 1000 // PUU (synapticon defalut set PUU as rpm (?)
#define TARGET_POSITION 1000 // cnt
#define OP 9 // operation mode for 0x6060:0  (csv mode)

// 1 -- profile position mode
// 3 -- profile velocity mode
// 4 -- Torque profile mode
// 8 -- cyclic sync position mode
// 9 -- cyclic sync velocity mode
// 10-- cyclic sync torque mode

// in profile mode, master does not send new calculated new value at each cycle, instead, the slave computes the
// intermediate position/velocity/torque itself directly at trajectory generator.


// etherCAT object

static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc;
static ec_slave_config_state_t sc_state = {};


//==================== PDOs =========================
static uint8_t *domain1_pd = NULL;

#define SynapticonSlave 0,0
#define Synapticon  0x000022D2,0x00000201// vendor id + product id

// Offsets for PDO entries
static struct
{
    unsigned int operation_mode;
    unsigned int ctrl_word;
    unsigned int target_velocity;
    unsigned int status_word;
    unsigned int current_position;
    unsigned int current_velocity;
    unsigned int target_position;
}offset;

const static ec_pdo_entry_reg_t domain1_reg[] = {
    {SynapticonSlave, Synapticon, 0x6040, 0, &offset.ctrl_word},
    {SynapticonSlave, Synapticon, 0x6060, 0, &offset.operation_mode},
    {SynapticonSlave, Synapticon, 0x60FF, 0, &offset.target_velocity},
    {SynapticonSlave, Synapticon, 0x6041, 0, &offset.status_word},
    {SynapticonSlave, Synapticon, 0x6064, 0, &offset.current_position},
    {SynapticonSlave, Synapticon, 0x606C, 0, &offset.current_velocity},
    {SynapticonSlave, Synapticon, 0x607A, 0, &offset.target_position},
    {}
};

// config PDOs
static ec_pdo_entry_info_t device_pdo_entries[] = {
    /* RxPdo 0x1600 */
    {0x6040, 0x00, 16}, // control word
    {0x6060, 0x00,  8}, // Modes of operation
    {0x60FF, 0x00, 32}, // Target velocity
    {0x607A, 0x00, 32},  // Target position

    /* TxPdo 0x1A00 */
    {0x6041, 0x00, 16}, // Status word
    {0x606C, 0x00, 32}, // actual velocity
    {0x6064, 0x00, 32}  // actual position
};

static ec_pdo_info_t device_pdos[] = {
        /* RxPDO */
        {0x1600,4,device_pdo_entries + 0},
        /* TxPDO */
        {0x1A00,3,device_pdo_entries + 4}
};

static ec_sync_info_t device_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, device_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, device_pdos + 1, EC_WD_DISABLE},
    {0xFF}
};
// ====================================================

// =================== Function =======================


void check_domain1_state(void){
//    std::cout << "start checking domain state. " << std::endl;
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);  // read state of the domain
    if (ds.working_counter != domain1_state.working_counter)
        std::cout << "Domain1 : Wc " << ds.working_counter << std::endl;
    if (ds.wc_state != domain1_state.wc_state)
        std::cout << "Domain1 : state " << ds.wc_state << std::endl;
    domain1_state = ds;
}

void check_master_state(void){
//    std::cout << "start checking master state. " << std::endl;

    ec_master_state_t ms;
    ecrt_master_state(master,&ms); // read state of the master
    if(ms.slaves_responding != master_state.slaves_responding)
        std::cout << ms.slaves_responding << " slave(s)." << std::endl;
    if(ms.al_states != master_state.al_states)
        std::cout << "AL state : " << ms.al_states << std::endl; // applied layout
    if(ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up":"down");
    master_state = ms;
}

void check_slave_config_states(void){
//    std::cout << "start checking slave config state. " << std::endl;
    ec_slave_config_state_t s;
    ecrt_slave_config_state(sc,&s);
    if (s.al_state != sc_state.al_state)
        printf("Slave: State 0x%02X.\n",s.al_state);
    if(s.online != sc_state.online)
        printf("Slave: %s.\n",s.online ? "online" : "offline");
    if(s.operational != sc_state.operational)
        printf("slave: %soperational.\n",s.operational ? "":"Not ");
    sc_state = s;
}

void cyclic_task(){
    static uint16_t command = 0x004F; // used for judge status word
    uint16_t  status;
    // RxP data
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    // check process data state
    check_domain1_state();

    check_master_state();

    check_slave_config_states();

    std::cout << "pos = " << EC_READ_S32(domain1_pd + offset.current_position) << std::endl;
//    std::cout << "target pos = " << EC_READ_S32(domain1_pd + offset.current_position) + TARGET_POSITION << std::endl;
    std::cout << "vel = " << EC_READ_S32(domain1_pd + offset.current_velocity) << std::endl;


    // read state
    status = EC_READ_U16((domain1_pd + offset.status_word));

//    std::cout << (status & command) << std::endl;

    // Enable (0x06 > 0x07 > 0x0F)

    if((status & command) == 0x0040){
        EC_WRITE_U16(domain1_pd + offset.ctrl_word,0x0006);
        EC_WRITE_S8(domain1_pd + offset.operation_mode, OP);
        command = 0x006F;
    }
    else if ((status & command) == 0x0021)
    {
        EC_WRITE_U16(domain1_pd+offset.ctrl_word,0x0007);
        command = 0x006F;
    }

    else if((status & command) == 0x0023)
    {
        EC_WRITE_U16(domain1_pd + offset.ctrl_word,0x000F);
        command - 0x006F;
    }

    else if ((status & command) == 0x0027)
    {
//        std::cout << "Enter velocity assignments. " << std::endl;
        /* csp mode */
//        int32_t pos_now = 0;
//        pos_now = EC_READ_S32(domain1_pd + offset.current_position);
//        EC_WRITE_S32(domain1_pd + offset.target_position, TARGET_POSITION + pos_now);
//        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x001f); // switch on and enable operation

        /* csv mode */
        EC_WRITE_S32(domain1_pd + offset.target_velocity, TARGET_VELOCITY);
        EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x001f); // switch on and enable operation
    }

    // send process data to domain
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
}


int main() {
    std::cout << "Requesting master ... " << std::endl;
    master = ecrt_request_master(0);
    if(!master)
        exit(EXIT_FAILURE);

    domain1 = ecrt_master_create_domain(master);
    if(!domain1)
        exit(EXIT_FAILURE);

    if(!(sc = ecrt_master_slave_config(master,SynapticonSlave,Synapticon)))
    {
        std::cout << "Failed to get slave configuration. " << std::endl;
        exit(EXIT_FAILURE);
    }

    if (ecrt_slave_config_pdos(sc,EC_END,device_syncs))
    {
        std::cout << "Failed to config slave PDOs" << std::endl;
        exit(EXIT_FAILURE);
    } else
        std::cout << "Success to configuring slave PDOs" << std::endl;

    if (ecrt_domain_reg_pdo_entry_list(domain1,domain1_reg))
    {
        std::cout << "PDO entry registration failed." << std::endl;
        exit(EXIT_FAILURE);
    } else
    {
        printf("success to configuring PDO entry.");
        printf("operation mode = %d, ctrl word= %d, target_velocity = %d, status_word = %d, current_velocity = %d \n",
                offset.operation_mode,offset.ctrl_word,offset.target_velocity,offset.status_word,offset.current_velocity);
    }

    std::cout << "Activating master... " << std::endl;

    if(ecrt_master_activate(master))
        exit((EXIT_FAILURE));
    else
        std::cout << "Master activated!" << std::endl;

    if(!(domain1_pd = ecrt_domain_data(domain1)))
        exit(EXIT_FAILURE);

    std::cout << "It is working now." << std::endl;

    while(1)
    {
        usleep(100000/TASK_FREQUENCY);
        cyclic_task();
    }

    return EXIT_SUCCESS;
}
