/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsUE library.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */


#include "srslte/upper/pdcp.h"


// MY INCLUDES
#include <iostream>
#include <iomanip>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <string.h>

using namespace std;


namespace srslte {

pdcp::pdcp()
{}

void pdcp::init(srsue::rlc_interface_pdcp *rlc_, srsue::rrc_interface_pdcp *rrc_, srsue::gw_interface_pdcp *gw_, log *pdcp_log_, uint32_t lcid_, uint8_t direction_)
{
  int mypid = getpid();
  int myppid = getppid();
  cout << "\nMy process ID: " << (int)mypid << endl;
  cout << "Parent ID: " << (int)myppid << endl;
  
  rlc       = rlc_;
  rrc       = rrc_;
  gw        = gw_;
  pdcp_log  = pdcp_log_;
  lcid      = lcid_;
  direction = direction_;

  // Default config
  srslte_pdcp_config_t cnfg;
  cnfg.is_control = false;
  cnfg.is_data = false;
  cnfg.direction = direction_;

  pdcp_array[0].init(rlc, rrc, gw, pdcp_log, lcid, cnfg);
}

void pdcp::stop()
{}

void pdcp::reset()
{
  for(uint32_t i=0;i<SRSLTE_N_RADIO_BEARERS;i++) {
    pdcp_array[i].reset();
  }

  pdcp_array[0].init(rlc, rrc, gw, pdcp_log, lcid, direction);
}
/*******************************************************************************
 Print Messages
*******************************************************************************/
void print_packet_message(byte_buffer_t *pdu)
{
  for(uint32_t i = 0; i < pdu->N_bytes; i++)
  {
    cout << setw(2) << setfill('0') << hex << (int)(pdu->msg[i]) << " ";
  }
  cout << endl;
  
}
/*******************************************************************************
  RRC/GW interface
*******************************************************************************/
bool pdcp::is_drb_enabled(uint32_t lcid)
{
  if(lcid >= SRSLTE_N_RADIO_BEARERS) {
    pdcp_log->error("Radio bearer id must be in [0:%d] - %d\n", SRSLTE_N_RADIO_BEARERS, lcid);
    return false;
  }
  return pdcp_array[lcid].is_active();
}

void pdcp::write_sdu(uint32_t lcid, byte_buffer_t *sdu)
{
  
  int mypid = getpid();
  int myppid = getppid();
  cout << dec << "\nMy process ID: " << mypid << endl;
  cout << dec << "Parent ID: " << myppid << endl;
  
  //WRITE MSG TO SHARED MEMORY
  key_t my_key = ftok("/tmp/shmfile", 65);
  if (my_key == -1){
    perror("Ftok error");
  }
  // Create shared memory segment
  int shmid;
  shmid = shmget(my_key, 1024, 0666 | IPC_CREAT);
  if (shmid == -1){
    perror("Shared memory error");
  }
  // Attach shared mem segment to address space
  uint8_t *shmaddr = (uint8_t*) shmat(shmid, (void*)0, 0);
  *shmaddr = *(sdu->msg);
  // Detach from segment
  shmdt(shmaddr);
  
  
  cout << "Writing SDU" << endl; 
  print_packet_message(sdu);
  if(valid_lcid(lcid))
    pdcp_array[lcid].write_sdu(sdu);
}

void pdcp::add_bearer(uint32_t lcid, srslte_pdcp_config_t cfg)
{
  if(lcid >= SRSLTE_N_RADIO_BEARERS) {
    pdcp_log->error("Radio bearer id must be in [0:%d] - %d\n", SRSLTE_N_RADIO_BEARERS, lcid);
    return;
  }
  if (!pdcp_array[lcid].is_active()) {
    pdcp_array[lcid].init(rlc, rrc, gw, pdcp_log, lcid, cfg);
    pdcp_log->info("Added bearer %s\n", rrc->get_rb_name(lcid).c_str());
  } else {
    pdcp_log->warning("Bearer %s already configured. Reconfiguration not supported\n", rrc->get_rb_name(lcid).c_str());
  }
}

void pdcp::config_security(uint32_t lcid,
                           uint8_t *k_rrc_enc,
                           uint8_t *k_rrc_int,
                           CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                           INTEGRITY_ALGORITHM_ID_ENUM integ_algo)
{
  if(valid_lcid(lcid))
    pdcp_array[lcid].config_security(k_rrc_enc, k_rrc_int, cipher_algo, integ_algo);
}

/*******************************************************************************
  RLC interface
*******************************************************************************/
void pdcp::write_pdu(uint32_t lcid, byte_buffer_t *pdu)
{
  
  int mypid = getpid();
  int myppid = getppid();
  cout << dec << "\nMy process ID: " << mypid << endl;
  cout << dec << "Parent ID: " << myppid << endl;
  
  cout << "Writing PDU" << endl;
  print_packet_message(pdu);
  if(valid_lcid(lcid))
    pdcp_array[lcid].write_pdu(pdu);
}

void pdcp::write_pdu_bcch_bch(byte_buffer_t *sdu)
{
  rrc->write_pdu_bcch_bch(sdu);
}
void pdcp::write_pdu_bcch_dlsch(byte_buffer_t *sdu)
{
  rrc->write_pdu_bcch_dlsch(sdu);
}

void pdcp::write_pdu_pcch(byte_buffer_t *sdu)
{
  rrc->write_pdu_pcch(sdu);
}

/*******************************************************************************
  Helpers
*******************************************************************************/
bool pdcp::valid_lcid(uint32_t lcid)
{
  if(lcid >= SRSLTE_N_RADIO_BEARERS) {
    pdcp_log->error("Radio bearer id must be in [0:%d] - %d", SRSLTE_N_RADIO_BEARERS, lcid);
    return false;
  }
  if(!pdcp_array[lcid].is_active()) {
    pdcp_log->error("PDCP entity for logical channel %d has not been activated\n", lcid);
    return false;
  }
  return true;
}

} // namespace srsue
