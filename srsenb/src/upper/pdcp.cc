/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2017 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of srsLTE.
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

#include "upper/pdcp.h"
#include "upper/common_enb.h"

#include <iostream>
#include <iomanip>
#include <thread>
#include <unistd.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <inttypes.h>

using namespace std;

struct temp_packet_t{
  uint32_t N_bytes;
  uint32_t lcid;
//  uint8_t msg[1000];
  uint8_t msg[8000];
  
};

struct message {
   long mtype;
//   char mtext[200];
   temp_packet_t  temp;
};

void print_packet_message(srslte::byte_buffer_t *pdu)
{
  for(uint32_t i = 0; i < pdu->N_bytes; i++)
  {
    cout << setw(2) << setfill('0') << hex << (int)(pdu->msg[i]) << " ";
  }
  cout << endl;
  
}

void bar(){
  cout << "bar " << endl;
  sleep(5);
}

namespace srsenb {
  
void pdcp::init(rlc_interface_pdcp* rlc_, rrc_interface_pdcp* rrc_, gtpu_interface_pdcp* gtpu_, srslte::log* pdcp_log_)
{
  cout << "***INIT PDCP ON ENB***" << endl;
  rlc   = rlc_; 
  rrc   = rrc_; 
  gtpu  = gtpu_;
  log_h = pdcp_log_;
  
  pool = srslte::byte_buffer_pool::get_instance();
  
  // Starting thread
  pthread_t msg_queue_thread;
  pthread_create(&msg_queue_thread, NULL, this->read_ue_messageq, NULL);
//  pthread_cancel(msg_queue_thread);
  pthread_exit(NULL);
  
  
  
}

void pdcp::stop()
{
  for(std::map<uint32_t, user_interface>::iterator iter=users.begin(); iter!=users.end(); ++iter) {
    rem_user((uint32_t) iter->first);
  }
  users.clear();
}


/*******************************************************************************
 Read from message queue
*******************************************************************************/
void * pdcp::read_ue_messageq(void*)
{
  srslte::byte_buffer_t pdu_structure;
  srslte::byte_buffer_t *pdu = &pdu_structure;
  message msg;
  
  key_t key = ftok("/tmp/msgq.txt", 'B');
  if (key == -1){
    perror("ftok");
    exit(-1);
  }
  
  int msg_id = msgget(key, 0666 | IPC_CREAT);
  cout << sizeof(message) << endl;
  cout << "Receiving messages..." << endl;
  
  //Always be listening to message queue
  while(true){
    msgrcv(msg_id, &msg, sizeof(message), 1, 0);
    printf("N_bytes : %zu\n" , msg.temp.N_bytes);
    printf("LCID    : %zu\n" , msg.temp.lcid);
    for(uint32_t i = 0; i < msg.temp.N_bytes; i++)
    {
      cout << setw(2) << setfill('0') << hex << (int)(msg.temp.msg[i]) << " ";
      pdu->msg[i] = msg.temp.msg[i];
    }
    cout << endl;
    pdu->N_bytes = msg.temp.N_bytes;
    
    // TODO: pass pdu messages to the next layer
    // FIXME: cannot exit this thread
    this->users.count(10);
    
  }
  return NULL;

}

void pdcp::add_user(uint16_t rnti)
{
  if (users.count(rnti) == 0) {
    srslte::pdcp *obj = new srslte::pdcp;     
    obj->init(&users[rnti].rlc_itf, &users[rnti].rrc_itf, &users[rnti].gtpu_itf, log_h, RB_ID_SRB0, SECURITY_DIRECTION_DOWNLINK);
    users[rnti].rlc_itf.rnti  = rnti;
    users[rnti].gtpu_itf.rnti = rnti;
    users[rnti].rrc_itf.rnti  = rnti;
    
    users[rnti].rrc_itf.rrc   = rrc;
    users[rnti].rlc_itf.rlc   = rlc;
    users[rnti].gtpu_itf.gtpu = gtpu;
    users[rnti].pdcp = obj;
  }
}

void pdcp::rem_user(uint16_t rnti)
{
  if (users.count(rnti)) {
    users[rnti].pdcp->stop();
    delete users[rnti].pdcp; 
    users[rnti].pdcp = NULL;
    users.erase(rnti);
  }
}

void pdcp::add_bearer(uint16_t rnti, uint32_t lcid, srslte::srslte_pdcp_config_t cfg)
{
  if (users.count(rnti)) {
    users[rnti].pdcp->add_bearer(lcid, cfg);
  }
}

void pdcp::reset(uint16_t rnti)
{
  if (users.count(rnti)) {
    users[rnti].pdcp->reset();
  }
}

void pdcp::config_security(uint16_t rnti, uint32_t lcid, uint8_t* k_rrc_enc_, uint8_t* k_rrc_int_, 
                           srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo_, 
                           srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo_)
{
  if (users.count(rnti)) {
    users[rnti].pdcp->config_security(lcid, k_rrc_enc_, k_rrc_int_, cipher_algo_, integ_algo_);
  }
}

void pdcp::write_pdu(uint16_t rnti, uint32_t lcid, srslte::byte_buffer_t* sdu)
{
  if (users.count(rnti)) {
//    cout << "Writing PDU (PDCP)" << endl;
//    cout << "N bytes: " << (int)(sdu -> N_bytes) << endl; 
//    print_packet_message(sdu);
    users[rnti].pdcp->write_pdu(lcid, sdu);
  } else {
    pool->deallocate(sdu);
  }
}

void pdcp::write_sdu(uint16_t rnti, uint32_t lcid, srslte::byte_buffer_t* sdu)
{
//  cout << "Writing SDU" << endl;
//  print_packet_message(sdu);
  cout << "WRITE SDU RNTI: " << rnti << endl;
  if (users.count(rnti)) {
    users[rnti].pdcp->write_sdu(lcid, sdu);
  } else {
    pool->deallocate(sdu);
  }
}

void pdcp::user_interface_gtpu::write_pdu(uint32_t lcid, srslte::byte_buffer_t *pdu)
{
  gtpu->write_pdu(rnti, lcid, pdu);
}

void pdcp::user_interface_rlc::write_sdu(uint32_t lcid, srslte::byte_buffer_t* sdu)
{
  cout << "Write SDU (RLC interface)" << endl;
  print_packet_message(sdu);
  rlc->write_sdu(rnti, lcid, sdu);
}

void pdcp::user_interface_rrc::write_pdu(uint32_t lcid, srslte::byte_buffer_t* pdu)
{
  cout << "Write PDU on RRC interface" << endl;
  print_packet_message(pdu);
  rrc->write_pdu(rnti, lcid, pdu);
  
}

void pdcp::user_interface_rrc::write_pdu_bcch_bch(srslte::byte_buffer_t* pdu)
{
  fprintf(stderr, "Error: Received BCCH from ue=%d\n", rnti);
}

void pdcp::user_interface_rrc::write_pdu_bcch_dlsch(srslte::byte_buffer_t* pdu)
{
  fprintf(stderr, "Error: Received BCCH from ue=%d\n", rnti);
}

void pdcp::user_interface_rrc::write_pdu_pcch(srslte::byte_buffer_t* pdu)
{
  fprintf(stderr, "Error: Received PCCH from ue=%d\n", rnti);
}

std::string pdcp::user_interface_rrc::get_rb_name(uint32_t lcid)
{
  return std::string(rb_id_text[lcid]);
}

}
