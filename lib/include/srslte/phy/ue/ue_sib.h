/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ue_sib.h
 * Author: redteam
 *
 * Created on August 6, 2019, 2:34 PM
 */

#ifndef UE_SIB_H
#define UE_SIB_H

#include "srslte/config.h"
#include "srslte/srslte.h"


typedef struct SRSLTE_API {
    uint16_t mcc;
    uint16_t mnc;
    uint16_t tac;
    uint16_t cid;
    uint16_t enb_id;
    
} srslte_ue_sib1_t;

SRSLTE_API void srslte_sib1_unpack(uint8_t *data, srslte_ue_sib1_t *sib1){
    
    // MCC
    uint8_t mcc1 = data[2] >> 2;
    uint8_t mcc2 = data[1] & 0x3F;
    uint16_t mcc = (mcc2<<6)|(mcc1);
    sib1->mcc = mcc;
    
    // MNC
    uint16_t mnc = ((data[2] & 0x01)<<7) | (data[3] >>1);
    sib1->mnc = mnc;
    
    // TAC
    uint16_t tac = ((data[4]) << 8) | (data[5]);
    sib1->tac = tac;
    
    // CID
    uint16_t cid = ((data[8] & 0x0F) << 4) | (data[9] >> 4);
    sib1->cid = cid;;
                  
    // eNB ID
    uint16_t enb_id = (data[7] << 4) | (data[8] >> 4);
    sib1->enb_id = enb_id;
    
};

SRSLTE_API void srslte_sib1_fprint(FILE *stream, srslte_ue_sib1_t *sib1){
    
    // Printing Properties
    fprintf(stream, "- MCC:              %03x\n", sib1->mcc);
    fprintf(stream, "- MNC:              %02x\n", sib1->mnc);
    fprintf(stream, "- TAC:              0x%04x\n", sib1->tac);
    fprintf(stream, "- CID:              0x%02x\n", sib1->cid);
    fprintf(stream, "- eNB ID:           0x%03x\n", sib1->enb_id);
};

SRSLTE_API int print_test(void){
    printf("Header test\n");
    return 0;
};  


#endif /* UE_SIB_H */

