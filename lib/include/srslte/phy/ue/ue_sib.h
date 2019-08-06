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

SRSLTE_API void srslte_sib1_unpack(uint8_t *x, srslte_ue_sib1_t *sib1){
    
    printf("Successful!\n");
};

SRSLTE_API int print_test(void){
    printf("Header test\n");
    return 0;
};  


#endif /* UE_SIB_H */

