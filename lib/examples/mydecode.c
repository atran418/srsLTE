/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   mydecode.c
 * Author: Andy Tran
 *
 * Created on July 29, 2019, 12:02 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <assert.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>

//SRSLTE includes
#include "srslte/common/gen_mch_tables.h"
#include "srslte/common/crash_handler.h"
#include "srslte/phy/io/filesink.h"
#include "srslte/srslte.h"
#include "srslte/phy/utils/debug.h"
#include "srslte/phy/ue/ue_sib.h"

#include "srslte/phy/rf/rf.h"
#include "srslte/phy/rf/rf_utils.h"
 
// Initialize cell search configs
cell_search_cfg_t cell_detect_config = {.max_frames_pbch      = SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
                                        .max_frames_pss       = SRSLTE_DEFAULT_MAX_FRAMES_PSS,
                                        .nof_valid_pss_frames = SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
                                        .init_agc             = 0,
                                        .force_tdd            = false};

// Argument Struct:
typedef struct {
  int nof_subframes;
  int cpu_affinity;
  bool disable_plots;
  bool disable_plots_except_constellation;
  bool disable_cfo; 
  uint32_t time_offset; 
  int force_N_id_2;
  uint16_t rnti;
  char *input_file_name;
  int file_offset_time; 
  float file_offset_freq;
  uint32_t file_nof_prb;
  uint32_t file_nof_ports;
  uint32_t file_cell_id;
  bool enable_cfo_ref;
  bool average_subframe;
  char *rf_dev;
  char *rf_args; 
  uint32_t rf_nof_rx_ant; 
  double rf_freq; 
  float rf_gain;
  int net_port; 
  char *net_address; 
  int net_port_signal; 
  char *net_address_signal;
  int decimate;
  int32_t mbsfn_area_id;
  uint8_t  non_mbsfn_region;
  uint8_t  mbsfn_sf_mask;
  int      tdd_special_sf;
  int      sf_config;
  int verbose;
}prog_args_t;

// Default arguments: keep everything the same and learn how to change frequency
void args_default(prog_args_t* args)
{
  args->disable_plots                      = false;
  args->disable_plots_except_constellation = false;
  args->nof_subframes = -1;
  args->rnti = SRSLTE_SIRNTI;
  args->force_N_id_2 = -1; // Pick the best
  args->tdd_special_sf                     = -1;
  args->sf_config                          = -1;
  args->input_file_name = NULL;
  args->disable_cfo                        = false;
  args->time_offset                        = 0;
  args->file_nof_prb                       = 25;
  args->file_nof_ports                     = 1;
  args->file_cell_id                       = 0;
  args->file_offset_time                   = 0;
  args->file_offset_freq = 0;
  args->rf_dev = "";
  args->rf_args = "";
  args->rf_freq = -1.0;
  args->rf_nof_rx_ant = 1;
  args->enable_cfo_ref = false;
  args->average_subframe = false;
#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1.0;
#else
  args->rf_gain = 50.0;
#endif
  args->net_port           = -1;
  args->net_address        = "127.0.0.1";
  args->net_port_signal    = -1;
  args->net_address_signal = "127.0.0.1";
  args->decimate = 0;
  args->cpu_affinity       = -1;
  args->mbsfn_area_id = -1;
  args->non_mbsfn_region   = 2;
  args->mbsfn_sf_mask = 32;
}

void add_frequency(prog_args_t *args, double frequency)
{
    args_default(args);
    // add frequency
    args->rf_freq = frequency;
}


uint8_t *data[SRSLTE_MAX_CODEWORDS];

bool go_exit = false;

void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  } else if (signo == SIGSEGV) {
    exit(1);
  }
}

cf_t *sf_buffer[SRSLTE_MAX_PORTS] = {NULL};


#ifndef DISABLE_RF

int srslte_rf_recv_wrapper(void* h, cf_t* data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t* t)
{
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  void* ptr[SRSLTE_MAX_PORTS];
  for (int i = 0; i < SRSLTE_MAX_PORTS; i++) {
    ptr[i] = data[i];
  }
  return srslte_rf_recv_with_time_multi(h, ptr, nsamples, true, NULL, NULL);
}

double srslte_rf_set_rx_gain_th_wrapper_(void *h, double f) {
  return srslte_rf_set_rx_gain_th((srslte_rf_t*)h, f);
}

#endif

extern float mean_exec_time;

enum receiver_state { DECODE_MIB, DECODE_PDSCH } state;

srslte_cell_t      cell;
srslte_ue_dl_t     ue_dl;
srslte_ue_dl_cfg_t ue_dl_cfg;
srslte_dl_sf_cfg_t dl_sf;
srslte_pdsch_cfg_t pdsch_cfg;
srslte_ue_sync_t   ue_sync;
prog_args_t        prog_args;

uint32_t pkt_errors = 0, pkt_total = 0, nof_detected = 0, pmch_pkt_errors = 0, pmch_pkt_total = 0, nof_trials = 0;

srslte_netsink_t net_sink, net_sink_signal;
/* Useful macros for printing lines which will disappear */

#define PRINT_LINE_INIT()                                                                                              \
//  int        this_nof_lines = 0;                                                                                       \
//  static int prev_nof_lines = 0;
#define PRINT_LINE(_fmt, ...)                                                                                          \
  printf("\033[K" _fmt "\n", ##__VA_ARGS__);                                                                           \
//  this_nof_lines++
//#define PRINT_LINE_RESET_CURSOR() printf("\033[%dA", this_nof_lines); prev_nof_lines = this_nof_lines
//#define PRINT_LINE_ADVANCE_CURSOR() printf("\033[%dB", prev_nof_lines + 1)

int main(int argc, char** argv) {
    printf("Hello srsLTE\n");
    
    // Take user input for downlink frequency
    double frequency;
    printf("Enter downlink frequency: ");
    scanf("%lf", &frequency);
    
    int ret;

#ifndef DISABLE_RF
  srslte_rf_t rf;
#endif

  srslte_debug_handle_crash(argc, argv);
  
  add_frequency(&prog_args, frequency);  

   #ifdef ENABLE_GUI
  if (prog_args.mbsfn_area_id > -1) {
    enable_mbsfn_plot = true;
  }
#endif /* ENABLE_GUI */

  for (int i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
    data[i] = srslte_vec_malloc(sizeof(uint8_t) * 1500 * 8);
    if (!data[i]) {
      ERROR("Allocating data");
      go_exit = true;
    }
  }

  uint8_t mch_table[10];
  bzero(&mch_table[0], sizeof(uint8_t) * 10);
  if (prog_args.mbsfn_area_id > -1) {
    generate_mcch_table(mch_table, prog_args.mbsfn_sf_mask);
  }
  if (prog_args.cpu_affinity > -1) {

    cpu_set_t cpuset;
    pthread_t thread;

    thread = pthread_self();
    for (int i = 0; i < 8; i++) {
      if (((prog_args.cpu_affinity >> i) & 0x01) == 1) {
        printf("Setting pdsch_ue with affinity to core %d\n", i);
        CPU_SET((size_t)i, &cpuset);
      }
      if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        ERROR("Error setting main thread affinity to %d \n", prog_args.cpu_affinity);
        exit(-1);
      }
    }
  }

  if (prog_args.net_port > 0) {
    if (srslte_netsink_init(&net_sink, prog_args.net_address, prog_args.net_port, SRSLTE_NETSINK_UDP)) {
      ERROR("Error initiating UDP socket to %s:%d\n", prog_args.net_address, prog_args.net_port);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink);
  }
  if (prog_args.net_port_signal > 0) {
    if (srslte_netsink_init(
            &net_sink_signal, prog_args.net_address_signal, prog_args.net_port_signal, SRSLTE_NETSINK_UDP)) {
      ERROR("Error initiating UDP socket to %s:%d\n", prog_args.net_address_signal, prog_args.net_port_signal);
      exit(-1);
    }
    srslte_netsink_set_nonblocking(&net_sink_signal);
  }

  float search_cell_cfo = 0;

#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {

    printf("Opening RF device with %d RX antennas...\n", prog_args.rf_nof_rx_ant);
    if (srslte_rf_open_devname(&rf, prog_args.rf_dev, prog_args.rf_args, prog_args.rf_nof_rx_ant)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    /* Set receiver gain */
    if (prog_args.rf_gain > 0) {
      srslte_rf_set_rx_gain(&rf, prog_args.rf_gain);
    } else {
      printf("Starting AGC thread...\n");
      if (srslte_rf_start_gain_thread(&rf, false)) {
        ERROR("Error opening rf\n");
        exit(-1);
      }
      srslte_rf_set_rx_gain(&rf, srslte_rf_get_rx_gain(&rf));
      cell_detect_config.init_agc = srslte_rf_get_rx_gain(&rf);
    }

    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGINT);
    sigprocmask(SIG_UNBLOCK, &sigset, NULL);
    signal(SIGINT, sig_int_handler);

    srslte_rf_set_master_clock_rate(&rf, 30.72e6);

    /* set receiver frequency */
    printf("Tunning receiver to %.3f MHz\n", (prog_args.rf_freq + prog_args.file_offset_freq) / 1000000);
    srslte_rf_set_rx_freq(&rf, prog_args.rf_nof_rx_ant, prog_args.rf_freq + prog_args.file_offset_freq);
    srslte_rf_rx_wait_lo_locked(&rf);

    uint32_t ntrial = 0;
    do {
      ret = rf_search_and_decode_mib(
          &rf, prog_args.rf_nof_rx_ant, &cell_detect_config, prog_args.force_N_id_2, &cell, &search_cell_cfo);
      if (ret < 0) {
        ERROR("Error searching for cell\n");
        exit(-1);
      } else if (ret == 0 && !go_exit) {
        printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
        if(ntrial == 11)
        {
            printf("Cell not found. Exiting");
            exit(-1);
        }
      }
    } while (ret == 0 && !go_exit);

    if (go_exit) {
      srslte_rf_close(&rf);
      exit(0);
    }

    /* set sampling frequency */
    int srate = srslte_sampling_freq_hz(cell.nof_prb);
    if (srate != -1) {
      if (srate < 10e6) {
        srslte_rf_set_master_clock_rate(&rf, 4 * srate);
      } else {
        srslte_rf_set_master_clock_rate(&rf, srate);
      }
      printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
      float srate_rf = srslte_rf_set_rx_srate(&rf, (double) srate);
      if (srate_rf != srate) {
        ERROR("Could not set sampling rate\n");
        exit(-1);
      }
    } else {
      ERROR("Invalid number of PRB %d\n", cell.nof_prb);
      exit(-1);
    }

    INFO("Stopping RF and flushing buffer...\r");
  }
#endif

  /* If reading from file, go straight to PDSCH decoding. Otherwise, decode MIB first */
  if (prog_args.input_file_name) {
    /* preset cell configuration */
    cell.id              = prog_args.file_cell_id;
    cell.cp              = SRSLTE_CP_NORM;
    cell.phich_length = SRSLTE_PHICH_NORM;
    cell.phich_resources = SRSLTE_PHICH_R_1;
    cell.nof_ports       = prog_args.file_nof_ports;
    cell.nof_prb         = prog_args.file_nof_prb;
    printf("Reading from file\n");

    if (srslte_ue_sync_init_file_multi(&ue_sync,
                                       prog_args.file_nof_prb,
                                       prog_args.input_file_name,
                                       prog_args.file_offset_time,
                                       prog_args.file_offset_freq,
                                       prog_args.rf_nof_rx_ant)) {
      ERROR("Error initiating ue_sync\n");
      exit(-1);
    }

  } else {
#ifndef DISABLE_RF
    int decimate = 0;
    if (prog_args.decimate) {
      if (prog_args.decimate > 4 || prog_args.decimate < 0) {
        printf("Invalid decimation factor, setting to 1 \n");
      } else {
        decimate = prog_args.decimate;
      }
    }
    if (srslte_ue_sync_init_multi_decim(&ue_sync,
                                        cell.nof_prb,
                                        cell.id == 1000,
                                        srslte_rf_recv_wrapper,
                                        prog_args.rf_nof_rx_ant,
                                        (void*)&rf,
                                        decimate)) {
      ERROR("Error initiating ue_sync\n");
      exit(-1);
    }
    if (srslte_ue_sync_set_cell(&ue_sync, cell)) {
      ERROR("Error initiating ue_sync\n");
      exit(-1);
    }
#endif
  }

  for (int i = 0; i < prog_args.rf_nof_rx_ant; i++) {
    sf_buffer[i] = srslte_vec_malloc(3 * sizeof(cf_t) * SRSLTE_SF_LEN_PRB(cell.nof_prb));
  }
  srslte_ue_mib_t ue_mib;
  
//  printf("Initialize ue_mib object....\n");
//  printf("PRB %d\n", cell.nof_prb);
//  printf("PCI %d\n", cell.id);
//  printf("NEW PCI %d\n", ue_mib.pbch.cell.id);
  
  if (srslte_ue_mib_init(&ue_mib, sf_buffer, cell.nof_prb)) {
    ERROR("Error initaiting UE MIB decoder\n");
    exit(-1);
  }
  if (srslte_ue_mib_set_cell(&ue_mib, cell)) {
    ERROR("Error initaiting UE MIB decoder\n");
    exit(-1);
  }
  

  if (srslte_ue_dl_init(&ue_dl, sf_buffer, cell.nof_prb, prog_args.rf_nof_rx_ant)) {
    ERROR("Error initiating UE downlink processing module\n");
    exit(-1);
  }
  if (srslte_ue_dl_set_cell(&ue_dl, cell)) {
    ERROR("Error initiating UE downlink processing module\n");
    exit(-1);
  }

  // Disable CP based CFO estimation during find
  ue_sync.cfo_current_value       = search_cell_cfo / 15000;
  ue_sync.cfo_is_copied           = true;
  ue_sync.cfo_correct_enable_find = true;
  srslte_sync_set_cfo_cp_enable(&ue_sync.sfind, false, 0);

  ZERO_OBJECT(ue_dl_cfg);
  ZERO_OBJECT(dl_sf);
  ZERO_OBJECT(pdsch_cfg);

  if (cell.frame_type == SRSLTE_TDD && prog_args.tdd_special_sf >= 0 && prog_args.sf_config >= 0) {
    dl_sf.tdd_config.ss_config  = prog_args.tdd_special_sf;
    dl_sf.tdd_config.sf_config  = prog_args.sf_config;
    dl_sf.tdd_config.configured = true;
  }

  srslte_chest_dl_cfg_t chest_pdsch_cfg;
  chest_pdsch_cfg.cfo_estimate_enable  = prog_args.enable_cfo_ref;
  chest_pdsch_cfg.cfo_estimate_sf_mask = 1023;
  chest_pdsch_cfg.interpolate_subframe = !prog_args.average_subframe;

  // Special configuration for MBSFN channel estimation
  srslte_chest_dl_cfg_t chest_mbsfn_cfg;
  chest_mbsfn_cfg.filter_type          = SRSLTE_CHEST_FILTER_TRIANGLE;
  chest_mbsfn_cfg.filter_coef[0]       = 0.1;
  chest_mbsfn_cfg.interpolate_subframe = true;
  chest_mbsfn_cfg.noise_alg            = SRSLTE_NOISE_ALG_PSS;

  // Allocate softbuffer buffers
  srslte_softbuffer_rx_t rx_softbuffers[SRSLTE_MAX_CODEWORDS];
  for (uint32_t i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
    pdsch_cfg.softbuffers.rx[i] = &rx_softbuffers[i];
    srslte_softbuffer_rx_init(pdsch_cfg.softbuffers.rx[i], cell.nof_prb);
  }

  pdsch_cfg.rnti = prog_args.rnti;

  /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
  srslte_ue_dl_set_rnti(&ue_dl, prog_args.rnti);

  /* Configure MBSFN area id and non-MBSFN Region */
  if (prog_args.mbsfn_area_id > -1) {
    srslte_ue_dl_set_mbsfn_area_id(&ue_dl, prog_args.mbsfn_area_id);
    srslte_ue_dl_set_non_mbsfn_region(&ue_dl, prog_args.non_mbsfn_region);
  }

#ifdef ENABLE_GUI
  if (!prog_args.disable_plots) {
    init_plots(cell);
    sleep(1);
  }
#endif /* ENABLE_GUI */

#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_rf_start_rx_stream(&rf, false);
  }
#endif

#ifndef DISABLE_RF
  if (prog_args.rf_gain < 0 && !prog_args.input_file_name) {
    srslte_rf_info_t *rf_info = srslte_rf_get_info(&rf);
    srslte_ue_sync_start_agc(&ue_sync,
                             srslte_rf_set_rx_gain_th_wrapper_,
                             rf_info->min_rx_gain,
                             rf_info->max_rx_gain,
                             cell_detect_config.init_agc);
  }
#endif
#ifdef PRINT_CHANGE_SCHEDULING
  srslte_ra_dl_grant_t old_dl_dci;
  bzero(&old_dl_dci, sizeof(srslte_ra_dl_grant_t));
#endif

  ue_sync.cfo_correct_enable_track = !prog_args.disable_cfo;

  srslte_pbch_decode_reset(&ue_mib.pbch);

  INFO("\nEntering main loop...\n\n");
  
  // Variables for measurements
  uint32_t nframes = 0;
  float    rsrp0 = 0.0, rsrp1 = 0.0, rsrq = 0.0, snr = 0.0, enodebrate = 0.0, uerate = 0.0, procrate = 0.0,
        sinr[SRSLTE_MAX_LAYERS][SRSLTE_MAX_CODEBOOKS];
  bool decode_pdsch = false;

  for (int i = 0; i < SRSLTE_MAX_LAYERS; i++) {
    bzero(sinr[i], sizeof(float) * SRSLTE_MAX_CODEBOOKS);
  }

  /* Main loop */
  uint64_t sf_cnt          = 0;
  uint32_t sfn             = 0;
  int cnt             = 0;
  uint8_t temp[20];
//  uint32_t last_decoded_tm = 0;
  while (!go_exit && (sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1)) {
    char input[128];
    PRINT_LINE_INIT();

    fd_set set;
    FD_ZERO(&set);
    FD_SET(0, &set);

    struct timeval to;
    to.tv_sec = 0;
    to.tv_usec = 0;

    /* Set default verbose level */
    srslte_verbose = prog_args.verbose;
    int n = select(1, &set, NULL, NULL, &to);
    if (n == 1) {
      /* If a new line is detected set verbose level to Debug */
      if (fgets(input, sizeof(input), stdin)) {
        srslte_verbose = SRSLTE_VERBOSE_DEBUG;
        pkt_errors     = 0;
        pkt_total      = 0;
        nof_detected   = 0;
        nof_trials = 0;
      }
    }

    cf_t* buffers[SRSLTE_MAX_PORTS] = {};
    for (int p = 0; p < SRSLTE_MAX_PORTS; p++) {
      buffers[p] = sf_buffer[p];
    }
    ret = srslte_ue_sync_zerocopy(&ue_sync, buffers);
    if (ret < 0) {
      ERROR("Error calling srslte_ue_sync_work()\n");
    }

#ifdef CORRECT_SAMPLE_OFFSET
    float sample_offset =
        (float)srslte_ue_sync_get_last_sample_offset(&ue_sync) + srslte_ue_sync_get_sfo(&ue_sync) / 1000;
    srslte_ue_dl_set_sample_offset(&ue_dl, sample_offset);
#endif

    /* srslte_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
    if (ret == 1) {

      bool           acks[SRSLTE_MAX_CODEWORDS] = {false};
      struct timeval t[3];

      uint32_t sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);

      switch (state) {
        case DECODE_MIB:
          if (sf_idx == 0) {
            uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
            int     sfn_offset;
            n = srslte_ue_mib_decode(&ue_mib, bch_payload, NULL, &sfn_offset);
            if (n < 0) {
              ERROR("Error decoding UE MIB\n");
              exit(-1);
            } else if (n == SRSLTE_UE_MIB_FOUND) {
              srslte_pbch_mib_unpack(bch_payload, &cell, &sfn);
              srslte_cell_fprint(stdout, &cell, sfn);
              // PRINT TO LOG 
              FILE *fp;
              fp = fopen("mib.txt", "w");
              srslte_cell_fprint(fp, &cell, sfn);
              fclose(fp);
              printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
              sfn   = (sfn + sfn_offset) % 1024;
              state = DECODE_PDSCH;
            }
          }
          break;
        case DECODE_PDSCH:
          if (prog_args.rnti != SRSLTE_SIRNTI) {
            decode_pdsch = true;
            if (srslte_sfidx_tdd_type(dl_sf.tdd_config, sf_idx) == SRSLTE_TDD_SF_U) {
              decode_pdsch = false;
            }
          } else {
            /* We are looking for SIB1 Blocks, search only in appropriate places */
//            if ((sf_idx == 5 && (sfn % 2) == 0) || mch_table[sf_idx] == 1) {
              if ((sf_idx ==5) || mch_table[sf_idx] ==1) {
              decode_pdsch = true;
            } else {
              decode_pdsch = false;
            }
          }

          uint32_t tti = sfn * 10 + sf_idx;

          gettimeofday(&t[1], NULL);
          if (decode_pdsch) {
            srslte_sf_t sf_type;
            if (mch_table[sf_idx] == 0 || prog_args.mbsfn_area_id < 0) { // Not an MBSFN subframe
              sf_type = SRSLTE_SF_NORM;

              // Set PDSCH channel estimation
              ue_dl_cfg.chest_cfg = chest_pdsch_cfg;
            } else {
              sf_type = SRSLTE_SF_MBSFN;

              // Set MBSFN channel estimation
              ue_dl_cfg.chest_cfg = chest_mbsfn_cfg;
            }

            n = 0;
            for (uint32_t tm = 0; tm < 4 && !n; tm++) {
              dl_sf.tti        = tti;
              dl_sf.sf_type    = sf_type;
              ue_dl_cfg.cfg.tm = (srslte_tm_t)tm;

              if ((ue_dl_cfg.cfg.tm == SRSLTE_TM1 && cell.nof_ports == 1) ||
                  (ue_dl_cfg.cfg.tm > SRSLTE_TM1 && cell.nof_ports > 1)) {
                n = srslte_ue_dl_find_and_decode(&ue_dl, &dl_sf, &ue_dl_cfg, &pdsch_cfg, data, acks);
                
                //On success
                if (n > 0) {
                    
                  int repeat = 0;
                  for (int i = 0; i < sizeof(temp); i++){
                      if(temp[i] == *data[0]){
                          repeat++;
//                          srslte_vec_fprint_byte(stdout, data[0], 18);
                          if (repeat == 3){
                            printf("Found match at count %i\n", cnt);

                            // Decode SIB1
                            printf("SIB1: ");
                            srslte_vec_fprint_byte(stdout, data[0], 18);

                            srslte_ue_sib1_t sib1;
                            srslte_sib1_unpack(data[0], &sib1);
                            srslte_sib1_fprint(stdout, &sib1);

                             //Print to log
                             FILE *fp;
                             fp = fopen("sib1.txt", "w");
                             srslte_sib1_fprint(fp, &sib1);
                             fclose(fp);
                             
                             

                            exit(1);
                          }
                      }
                  }  
                  
                  temp[cnt] = *data[0];
                  cnt++;
//                  if(temp[0] == *data[0])


                  
                  // Decode SIB1
//                    printf("SIB1: ");
//                  srslte_vec_fprint_byte(stdout, data[0], 18);
//                  
//                  srslte_ue_sib1_t sib1;
//                  
//                  srslte_sib1_unpack(data[0], &sib1);
//                  
//                  srslte_sib1_fprint(stdout, &sib1);
//                  
//                  //Print to log
//                  FILE *fp;
//                  fp = fopen("sib1.txt", "w");
//                  srslte_sib1_fprint(fp, &sib1);
//                  fclose(fp);
                  
                  // Exit
//                  printf("\nFinished Decoding... exiting\nGoodbye!\n");
//                  exit(1);
                  
                  nof_detected++;
//                  last_decoded_tm = tm;
                  for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
                    if (pdsch_cfg.grant.tb[tb].enabled) {
                      if (!acks[tb]) {
                        if (sf_type == SRSLTE_SF_NORM) {
                          pkt_errors++;
                        } else {
                          pmch_pkt_errors++;
                        }
                      }
                      if (sf_type == SRSLTE_SF_NORM) {
                        pkt_total++;
                      } else {
                        pmch_pkt_total++;
                      }
                    }
                  }
                }
              }
            }
            // Feed-back ue_sync with chest_dl CFO estimation
            if (sf_idx == 5 && prog_args.enable_cfo_ref) {
              srslte_ue_sync_set_cfo_ref(&ue_sync, ue_dl.chest_res.cfo);
            }

            gettimeofday(&t[2], NULL);
            get_time_interval(t);

            if (n > 0) {

              /* Send data if socket active */
              if (prog_args.net_port > 0) {
                if (sf_idx == 1) {
                  srslte_netsink_write(&net_sink, data[0], 1 + (n - 1) / 8);
                } else {
                  // FIXME: UDP Data transmission does not work
                  for (uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; tb++) {
                    if (pdsch_cfg.grant.tb[tb].enabled) {
                      srslte_netsink_write(&net_sink, data[tb], 1 + (pdsch_cfg.grant.tb[tb].tbs - 1) / 8);
                    }
                  }
                }
              }
#ifdef PRINT_CHANGE_SCHEDULING
              if (pdsch_cfg.dci.cw[0].mcs_idx != old_dl_dci.cw[0].mcs_idx ||
                  memcmp(&pdsch_cfg.dci.type0_alloc, &old_dl_dci.type0_alloc, sizeof(srslte_ra_type0_t)) ||
                  memcmp(&pdsch_cfg.dci.type1_alloc, &old_dl_dci.type1_alloc, sizeof(srslte_ra_type1_t)) ||
                  memcmp(&pdsch_cfg.dci.type2_alloc, &old_dl_dci.type2_alloc, sizeof(srslte_ra_type2_t))) {
                old_dl_dci = pdsch_cfg.dci;
                fflush(stdout);
                printf("DCI %s\n", srslte_dci_format_string(pdsch_cfg.dci.dci_format));
                srslte_ra_pdsch_fprint(stdout, &old_dl_dci, cell.nof_prb);
              }
#endif
            }

            nof_trials++;

            uint32_t enb_bits = ((pdsch_cfg.grant.tb[0].enabled ? pdsch_cfg.grant.tb[0].tbs : 0) +
                                 (pdsch_cfg.grant.tb[1].enabled ? pdsch_cfg.grant.tb[1].tbs : 0));
            uint32_t ue_bits  = ((acks[0] ? pdsch_cfg.grant.tb[0].tbs : 0) + (acks[1] ? pdsch_cfg.grant.tb[1].tbs : 0));
            rsrq              = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_dbm, rsrq, 0.1f);
            rsrp0             = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_port_dbm[0], rsrp0, 0.05f);
            rsrp1             = SRSLTE_VEC_EMA(ue_dl.chest_res.rsrp_port_dbm[1], rsrp1, 0.05f);
            snr               = SRSLTE_VEC_EMA(ue_dl.chest_res.snr_db, snr, 0.05f);
            enodebrate        = SRSLTE_VEC_EMA(enb_bits / 1000.0f, enodebrate, 0.05f);
            uerate            = SRSLTE_VEC_EMA(ue_bits / 1000.0f, uerate, 0.001f);
            float elapsed     = (float)t[0].tv_usec + t[0].tv_sec * 1.0e+6f;
            if (elapsed != 0.0f) {
              procrate = SRSLTE_VEC_EMA(ue_bits / elapsed, procrate, 0.01f);
            }

            nframes++;
            if (isnan(rsrq)) {
              rsrq = 0;
            }
            if (isnan(snr)) {
              snr = 0;
            }
            if (isnan(rsrp0)) {
              rsrp0 = 0;
            }
            if (isnan(rsrp1)) {
              rsrp1 = 0;
            }
          }

          break;
      }
      

     

#ifdef ENABLE_GUI
      if (!prog_args.disable_plots) {
        if ((sfn % 3) == 0 && decode_pdsch) {
          plot_sf_idx = sf_idx;
          plot_track  = true;
          sem_post(&plot_sem);
        }
      }
#endif /* ENABLE_GUI */
    } else if (ret == 0) {
//      printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r",
//             srslte_sync_get_peak_value(&ue_sync.sfind),
//             ue_sync.frame_total_cnt,
//             ue_sync.state);
#ifdef ENABLE_GUI
      if (!prog_args.disable_plots) {
        plot_sf_idx = srslte_ue_sync_get_sfidx(&ue_sync);
        plot_track  = false;
        sem_post(&plot_sem);
      }
#endif /* ENABLE_GUI */
    }

    sf_cnt++;
    
  } // Main loop

#ifdef ENABLE_GUI
  if (!prog_args.disable_plots) {
    if (!pthread_kill(plot_thread, 0)) {
      pthread_kill(plot_thread, SIGHUP);
      pthread_join(plot_thread, NULL);
    }
  }
#endif
  srslte_ue_dl_free(&ue_dl);
  srslte_ue_sync_free(&ue_sync);
  for (int i = 0; i < SRSLTE_MAX_CODEWORDS; i++) {
    if (data[i]) {
      free(data[i]);
    }
  }
  for (int i = 0; i < prog_args.rf_nof_rx_ant; i++) {
    if (sf_buffer[i]) {
      free(sf_buffer[i]);
    }
  }

#ifndef DISABLE_RF
  if (!prog_args.input_file_name) {
    srslte_ue_mib_free(&ue_mib);
    srslte_rf_close(&rf);
  }
#endif

  printf("\nBye\n");
  exit(0);
}